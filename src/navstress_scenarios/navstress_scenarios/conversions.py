"""ScenarioSpec (validated pydantic model) <-> navstress_interfaces/ScenarioSpec message."""

from __future__ import annotations

import math

from geometry_msgs.msg import Quaternion

from navstress_interfaces.msg import NavOverride, Obstacle, PlanarPose, SensorNoise
from navstress_interfaces.msg import ScenarioSpec as ScenarioSpecMsg
from navstress_scenarios.schema import (
    ScenarioSpec,
    ScenarioValidationError,
    ValidationContext,
    config_hash,
    validate_or_raise,
)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def quaternion_to_yaw(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _planar(p) -> PlanarPose:
    return PlanarPose(x=float(p.x), y=float(p.y), yaw=float(p.yaw))


def spec_to_msg(spec: ScenarioSpec) -> ScenarioSpecMsg:
    msg = ScenarioSpecMsg()
    msg.scenario_id = spec.scenario_id
    msg.family = spec.family
    msg.config_hash = config_hash(spec)
    msg.goal = _planar(spec.goal)
    msg.goal_id = spec.goal_id
    msg.has_start = spec.start is not None
    if spec.start is not None:
        msg.start = _planar(spec.start)
    for o in spec.obstacles:
        m = Obstacle(
            name=o.name,
            asset=o.asset,
            pose=_planar(o.pose),
            size_x=float(o.size_x),
            size_y=float(o.size_y),
            height=float(o.height),
            moving=o.motion is not None,
        )
        if o.motion is not None:
            m.end_x = float(o.motion.end_x)
            m.end_y = float(o.motion.end_y)
            m.speed_mps = float(o.motion.speed_mps)
        msg.obstacles.append(m)
    msg.sensor_noise = SensorNoise(
        gaussian_std_m=float(spec.sensor_noise.gaussian_std_m),
        dropout_prob=float(spec.sensor_noise.dropout_prob),
    )
    msg.nav_overrides = [
        NavOverride(name=k, value=float(v)) for k, v in sorted(spec.nav_overrides.items())
    ]
    msg.seed = int(spec.seed)
    msg.episode_timeout_s = float(spec.episode_timeout_s)
    return msg


def msg_to_dict(msg: ScenarioSpecMsg) -> dict:
    def pose(p: PlanarPose) -> dict:
        return {"x": p.x, "y": p.y, "yaw": p.yaw}

    d = {
        "scenario_id": msg.scenario_id,
        "family": msg.family,
        "goal": pose(msg.goal),
        "goal_id": msg.goal_id,
        "obstacles": [],
        "sensor_noise": {
            "gaussian_std_m": msg.sensor_noise.gaussian_std_m,
            "dropout_prob": msg.sensor_noise.dropout_prob,
        },
        "nav_overrides": {o.name: o.value for o in msg.nav_overrides},
        "seed": msg.seed,
        "episode_timeout_s": msg.episode_timeout_s,
    }
    if msg.has_start:
        d["start"] = pose(msg.start)
    for o in msg.obstacles:
        ob = {
            "name": o.name,
            "asset": o.asset,
            "pose": pose(o.pose),
            "size_x": o.size_x,
            "size_y": o.size_y,
            "height": o.height,
        }
        if o.moving:
            ob["motion"] = {"end_x": o.end_x, "end_y": o.end_y, "speed_mps": o.speed_mps}
        d["obstacles"].append(ob)
    return d


def msg_to_spec(msg: ScenarioSpecMsg, ctx: ValidationContext | None = None) -> ScenarioSpec:
    """Re-validate on arrival: a message is only trusted if it hashes to what it claims."""
    spec = validate_or_raise(msg_to_dict(msg), ctx)
    h = config_hash(spec)
    if msg.config_hash and msg.config_hash != h:
        raise ScenarioValidationError(
            [f"config_hash mismatch: message says {msg.config_hash[:12]}, content is {h[:12]}"]
        )
    return spec
