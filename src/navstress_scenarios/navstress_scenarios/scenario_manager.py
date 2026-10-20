"""scenario_manager: put the simulator into the state a ScenarioSpec describes.

Order: reset -> spawn obstacles -> (optional) place robot -> configure noise ->
apply Nav2 overrides -> announce the scenario -> re-seed localization -> clear
costmaps -> settle. Any step that would make the scenario differ from its spec
fails the call; steps that only matter when present (costmap clearing) are skipped
when the service doesn't exist, and that is logged.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from simulation_interfaces.msg import EntityState, Resource, Result
from simulation_interfaces.msg import SpawnEntity as SpawnRequest
from simulation_interfaces.srv import ResetSimulation, SetEntityState, SpawnEntities

from navstress_interfaces.msg import ScenarioSpec as ScenarioSpecMsg
from navstress_interfaces.srv import ApplyScenario
from navstress_scenarios.conversions import msg_to_spec, yaw_to_quaternion
from navstress_scenarios.motion import map_to_world
from navstress_scenarios.obstacle_mover import ACTIVE_SCENARIO_QOS
from navstress_scenarios.rosutil import CallError, RemoteParams, call
from navstress_scenarios.schema import NAV_OVERRIDE_RANGES, ScenarioSpec, ScenarioValidationError
from navstress_scenarios.usd_assets import ensure_asset

# RViz's default initial-pose covariance: 0.5 m std-dev in x/y, ~15 deg in yaw.
_INITIAL_COV = (0.25, 0.25, 0.06853891945200942)


def _parse_targets(entries: list[str]) -> dict[str, tuple[str, str]]:
    """'max_vel_x=/controller_server:FollowPath.max_vel_x' -> {name: (node, param)}."""
    out = {}
    for e in entries:
        if not e:
            continue
        name, _, target = e.partition("=")
        node, _, param = target.partition(":")
        if name not in NAV_OVERRIDE_RANGES or not node or not param:
            raise ValueError(f"bad nav_override_targets entry {e!r}")
        out[name] = (node, param)
    return out


class ScenarioManager(Node):
    def __init__(self) -> None:
        super().__init__("scenario_manager")
        p = self.declare_parameter
        self.timeout_s = p("service_timeout_s", 15.0).value
        self.prefix = p("entity_prefix", "/navstress_").value
        self.asset_dir = p("asset_cache_dir", "/tmp/navstress_assets").value
        self.map_to_world = tuple(p("map_to_world", [0.0, 0.0, 0.0]).value)
        self.robot_entity = p("robot_entity", "").value
        self.noise_nodes = [n for n in p("noise_injector_nodes", ["/noise_injector"]).value if n]
        self.targets = _parse_targets(p("nav_override_targets", [""]).value)
        self.publish_initial_pose = p("publish_initial_pose", True).value
        self.default_start = tuple(p("default_start", [0.0, 0.0, 0.0]).value)
        self.clear_services = [s for s in p("clear_costmap_services", [""]).value if s]
        self.settle_s = p("settle_time_s", 2.0).value

        group = ReentrantCallbackGroup()
        self.reset = self.create_client(ResetSimulation, "/reset_simulation", callback_group=group)
        self.spawn = self.create_client(SpawnEntities, "/spawn_entities", callback_group=group)
        self.set_state = self.create_client(
            SetEntityState, "/set_entity_state", callback_group=group
        )
        self.clear = [
            self.create_client(ClearEntireCostmap, s, callback_group=group)
            for s in self.clear_services
        ]
        self.noise = [RemoteParams(self, n, group, self.timeout_s) for n in self.noise_nodes]
        self.nav_params = {
            name: RemoteParams(self, node, group, self.timeout_s)
            for name, (node, _) in self.targets.items()
        }
        self.baseline_values: dict[str, float] = {}
        self.active_pub = self.create_publisher(
            ScenarioSpecMsg, "/navstress/active_scenario", ACTIVE_SCENARIO_QOS
        )
        # AMCL subscribes to initialpose with SystemDefaultsQoS (reliable, volatile).
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.create_service(
            ApplyScenario, "/navstress/apply_scenario", self._on_apply, callback_group=group
        )

    # ------------------------------------------------------------------ service

    def _on_apply(self, req: ApplyScenario.Request, resp: ApplyScenario.Response):
        t0 = time.monotonic()
        try:
            spec = msg_to_spec(req.scenario)
            self._apply(spec, req.scenario)
        except (CallError, ScenarioValidationError, ValueError, OSError) as exc:
            resp.success, resp.message = False, str(exc)
            self.get_logger().error(f"apply {req.scenario.scenario_id} failed: {exc}")
            return resp
        resp.success = True
        resp.message = f"ready in {time.monotonic() - t0:.1f} s"
        self.get_logger().info(
            f"{spec.scenario_id} [{req.scenario.config_hash[:12]}] {resp.message}"
        )
        return resp

    def _apply(self, spec: ScenarioSpec, msg: ScenarioSpecMsg) -> None:
        if spec.start is not None and not self.robot_entity:
            # Running without the requested start pose would silently test something else.
            raise ValueError("scenario sets a start pose but robot_entity is not configured")
        self._reset()
        self._spawn(spec)
        if spec.start is not None:
            self._place_robot(spec)
        self._configure_noise(spec)
        self._apply_overrides(spec)
        self.active_pub.publish(msg)
        if self.publish_initial_pose:
            self._publish_initial_pose(spec)
        self._clear_costmaps()
        time.sleep(self.settle_s)

    def _check(self, result: Result, what: str) -> None:
        if result.result != Result.RESULT_OK:
            raise CallError(f"{what} failed (code {result.result}): {result.error_message}")

    def _reset(self) -> None:
        req = ResetSimulation.Request(scope=ResetSimulation.Request.SCOPE_DEFAULT)
        self._check(call(self.reset, req, self.timeout_s).result, "reset_simulation")

    def _spawn(self, spec: ScenarioSpec) -> None:
        if not spec.obstacles:
            return
        req = SpawnEntities.Request()
        for o in spec.obstacles:
            uri = ensure_asset(
                self.asset_dir,
                o.asset,
                o.size_x,
                o.size_y,
                o.height,
                kinematic=o.motion is not None,
            )
            pose = PoseStamped()
            x, y, yaw = map_to_world(o.pose.x, o.pose.y, o.pose.yaw, self.map_to_world)
            pose.pose.position.x, pose.pose.position.y = x, y
            pose.pose.orientation = yaw_to_quaternion(yaw)
            req.spawn_requests.append(
                SpawnRequest(
                    name=self.prefix + o.name,
                    allow_renaming=False,  # obstacle_mover addresses entities by this name
                    entity_resource=Resource(uri=str(uri)),
                    initial_pose=pose,
                )
            )
        resp = call(self.spawn, req, self.timeout_s)
        errors = [
            f"{r.entity_name or req.spawn_requests[i].name}: {r.result.error_message}"
            for i, r in enumerate(resp.results)
            if r.result.result != Result.RESULT_OK
        ]
        if errors or resp.result.result != Result.RESULT_OK:
            raise CallError(
                "spawn_entities failed: " + ("; ".join(errors) or resp.result.error_message)
            )

    def _place_robot(self, spec: ScenarioSpec) -> None:
        x, y, yaw = map_to_world(spec.start.x, spec.start.y, spec.start.yaw, self.map_to_world)
        state = EntityState()
        state.pose.position.x, state.pose.position.y = x, y
        state.pose.orientation = yaw_to_quaternion(yaw)
        req = SetEntityState.Request(entity=self.robot_entity, state=state)
        self._check(call(self.set_state, req, self.timeout_s).result, "set_entity_state(robot)")

    def _configure_noise(self, spec: ScenarioSpec) -> None:
        values = {
            "gaussian_std_m": float(spec.sensor_noise.gaussian_std_m),
            "dropout_prob": float(spec.sensor_noise.dropout_prob),
            "seed": int(spec.seed),
        }
        for node in self.noise:
            node.set(values)  # a missing injector is an error: the noise is part of the spec

    def _apply_overrides(self, spec: ScenarioSpec) -> None:
        unknown = set(spec.nav_overrides) - set(self.targets)
        if unknown:
            raise ValueError(f"no nav_override_targets entry for {sorted(unknown)}")
        for name, (node, param) in self.targets.items():
            remote = self.nav_params[name]
            if name not in self.baseline_values:
                if not remote.available():
                    if name in spec.nav_overrides:
                        raise CallError(f"{node} is not available to apply {name}")
                    continue
                # Cache the launch value so later scenarios can restore it.
                self.baseline_values[name] = float(remote.get([param])[param])
            value = spec.nav_overrides.get(name, self.baseline_values[name])
            remote.set({param: float(value)})

    def _publish_initial_pose(self, spec: ScenarioSpec) -> None:
        x, y, yaw = (
            (spec.start.x, spec.start.y, spec.start.yaw)
            if spec.start is not None
            else self.default_start
        )
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation = yaw_to_quaternion(float(yaw))
        cov = [0.0] * 36
        cov[0], cov[7], cov[35] = _INITIAL_COV
        msg.pose.covariance = cov
        self.initial_pose_pub.publish(msg)

    def _clear_costmaps(self) -> None:
        for client in self.clear:
            if not client.wait_for_service(timeout_sec=0.5):
                self.get_logger().info(f"{client.srv_name} not available; not clearing")
                continue
            call(client, ClearEntireCostmap.Request(), self.timeout_s)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScenarioManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
