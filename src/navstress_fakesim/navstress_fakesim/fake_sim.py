"""fake_sim: stands in for Isaac Sim + Nav2 in CI (no GPU).

Serves the subset of simulation_interfaces NavStress uses, publishes a clean 2D scan,
odometry and TF, and runs a fake NavigateToPose server that drives the World model.
Its outcomes are produced by the deliberately crude rules in world.py and the noise
rule below. They exercise the pipeline; they are not evidence about Nav2.
"""

from __future__ import annotations

import math
import os
import random
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from simulation_interfaces.msg import EntityState, Result, SimulatorFeatures, SpawnResult
from simulation_interfaces.srv import (
    DeleteEntity,
    GetEntities,
    GetEntityState,
    GetSimulatorFeatures,
    ResetSimulation,
    SetEntityState,
    SpawnEntities,
    SpawnEntity,
)
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from navstress_fakesim.world import Thing, World, WorldConfig
from navstress_interfaces.msg import ScenarioSpec
from navstress_scenarios.conversions import quaternion_to_yaw, yaw_to_quaternion
from navstress_scenarios.obstacle_mover import ACTIVE_SCENARIO_QOS
from navstress_scenarios.usd_assets import read_metadata


def _ok() -> Result:
    return Result(result=Result.RESULT_OK)


def _err(code: int, msg: str) -> Result:
    return Result(result=code, error_message=msg)


class FakeSim(Node):
    def __init__(self) -> None:
        super().__init__("fake_sim")
        p = self.declare_parameter
        default_world = os.path.join(
            get_package_share_directory("navstress_fakesim"), "config", "world.yaml"
        )
        with open(p("world_file", default_world).value) as f:
            self.world = World(WorldConfig.from_dict(yaml.safe_load(f)))
        self.robot_entity = p("robot_entity", "/fake_robot").value
        self.speed = p("speed_mps", 1.5).value
        self.dt = 1.0 / p("rate_hz", 20.0).value
        self.goal_tol = p("xy_goal_tolerance", 0.25).value
        self.blocked_recovery_s = p("blocked_recovery_s", 1.5).value
        self.max_recoveries = p("max_recoveries", 2).value
        self.replan_s = p("replan_period_s", 1.0).value
        # Fake noise rule: P(abort) = std_gain * std + dropout_gain * dropout, capped.
        self.std_gain = p("noise_abort_gain_std", 6.0).value
        self.dropout_gain = p("noise_abort_gain_dropout", 1.0).value

        self.lock = threading.RLock()
        self.v = 0.0
        self.nav_active = False
        self.reset_count = 0
        self.noise = (0.0, 0.0, 0)
        group = ReentrantCallbackGroup()
        for srv_type, name, cb in (
            (GetSimulatorFeatures, "/get_simulator_features", self._features),
            (ResetSimulation, "/reset_simulation", self._reset),
            (SpawnEntities, "/spawn_entities", self._spawn_many),
            (SpawnEntity, "/spawn_entity", self._spawn_one),
            (DeleteEntity, "/delete_entity", self._delete),
            (SetEntityState, "/set_entity_state", self._set_state),
            (GetEntityState, "/get_entity_state", self._get_state),
            (GetEntities, "/get_entities", self._entities),
        ):
            self.create_service(srv_type, name, cb, callback_group=group)
        self.create_subscription(
            ScenarioSpec, "/navstress/active_scenario", self._on_scenario, ACTIVE_SCENARIO_QOS
        )
        # Same QoS as Isaac Sim's sensor publishers: best effort.
        self.scan_pub = self.create_publisher(LaserScan, "scan_raw", qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, "odom", qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self._publish_static_tf()
        self.create_timer(self.dt, self._publish_state, callback_group=group)
        self.create_timer(1.0 / p("scan_rate_hz", 10.0).value, self._publish_scan,
                          callback_group=group)  # fmt: skip
        self.nav_server = ActionServer(
            self, NavigateToPose, "navigate_to_pose", self._navigate, callback_group=group,
            goal_callback=self._nav_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
        )  # fmt: skip

    # ------------------------------------------------------------ simulation_interfaces

    def _features(self, _req, resp):
        f = SimulatorFeatures
        resp.features.features = [
            f.SPAWNING, f.SPAWNING_ENTITIES, f.DELETING, f.ENTITY_STATE_GETTING,
            f.ENTITY_STATE_SETTING, f.SIMULATION_RESET,
        ]  # fmt: skip
        resp.features.spawn_formats = ["usda"]
        resp.features.custom_info = "navstress_fakesim: CI test double, not a simulator"
        return resp

    def _reset(self, _req, resp):
        with self.lock:
            self.world.reset()
            self.v = 0.0
            self.reset_count += 1  # aborts a navigation that is still running
        resp.result = _ok()
        return resp

    def _spawn_entry(self, name: str, allow_renaming: bool, uri: str, pose) -> SpawnResult:
        if not uri:
            return SpawnResult(result=_err(SpawnResult.NO_RESOURCE, "uri is empty"))
        try:
            meta = read_metadata(uri)
        except (OSError, ValueError) as exc:
            return SpawnResult(result=_err(SpawnResult.RESOURCE_PARSE_ERROR, str(exc)))
        entity = name if name.startswith("/") else "/" + name
        with self.lock:
            if entity in self.world.things or entity == self.robot_entity:
                if not allow_renaming:
                    return SpawnResult(
                        result=_err(SpawnResult.NAME_NOT_UNIQUE, f"{entity} exists")
                    )
                k = 1
                while f"{entity}_{k}" in self.world.things:
                    k += 1
                entity = f"{entity}_{k}"
            self.world.things[entity] = Thing(
                entity, meta["asset"], pose.position.x, pose.position.y,
                quaternion_to_yaw(pose.orientation), meta["size_x"], meta["size_y"],
                meta["height"],
            )  # fmt: skip
        return SpawnResult(result=_ok(), entity_name=entity)

    def _spawn_many(self, req, resp):
        for r in req.spawn_requests:
            resp.results.append(
                self._spawn_entry(
                    r.name, r.allow_renaming, r.entity_resource.uri, r.initial_pose.pose
                )  # fmt: skip
            )
        failed = any(r.result.result != Result.RESULT_OK for r in resp.results)
        resp.result = (
            _err(SpawnEntities.Response.ENTITIES_SPAWN_FAILED, "see results") if failed else _ok()
        )
        return resp

    def _spawn_one(self, req, resp):
        r = self._spawn_entry(req.name, req.allow_renaming, req.uri, req.initial_pose.pose)
        resp.result, resp.entity_name = r.result, r.entity_name
        return resp

    def _delete(self, req, resp):
        with self.lock:
            found = self.world.things.pop(req.entity, None) is not None
        resp.result = _ok() if found else _err(Result.RESULT_NOT_FOUND, req.entity)
        return resp

    def _set_state(self, req, resp):
        pose = req.state.pose
        with self.lock:
            if req.entity == self.robot_entity:
                self.world.x, self.world.y = pose.position.x, pose.position.y
                self.world.yaw = quaternion_to_yaw(pose.orientation)
            elif req.entity in self.world.things:
                t = self.world.things[req.entity]
                t.x, t.y, t.yaw = (
                    pose.position.x,
                    pose.position.y,
                    quaternion_to_yaw(pose.orientation),
                )
            else:
                resp.result = _err(Result.RESULT_NOT_FOUND, f"{req.entity} does not exist")
                return resp
        resp.result = _ok()
        return resp

    def _get_state(self, req, resp):
        with self.lock:
            if req.entity == self.robot_entity:
                x, y, yaw = self.world.x, self.world.y, self.world.yaw
            elif req.entity in self.world.things:
                t = self.world.things[req.entity]
                x, y, yaw = t.x, t.y, t.yaw
            else:
                resp.result = _err(Result.RESULT_NOT_FOUND, req.entity)
                return resp
        resp.state = EntityState()
        resp.state.pose.position.x, resp.state.pose.position.y = x, y
        resp.state.pose.orientation = yaw_to_quaternion(yaw)
        resp.result = _ok()
        return resp

    def _entities(self, req, resp):
        with self.lock:
            names = [self.robot_entity, *self.world.things]
        resp.entities = [n for n in names if req.filters.filter in n]
        resp.result = _ok()
        return resp

    def _on_scenario(self, msg: ScenarioSpec) -> None:
        with self.lock:
            self.noise = (msg.sensor_noise.gaussian_std_m, msg.sensor_noise.dropout_prob,
                          msg.seed)  # fmt: skip

    # ------------------------------------------------------------ sensors

    def _publish_static_tf(self) -> None:
        stamp = self.get_clock().now().to_msg()
        out = []
        for parent, child, z in (("map", "odom", 0.0), ("base_link", "laser", 0.2)):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.translation.z = z
            t.transform.rotation.w = 1.0
            out.append(t)
        self.static_tf.sendTransform(out)

    def _publish_state(self) -> None:
        with self.lock:
            x, y, yaw, v = self.world.x, self.world.y, self.world.yaw, self.v
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id, t.child_frame_id = "odom", "base_link"
        t.transform.translation.x, t.transform.translation.y = x, y
        t.transform.rotation = yaw_to_quaternion(yaw)
        self.tf.sendTransform(t)
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id, odom.child_frame_id = "odom", "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = v
        self.odom_pub.publish(odom)

    def _publish_scan(self) -> None:
        with self.lock:
            ranges = self.world.scan()
        n = len(ranges)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_min = -math.pi
        msg.angle_increment = 2.0 * math.pi / n
        msg.angle_max = msg.angle_min + (n - 1) * msg.angle_increment
        msg.range_min = 0.05
        msg.range_max = self.world.cfg.lidar_range
        msg.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(msg)

    # ------------------------------------------------------------ fake Nav2

    def _nav_goal(self, _goal) -> GoalResponse:
        return GoalResponse.REJECT if self.nav_active else GoalResponse.ACCEPT

    def _navigate(self, goal_handle):
        self.nav_active = True
        try:
            return self._drive(goal_handle)
        finally:
            with self.lock:
                self.v = 0.0
            self.nav_active = False

    def _drive(self, goal_handle):
        pose = goal_handle.request.pose.pose
        gx, gy = pose.position.x, pose.position.y
        result = NavigateToPose.Result()
        with self.lock:
            reset_at_start = self.reset_count
            std, dropout, seed = self.noise
            path = self.world.plan(gx, gy)
        # Fake noise rule: decide up front, from the scenario seed, whether this episode
        # "loses localization", and at what fraction of the way.
        rng = random.Random(seed)
        p_abort = min(0.95, self.std_gain * std + self.dropout_gain * dropout)
        abort_after = rng.uniform(0.3, 0.7) if rng.random() < p_abort else None
        if path is None:
            result.error_msg = "fakesim: no path to goal"
            goal_handle.abort()
            return result
        start_dist = math.hypot(gx - self.world.x, gy - self.world.y)
        t0 = time.monotonic()
        recoveries = 0
        blocked_since = None
        last_plan = time.monotonic()
        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return result
            with self.lock:
                if self.reset_count != reset_at_start:
                    result.error_msg = "fakesim: simulation reset during navigation"
                    goal_handle.abort()
                    return result
                x, y = self.world.x, self.world.y
                remaining = math.hypot(gx - x, gy - y)
                if remaining <= self.goal_tol:
                    goal_handle.succeed()
                    return result
                if abort_after is not None and remaining < (1.0 - abort_after) * start_dist:
                    result.error_msg = "fakesim: noise rule triggered (localization lost)"
                    goal_handle.abort()
                    return result
                if time.monotonic() - last_plan > self.replan_s:
                    path = self.world.plan(gx, gy) or path
                    last_plan = time.monotonic()
                while path and math.hypot(path[0][0] - x, path[0][1] - y) < 0.05:
                    path.pop(0)
                tx, ty = path[0] if path else (gx, gy)
                heading = math.atan2(ty - y, tx - x)
                step = min(self.speed * self.dt, math.hypot(tx - x, ty - y))
                nx, ny = x + step * math.cos(heading), y + step * math.sin(heading)
                if self.world.blocked(nx, ny, heading):
                    self.v = 0.0
                    blocked_since = blocked_since or time.monotonic()
                    if time.monotonic() - blocked_since > self.blocked_recovery_s:
                        # Like Nav2's recovery behaviours: try again, then give up.
                        recoveries += 1
                        blocked_since = None
                        if recoveries > self.max_recoveries:
                            result.error_msg = "fakesim: blocked, recoveries exhausted"
                            goal_handle.abort()
                            return result
                        path = self.world.plan(gx, gy) or path
                else:
                    blocked_since = None
                    self.world.x, self.world.y, self.world.yaw = nx, ny, heading
                    self.v = step / self.dt
            fb = NavigateToPose.Feedback()
            fb.current_pose.header.frame_id = "map"
            fb.current_pose.header.stamp = self.get_clock().now().to_msg()
            fb.current_pose.pose.position.x, fb.current_pose.pose.position.y = x, y
            fb.current_pose.pose.orientation = yaw_to_quaternion(heading)
            elapsed = time.monotonic() - t0
            fb.navigation_time.sec = int(elapsed)
            fb.navigation_time.nanosec = int((elapsed % 1.0) * 1e9)
            fb.number_of_recoveries = recoveries
            fb.distance_remaining = float(remaining)
            goal_handle.publish_feedback(fb)
            time.sleep(self.dt)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeSim()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
