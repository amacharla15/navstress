"""eval_runner: RunEpisode and RunBatch action servers.

One episode: ApplyScenario -> start bag -> NavigateToPose -> watch until Nav2 finishes,
the robot collides, gets stuck, or the timeout passes -> stop bag -> score -> write
the CSV row. Episodes run one at a time; a second goal while busy is rejected.
"""

from __future__ import annotations

import math
import os
import re
import threading
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from navstress_eval.bag_recorder import BagRecorder
from navstress_eval.conversions import result_to_row
from navstress_eval.episode import PathIntegrator, StuckDetector, classify
from navstress_eval.results_io import (
    RESULTS_CSV,
    SCENARIOS_JSONL,
    append_row,
    collect_metadata,
    make_run_id,
    read_scenario_dicts,
    update_metadata,
    write_metadata,
    write_scenarios,
)
from navstress_interfaces.action import RunBatch, RunEpisode
from navstress_interfaces.msg import Clearance, EpisodeResult
from navstress_interfaces.srv import ApplyScenario
from navstress_scenarios.conversions import msg_to_spec, yaw_to_quaternion
from navstress_scenarios.rosutil import CallError, call, wait_future
from navstress_scenarios.schema import ScenarioValidationError, dump_jsonl

_NAV_STATUS = {
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_ABORTED: "aborted",
    GoalStatus.STATUS_CANCELED: "canceled",
}
_SAFE = re.compile(r"[^A-Za-z0-9_.-]+")


class _Monitor:
    """Episode measurements, fed by subscription callbacks on other executor threads."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.active = False

    def reset(self, stuck_window_s: float, stuck_radius_m: float) -> None:
        with self.lock:
            self.path = PathIntegrator()
            self.stuck = StuckDetector(stuck_window_s, stuck_radius_m)
            self.min_clearance = math.inf
            self.collisions = 0
            self.recoveries = 0
            self.distance_remaining = math.nan
            self.last_pose: tuple[float, float] | None = None
            self.active = True


class EvalRunner(Node):
    def __init__(self) -> None:
        super().__init__("eval_runner")
        p = self.declare_parameter
        self.results_dir = Path(
            p("results_dir", os.environ.get("NAVSTRESS_RESULTS_DIR", "results")).value
        )
        self.simulator = p("simulator", "isaac_sim").value
        self.simulator_version = p(
            "simulator_version", os.environ.get("NAVSTRESS_ISAAC_SIM_VERSION", "")
        ).value
        self.nav2_params_file = p("nav2_params_file", "").value
        self.repo_dir = p("repo_dir", "").value or None
        self.global_frame = p("global_frame", "map").value
        self.bag_topics = list(p("bag_topics", ["/tf", "/tf_static"]).value)
        self.bag_best_effort = list(p("bag_best_effort_topics", [""]).value)
        self.bag_latched = list(p("bag_latched_topics", ["/tf_static"]).value)
        self.bag_delay_s = p("bag_start_delay_s", 1.0).value
        self.apply_timeout_s = p("apply_timeout_s", 120.0).value
        self.accept_timeout_s = p("goal_accept_timeout_s", 30.0).value
        self.stuck_window_s = p("stuck_window_s", 30.0).value
        self.stuck_radius_m = p("stuck_radius_m", 0.25).value
        self.stop_on_collision = p("stop_on_collision", True).value
        self.rate_hz = p("monitor_rate_hz", 10.0).value
        # Wall-clock backstop in case the sim clock stalls: factor * timeout + 60 s.
        self.wall_factor = p("wall_timeout_factor", 3.0).value

        group = ReentrantCallbackGroup()
        self.apply_client = self.create_client(
            ApplyScenario, "/navstress/apply_scenario", callback_group=group
        )
        self.nav_action = p("nav_action", "navigate_to_pose").value
        self.nav = ActionClient(self, NavigateToPose, self.nav_action, callback_group=group)
        self.mon = _Monitor()
        # Best effort matches both best-effort and reliable publishers; Isaac Sim
        # publishes odometry and sensors with Sensor Data QoS.
        self.create_subscription(
            Odometry, p("odom_topic", "odom").value, self._on_odom, qos_profile_sensor_data,
            callback_group=group,
        )  # fmt: skip
        self.create_subscription(
            Clearance, "/navstress/clearance", self._on_clearance, qos_profile_sensor_data,
            callback_group=group,
        )  # fmt: skip
        # Collision events are rare and must not be dropped: reliable, deep queue.
        self.create_subscription(
            Clearance, "/navstress/collision_events", self._on_collision, 50, callback_group=group
        )
        self.result_pub = self.create_publisher(EpisodeResult, "/navstress/episode_results", 10)
        self.busy = threading.Lock()
        common = {
            "callback_group": group,
            "goal_callback": self._on_goal,
            "cancel_callback": lambda _: CancelResponse.ACCEPT,
        }
        self.episode_server = ActionServer(
            self, RunEpisode, "/navstress/run_episode", self._exec_episode, **common
        )
        self.batch_server = ActionServer(
            self, RunBatch, "/navstress/run_batch", self._exec_batch, **common
        )

    # ------------------------------------------------------------------ inputs

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_odom(self, msg: Odometry) -> None:
        with self.mon.lock:
            if not self.mon.active:
                return
            x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
            self.mon.path.add(x, y)
            self.mon.stuck.add(self._now_s(), x, y)

    def _on_clearance(self, msg: Clearance) -> None:
        with self.mon.lock:
            if self.mon.active:
                self.mon.min_clearance = min(self.mon.min_clearance, msg.min_clearance_m)

    def _on_collision(self, msg: Clearance) -> None:
        with self.mon.lock:
            if self.mon.active:
                self.mon.collisions += 1

    def _on_nav_feedback(self, fb_msg) -> None:
        fb = fb_msg.feedback
        with self.mon.lock:
            if self.mon.active:
                self.mon.recoveries = max(self.mon.recoveries, fb.number_of_recoveries)
                self.mon.distance_remaining = fb.distance_remaining
                p = fb.current_pose.pose.position
                self.mon.last_pose = (p.x, p.y)

    def _on_goal(self, _goal) -> GoalResponse:
        if self.busy.locked():
            self.get_logger().warn("rejecting goal: an episode is already running")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # ------------------------------------------------------------------ episode

    def run_episode(
        self,
        scenario,
        run_id: str,
        run_dir: Path | None,
        index: int,
        record_bag: bool,
        publish_fb,
        cancel_requested,
    ) -> EpisodeResult | None:
        """Run one episode. Returns None only if the client cancelled it."""
        wall0 = time.monotonic()
        res = EpisodeResult(
            run_id=run_id, scenario_id=scenario.scenario_id, family=scenario.family,
            config_hash=scenario.config_hash, seed=scenario.seed, min_clearance_m=math.inf,
            final_distance_to_goal_m=math.inf,
        )  # fmt: skip

        def fail_setup(why: str) -> EpisodeResult:
            res.success, res.failure_reason = False, EpisodeResult.FAILURE_SETUP
            res.nav2_error_msg = f"setup: {why}"[:500]
            res.wall_time_s = time.monotonic() - wall0
            self.get_logger().error(f"{scenario.scenario_id}: {why}")
            return res

        publish_fb("applying", 0.0)
        try:
            resp = call(self.apply_client, ApplyScenario.Request(scenario=scenario),
                        self.apply_timeout_s)  # fmt: skip
        except CallError as exc:
            return fail_setup(str(exc))
        if not resp.success:
            return fail_setup(resp.message)
        if not self.nav.wait_for_server(timeout_sec=self.accept_timeout_s):
            return fail_setup(f"{self.nav_action} action server not available")

        recorder = None
        if record_bag and run_dir is not None:
            name = f"{index:03d}_{_SAFE.sub('_', scenario.scenario_id)}_{scenario.config_hash[:8]}"
            recorder = BagRecorder(
                self.bag_topics,
                self.get_parameter("use_sim_time").value,
                self.bag_delay_s,
                self.bag_best_effort,
                self.bag_latched,
            )
            publish_fb("recording", 0.0)
            try:
                recorder.start(run_dir / "bags" / name)
            except (OSError, RuntimeError) as exc:
                return fail_setup(f"bag recorder: {exc}")
            res.bag_path = f"bags/{name}"

        try:
            outcome = self._drive(scenario, res, publish_fb, cancel_requested)
        finally:
            if recorder is not None:
                recorder.stop()
        if outcome is None:
            return None
        res.wall_time_s = time.monotonic() - wall0
        return res

    def _drive(self, scenario, res: EpisodeResult, publish_fb, cancel_requested):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = scenario.goal.x, scenario.goal.y
        goal.pose.pose.orientation = yaw_to_quaternion(scenario.goal.yaw)

        self.mon.reset(self.stuck_window_s, self.stuck_radius_m)
        nav_status, stopped_for, error_code, error_msg = "", "", 0, ""
        t0 = self._now_s()
        wall_cap = self.wall_factor * scenario.episode_timeout_s + 60.0
        wall0 = time.monotonic()
        publish_fb("navigating", 0.0)
        try:
            handle = wait_future(
                self.nav.send_goal_async(goal, feedback_callback=self._on_nav_feedback),
                self.accept_timeout_s, "NavigateToPose goal",
            )  # fmt: skip
        except CallError as exc:
            handle, nav_status, error_msg = None, "rejected", str(exc)
        if handle is not None and not handle.accepted:
            handle, nav_status, error_msg = None, "rejected", "goal rejected by Nav2"

        if handle is not None:
            result_future = handle.get_result_async()
            period = 1.0 / self.rate_hz
            while not result_future.done():
                elapsed = self._now_s() - t0
                with self.mon.lock:
                    collided = self.mon.collisions > 0
                    stuck = self.mon.stuck.stuck()
                    remaining, clearance = self.mon.distance_remaining, self.mon.min_clearance
                if cancel_requested():
                    stopped_for = "client"
                elif self.stop_on_collision and collided:
                    stopped_for = EpisodeResult.FAILURE_COLLISION
                elif stuck:
                    stopped_for = EpisodeResult.FAILURE_STUCK
                elif (elapsed > scenario.episode_timeout_s
                      or time.monotonic() - wall0 > wall_cap):  # fmt: skip
                    stopped_for = EpisodeResult.FAILURE_TIMEOUT
                if stopped_for:
                    handle.cancel_goal_async()
                    try:
                        wait_future(result_future, 15.0, "NavigateToPose cancel")
                    except CallError as exc:
                        self.get_logger().warn(str(exc))
                    break
                publish_fb("navigating", elapsed, remaining, clearance)
                time.sleep(period)
            if result_future.done() and result_future.result() is not None:
                wrapped = result_future.result()
                nav_status = _NAV_STATUS.get(wrapped.status, "")
                error_code = int(wrapped.result.error_code)
                error_msg = wrapped.result.error_msg

        t_end = self._now_s()
        with self.mon.lock:
            self.mon.active = False
            m = self.mon
            res.path_length_m = m.path.length_m
            res.min_clearance_m = m.min_clearance
            res.collisions = m.collisions
            res.recoveries_triggered = m.recoveries
            if m.last_pose is not None:
                res.final_distance_to_goal_m = math.hypot(
                    m.last_pose[0] - scenario.goal.x, m.last_pose[1] - scenario.goal.y
                )
        if stopped_for == "client":
            return None
        out = classify(nav_status, res.collisions, stopped_for)
        res.success, res.failure_reason = out.success, out.failure_reason
        res.nav2_error_code, res.nav2_error_msg = error_code, error_msg[:500]
        res.time_to_goal_s = t_end - t0
        return res

    # ------------------------------------------------------------------ servers

    def _record(self, run_dir: Path | None, res: EpisodeResult) -> None:
        self.result_pub.publish(res)
        if run_dir is not None:
            append_row(run_dir, result_to_row(res))
        verdict = "OK" if res.success else f"FAIL {res.failure_reason}"
        self.get_logger().info(
            f"{res.scenario_id} [{res.config_hash[:12]}] {verdict} "
            f"t={res.time_to_goal_s:.1f}s path={res.path_length_m:.2f}m "
            f"clear={res.min_clearance_m:.2f}m"
        )

    def _start_run(self, run_id: str, label: str, specs, n_planned: int, bags: bool) -> Path:
        run_dir = self.results_dir / run_id
        run_dir.mkdir(parents=True, exist_ok=True)
        existing = {d["config_hash"] for d in read_scenario_dicts(run_dir)}
        new = [s for s in specs if s.config_hash not in existing]
        if new or not existing:
            old = (run_dir / SCENARIOS_JSONL).read_text() if existing else ""
            write_scenarios(run_dir, old + dump_jsonl(new))
        if not (run_dir / "run_metadata.json").exists():
            write_metadata(
                run_dir,
                collect_metadata(
                    run_id, label, self.simulator, self.simulator_version,
                    self.nav2_params_file, self.repo_dir,
                    extra={"episodes_planned": n_planned, "record_bags": bags},
                ),
            )  # fmt: skip
        return run_dir

    def _validated(self, scenarios):
        try:
            return [msg_to_spec(m) for m in scenarios]
        except ScenarioValidationError as exc:
            raise ValueError(f"invalid scenario in goal: {exc}") from exc

    def _exec_episode(self, goal_handle):
        with self.busy:
            g = goal_handle.request
            try:
                spec = self._validated([g.scenario])[0]
            except ValueError as exc:
                self.get_logger().error(str(exc))
                goal_handle.abort()
                return RunEpisode.Result()
            run_dir = (
                self._start_run(g.run_id, "run_episode", [spec], 1, g.record_bag)
                if g.run_id
                else None
            )
            index = 0
            if run_dir is not None and (run_dir / RESULTS_CSV).exists():
                index = len((run_dir / RESULTS_CSV).read_text().splitlines()) - 1

            def fb(phase, elapsed, remaining=math.nan, clearance=math.inf):
                goal_handle.publish_feedback(
                    RunEpisode.Feedback(phase=phase, elapsed_s=float(elapsed),
                                        distance_remaining_m=float(remaining),
                                        min_clearance_m=float(clearance))
                )  # fmt: skip

            res = self.run_episode(g.scenario, g.run_id, run_dir, index, g.record_bag, fb,
                                   lambda: goal_handle.is_cancel_requested)  # fmt: skip
            if res is None:
                goal_handle.canceled()
                return RunEpisode.Result()
            self._record(run_dir, res)
            goal_handle.succeed()
            return RunEpisode.Result(result=res)

    def _exec_batch(self, goal_handle):
        with self.busy:
            g = goal_handle.request
            run_id = g.run_id or make_run_id(g.label or "batch")
            try:
                specs = self._validated(g.scenarios)
                if (self.results_dir / run_id / RESULTS_CSV).exists():
                    raise ValueError(f"results/{run_id} already has results; use a new run_id")
            except ValueError as exc:
                self.get_logger().error(str(exc))
                goal_handle.abort()
                return RunBatch.Result(run_id=run_id)
            run_dir = self._start_run(run_id, g.label, specs, len(specs), g.record_bags)
            self.get_logger().info(f"batch {run_id}: {len(specs)} episodes -> {run_dir}")
            results: list[EpisodeResult] = []
            canceled = False
            for i, scenario in enumerate(g.scenarios):
                if goal_handle.is_cancel_requested:
                    canceled = True
                    break

                def fb(phase, elapsed, remaining=math.nan, clearance=math.inf, i=i):
                    goal_handle.publish_feedback(
                        RunBatch.Feedback(episodes_done=i, episodes_total=len(specs),
                                          last_result=results[-1] if results else EpisodeResult())
                    )  # fmt: skip

                res = self.run_episode(scenario, run_id, run_dir, i, g.record_bags, fb,
                                       lambda: goal_handle.is_cancel_requested)  # fmt: skip
                if res is None:
                    canceled = True
                    break
                self._record(run_dir, res)
                results.append(res)
            update_metadata(
                run_dir,
                finished_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                episodes_completed=len(results),
                canceled=canceled,
            )
            out = RunBatch.Result(
                run_id=run_id,
                results_path=str(run_dir / RESULTS_CSV),
                episodes_completed=len(results),
                successes=sum(r.success for r in results),
                results=results,
            )
            if canceled:
                goal_handle.canceled()
            else:
                goal_handle.succeed()
            return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EvalRunner()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
