"""RunBatch action client, shared by the run_batch CLI and the stress agent."""

from __future__ import annotations

import argparse
import sys
import threading
from collections.abc import Callable
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from navstress_eval.conversions import result_to_row
from navstress_interfaces.action import RunBatch
from navstress_scenarios.conversions import spec_to_msg
from navstress_scenarios.goals import GoalSet
from navstress_scenarios.rosutil import wait_future
from navstress_scenarios.schema import ScenarioSpec, ValidationContext, load_jsonl


class BatchError(RuntimeError):
    pass


class BatchClient:
    """Sends RunBatch goals and waits for them. The node must be spinning elsewhere."""

    def __init__(self, node: Node, server: str = "/navstress/run_batch", record_bags: bool = True):
        self.node = node
        self.record_bags = record_bags
        self.client = ActionClient(node, RunBatch, server)
        self.on_feedback: Callable[[RunBatch.Feedback], None] | None = None

    def run(self, specs: list[ScenarioSpec], run_id: str, label: str) -> RunBatch.Result:
        if not self.client.wait_for_server(timeout_sec=60.0):
            raise BatchError("RunBatch server not available (is eval_runner running?)")
        goal = RunBatch.Goal(
            scenarios=[spec_to_msg(s) for s in specs],
            run_id=run_id,
            label=label,
            record_bags=self.record_bags,
        )
        fb = (lambda m: self.on_feedback(m.feedback)) if self.on_feedback else None
        handle = wait_future(self.client.send_goal_async(goal, feedback_callback=fb), 30.0, "goal")
        if not handle.accepted:
            raise BatchError("RunBatch goal rejected (another batch running?)")
        # A batch can take hours on the real simulator; no client-side timeout.
        wrapped = wait_future(handle.get_result_async(), float("inf"), "RunBatch result")
        if wrapped.status != 4:  # GoalStatus.STATUS_SUCCEEDED
            raise BatchError(f"RunBatch {run_id} ended with status {wrapped.status}")
        return wrapped.result

    def __call__(self, specs: list[ScenarioSpec], run_id: str, label: str) -> list[dict]:
        """The run_batch function the agent loop expects."""
        return [result_to_row(r) for r in self.run(specs, run_id, label).results]


def spin_in_background(node: Node) -> tuple[MultiThreadedExecutor, threading.Thread]:
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return executor, thread


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Run a scenarios.jsonl file as one batch.")
    ap.add_argument("--scenarios", required=True, type=Path)
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--label", default="")
    ap.add_argument("--goals", type=Path, help="goal-set YAML; validates goals are in the set")
    ap.add_argument("--no-bags", action="store_true")
    args = ap.parse_args(
        rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:]
    )

    ctx = ValidationContext(goal_set=GoalSet.from_yaml(args.goals)) if args.goals else None
    specs = load_jsonl(args.scenarios.read_text(), ctx)
    rclpy.init()
    node = rclpy.create_node("navstress_run_batch")
    executor, _ = spin_in_background(node)
    client = BatchClient(node, record_bags=not args.no_bags)
    client.on_feedback = lambda f: print(f"  {f.episodes_done}/{f.episodes_total} done", end="\r")
    try:
        result = client.run(specs, args.run_id, args.label or args.run_id)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    print(
        f"{result.run_id}: {result.successes}/{result.episodes_completed} succeeded -> "
        f"{result.results_path}"
    )
    return 0
