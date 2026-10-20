"""stress_agent: run the LLM stress-test loop against a running eval_runner.

ros2 run navstress_agent stress_agent --goals goals.yaml --mock
ros2 run navstress_agent stress_agent --list-models     # needs NAVSTRESS_LLM_* set
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import rclpy

from navstress_agent.llm import LLMError, MockLLM, OpenAICompatibleClient
from navstress_agent.loop import Budget, StressAgent
from navstress_eval.batch_client import BatchClient, spin_in_background
from navstress_eval.results_io import make_run_id, read_rows, results_root
from navstress_scenarios.goals import GoalSet


def load_env_file(path: Path) -> None:
    """KEY=VALUE lines from .env, without overriding variables already set."""
    if not path.exists():
        return
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip('"').strip("'"))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="NavStress LLM stress-test agent.")
    ap.add_argument("--goals", type=Path, help="reachable goal-set YAML")
    ap.add_argument(
        "--start",
        nargs=3,
        type=float,
        metavar=("X", "Y", "YAW"),
        help="robot start pose after reset (default: goal set's start_xy, yaw 0)",
    )
    ap.add_argument("--rounds", type=int, default=2)
    ap.add_argument("--episodes", type=int, default=40, help="max episodes per round")
    ap.add_argument("--llm-calls", type=int, default=4, help="max LLM calls in total")
    ap.add_argument("--mock", action="store_true", help="use the deterministic mock LLM")
    ap.add_argument("--list-models", action="store_true", help="list endpoint models and exit")
    ap.add_argument("--baseline", type=Path, help="results dir whose rows seed round 1")
    ap.add_argument("--run-id", default="")
    ap.add_argument("--results-dir", type=Path, default=None)
    ap.add_argument("--env-file", type=Path, default=Path(".env"))
    ap.add_argument("--no-bags", action="store_true")
    argv = rclpy.utilities.remove_ros_args(argv if argv is not None else sys.argv)[1:]
    args = ap.parse_args(argv)  # fmt: skip
    load_env_file(args.env_file)

    if args.list_models:
        try:
            print("\n".join(OpenAICompatibleClient.from_env().list_models()))
        except LLMError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2
        return 0
    if args.goals is None:
        ap.error("--goals is required")
    try:
        llm = MockLLM() if args.mock else OpenAICompatibleClient.from_env()
    except LLMError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    goal_set = GoalSet.from_yaml(args.goals)
    if args.start:
        start = tuple(args.start)
    else:
        sx, sy = goal_set.meta.get("start_xy", [0.0, 0.0])
        start = (float(sx), float(sy), 0.0)
    root = args.results_dir or results_root()
    run_id = args.run_id or make_run_id("agent")
    prior = read_rows(args.baseline) if args.baseline else None

    rclpy.init()
    node = rclpy.create_node("navstress_stress_agent")
    executor, _ = spin_in_background(node)
    try:
        agent = StressAgent(
            llm=llm,
            run_batch=BatchClient(node, record_bags=not args.no_bags),
            goal_set=goal_set,
            default_start=start,
            out_dir=root / run_id,
            run_id=run_id,
            budget=Budget(args.rounds, args.episodes, args.llm_calls),
        )
        result = agent.run(prior)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    for r in result.rounds:
        focus = f" focus={r.focus_family}" if r.focus_family else ""
        print(f"round {r.round}{focus}: {r.episodes} episodes -> {r.batch_run_id}")
        for fam, s in r.summaries.items():
            print(f"  {fam}: {s['successes']}/{s['episodes']} succeeded")
        if r.boundary:
            print(f"  boundary: {r.boundary}")
    print(f"stopped: {result.stop_reason}; log: {root / run_id / 'agent_log.jsonl'}")
    return 0
