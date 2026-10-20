"""Baseline scenario set: N episodes to seeded-random goals from the reachable set."""

from __future__ import annotations

import argparse
import random
from pathlib import Path

from navstress_scenarios.goals import GoalSet
from navstress_scenarios.schema import (
    ScenarioSpec,
    ValidationContext,
    dump_jsonl,
    validate_or_raise,
)


def make_baseline(
    goal_set: GoalSet,
    n: int,
    seed: int,
    family: str = "baseline",
    episode_timeout_s: float = 120.0,
) -> list[ScenarioSpec]:
    """Same (goal_set, n, seed) always gives the same scenarios and hashes."""
    if len(goal_set) == 0:
        raise ValueError("goal set is empty")
    rng = random.Random(seed)
    ids = goal_set.ids()
    ctx = ValidationContext(goal_set=goal_set)
    specs = []
    for i in range(n):
        raw = {
            "scenario_id": f"{family}/{i:03d}",
            "family": family,
            "goal_id": rng.choice(ids),
            "seed": rng.randrange(2**31 - 1),
            "episode_timeout_s": episode_timeout_s,
        }
        specs.append(validate_or_raise(raw, ctx))
    return specs


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Write a baseline scenarios.jsonl.")
    ap.add_argument("--goals", required=True, help="goal-set YAML")
    ap.add_argument("--n", type=int, default=50)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--timeout", type=float, default=120.0, help="episode timeout, sim seconds")
    ap.add_argument("-o", "--output", required=True)
    args = ap.parse_args(argv)
    specs = make_baseline(
        GoalSet.from_yaml(args.goals), args.n, args.seed, "baseline", args.timeout
    )
    Path(args.output).write_text(dump_jsonl(specs))
    print(f"wrote {len(specs)} scenarios to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
