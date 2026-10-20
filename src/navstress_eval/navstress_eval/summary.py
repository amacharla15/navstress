"""Per-family summaries, weakest-family choice, and sweep boundary estimation.

These are plain functions over result rows (dicts with navstress_eval RESULT_FIELDS),
so the agent and the report compute the same numbers the same way.
"""

from __future__ import annotations

import math
import statistics
from collections import Counter
from collections.abc import Iterable, Mapping
from dataclasses import asdict, dataclass, field
from typing import Any

from navstress_scenarios.schema import get_path

# Failure reasons ordered from most to least severe, used to rank worst cases.
_SEVERITY = {"collision": 0, "stuck": 1, "nav2_aborted": 2, "timeout": 3, "setup_failed": 4}


@dataclass
class FamilySummary:
    family: str
    episodes: int
    successes: int
    success_rate: float
    failure_reasons: dict[str, int]
    collisions: int
    min_clearance_m: float | None
    median_time_to_goal_s: float | None
    worst: list[dict] = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)


def _finite(values: Iterable[float]) -> list[float]:
    return [v for v in values if isinstance(v, int | float) and math.isfinite(v)]


def _severity_key(row: Mapping[str, Any]) -> tuple:
    clearance = row["min_clearance_m"]
    return (
        row["success"],
        _SEVERITY.get(row["failure_reason"], 9),
        clearance if math.isfinite(clearance) else math.inf,
        row["scenario_id"],
    )


def summarize(rows: Iterable[Mapping[str, Any]], worst_n: int = 3) -> dict[str, FamilySummary]:
    by_family: dict[str, list[Mapping[str, Any]]] = {}
    for r in rows:
        by_family.setdefault(r["family"], []).append(r)
    out = {}
    for fam in sorted(by_family):
        rs = by_family[fam]
        ok = [r for r in rs if r["success"]]
        clearances = _finite(r["min_clearance_m"] for r in rs)
        times = _finite(r["time_to_goal_s"] for r in ok)
        worst = sorted((r for r in rs if not r["success"]), key=_severity_key)[:worst_n]
        out[fam] = FamilySummary(
            family=fam,
            episodes=len(rs),
            successes=len(ok),
            success_rate=len(ok) / len(rs),
            failure_reasons=dict(
                sorted(Counter(r["failure_reason"] for r in rs if not r["success"]).items())
            ),
            collisions=sum(int(r["collisions"]) for r in rs),
            min_clearance_m=min(clearances) if clearances else None,
            median_time_to_goal_s=statistics.median(times) if times else None,
            worst=[
                {
                    k: r[k]
                    for k in ("scenario_id", "config_hash", "failure_reason", "min_clearance_m")
                }
                for r in worst
            ],
        )
    return out


def weakest_family(summaries: Mapping[str, FamilySummary]) -> str | None:
    """Lowest success rate; ties go to more collisions per episode, then less clearance."""
    if not summaries:
        return None

    def key(s: FamilySummary) -> tuple:
        clearance = s.min_clearance_m if s.min_clearance_m is not None else math.inf
        return (s.success_rate, -s.collisions / s.episodes, clearance, s.family)

    return min(summaries.values(), key=key).family


def sweep_curve(
    rows: Iterable[Mapping[str, Any]],
    specs_by_hash: Mapping[str, Any],
    parameter: str,
) -> list[dict]:
    """Success rate per value of the swept parameter, sorted by value."""
    buckets: dict[float, list[bool]] = {}
    for r in rows:
        spec = specs_by_hash.get(r["config_hash"])
        if spec is None:
            continue
        try:
            v = float(get_path(spec, parameter))
        except (KeyError, TypeError, ValueError):
            continue
        buckets.setdefault(round(v, 6), []).append(bool(r["success"]))
    return [
        {
            "value": v,
            "episodes": len(b),
            "successes": sum(b),
            "success_rate": sum(b) / len(b),
        }
        for v, b in sorted(buckets.items())
    ]


def estimate_boundary(curve: list[dict], threshold: float = 0.5) -> dict:
    """Where the success rate crosses the threshold between adjacent swept values.

    Reports the bracketing pair rather than interpolating: with a handful of episodes
    per value, a midpoint would claim precision the data doesn't have.
    """
    for a, b in zip(curve, curve[1:], strict=False):
        if (a["success_rate"] >= threshold) != (b["success_rate"] >= threshold):
            return {
                "found": True,
                "between": [a["value"], b["value"]],
                "success_rates": [a["success_rate"], b["success_rate"]],
                "threshold": threshold,
            }
    return {"found": False, "between": None, "threshold": threshold}


def summary_table(summaries: Mapping[str, FamilySummary]) -> str:
    """Compact plain-text table for the LLM prompt."""
    lines = ["family | episodes | success_rate | failures | collisions | min_clearance_m"]
    for s in summaries.values():
        fails = ", ".join(f"{k}={v}" for k, v in s.failure_reasons.items()) or "-"
        clr = "n/a" if s.min_clearance_m is None else f"{s.min_clearance_m:.2f}"
        lines.append(
            f"{s.family} | {s.episodes} | {s.success_rate:.2f} | {fails} | {s.collisions} | {clr}"
        )
    return "\n".join(lines)
