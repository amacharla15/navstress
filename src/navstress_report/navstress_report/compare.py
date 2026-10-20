"""Before/after comparison: rows are paired by config hash, never by position."""

from __future__ import annotations

from dataclasses import dataclass, field

from navstress_eval.summary import summarize
from navstress_report.data import Run


@dataclass
class Comparison:
    before: Run
    after: Run
    matched_hashes: list[str]
    only_before: list[str]
    only_after: list[str]
    families: list[dict] = field(default_factory=list)
    regressions: list[dict] = field(default_factory=list)
    fixed: list[dict] = field(default_factory=list)


def _by_hash(run: Run) -> dict[str, dict]:
    out: dict[str, dict] = {}
    for r in run.rows:
        if r["config_hash"] in out:
            # Same scenario twice in one run would make the pairing ambiguous.
            raise ValueError(f"{run.run_id}: config hash {r['config_hash'][:12]} appears twice")
        out[r["config_hash"]] = r
    return out


def compare(before: Run, after: Run) -> Comparison:
    b, a = _by_hash(before), _by_hash(after)
    matched = sorted(set(b) & set(a))
    cmp = Comparison(
        before,
        after,
        matched,
        only_before=sorted(set(b) - set(a)),
        only_after=sorted(set(a) - set(b)),
    )
    sb = summarize(b[h] for h in matched)
    sa = summarize(a[h] for h in matched)
    for fam in sorted(set(sb) | set(sa)):
        x, y = sb.get(fam), sa.get(fam)
        cmp.families.append(
            {
                "family": fam,
                "episodes": x.episodes if x else 0,
                "before_success_rate": x.success_rate if x else None,
                "after_success_rate": y.success_rate if y else None,
                "before_collisions": x.collisions if x else 0,
                "after_collisions": y.collisions if y else 0,
            }
        )
    for h in matched:
        rb, ra = b[h], a[h]
        entry = {
            "scenario_id": ra["scenario_id"],
            "family": ra["family"],
            "config_hash": h,
            "before": rb["failure_reason"] or "success",
            "after": ra["failure_reason"] or "success",
        }
        if rb["success"] and not ra["success"]:
            cmp.regressions.append(entry)
        elif ra["success"] and not rb["success"]:
            cmp.fixed.append(entry)
    return cmp
