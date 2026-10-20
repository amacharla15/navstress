"""The stress-test round loop: propose -> validate -> run -> summarize -> narrow.

The LLM only ever produces JSON. Code decides what runs: it validates every scenario,
enforces the episode and call budgets, and picks the family to narrow onto. A model
that ignores the focus instruction gets its off-focus scenarios dropped, not run.
"""

from __future__ import annotations

import datetime as dt
import json
from collections.abc import Callable, Mapping
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from navstress_agent.llm import LLMClient, Message, extract_json
from navstress_agent.prompts import retry_message, round_messages
from navstress_eval.summary import (
    FamilySummary,
    estimate_boundary,
    summarize,
    sweep_curve,
    weakest_family,
)
from navstress_scenarios.goals import GoalSet
from navstress_scenarios.schema import (
    ScenarioSpec,
    ValidationContext,
    config_hash,
    describe_schema,
    get_path,
    spec_to_dict,
    validate_scenario,
)

# (scenarios, batch_run_id, label) -> one result row per scenario
RunBatchFn = Callable[[list[ScenarioSpec], str, str], list[dict]]

AGENT_LOG = "agent_log.jsonl"
AGENT_SUMMARY = "agent_summary.json"


@dataclass
class Budget:
    max_rounds: int = 2
    max_episodes_per_round: int = 40
    max_llm_calls: int = 4


@dataclass
class Proposal:
    scenarios: list[ScenarioSpec]
    hypotheses: dict[str, str]
    sweep: dict | None
    log: dict


@dataclass
class RoundRecord:
    round: int
    focus_family: str | None
    batch_run_id: str
    hypotheses: dict[str, str]
    episodes: int
    summaries: dict[str, dict]
    weakest_family: str | None
    sweep: dict | None = None
    curve: list[dict] | None = None
    boundary: dict | None = None


@dataclass
class AgentResult:
    run_id: str
    rounds: list[RoundRecord] = field(default_factory=list)
    stop_reason: str = ""
    llm_calls: int = 0

    def to_dict(self) -> dict:
        return asdict(self)


class AgentLog:
    """Append-only JSONL: every prompt, raw reply, validation outcome and summary."""

    def __init__(self, path: Path, run_id: str):
        self.path = path
        self.run_id = run_id
        path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, event: str, round_no: int, **data: Any) -> None:
        rec = {
            "ts": dt.datetime.now(dt.UTC).isoformat(timespec="milliseconds"),
            "run_id": self.run_id,
            "round": round_no,
            "event": event,
            **data,
        }
        with open(self.path, "a") as f:
            f.write(json.dumps(rec, sort_keys=True, default=str) + "\n")


def _round_robin(by_family: Mapping[str, list[ScenarioSpec]], limit: int) -> list[ScenarioSpec]:
    """Truncate to the budget without starving any family."""
    queues = {f: list(v) for f, v in by_family.items()}
    out: list[ScenarioSpec] = []
    while len(out) < limit and any(queues.values()):
        for f in list(queues):
            if queues[f] and len(out) < limit:
                out.append(queues[f].pop(0))
    return out


class StressAgent:
    def __init__(
        self,
        llm: LLMClient,
        run_batch: RunBatchFn,
        goal_set: GoalSet,
        default_start: tuple[float, float, float],
        out_dir: Path,
        run_id: str,
        budget: Budget | None = None,
        ctx: ValidationContext | None = None,
    ):
        self.llm = llm
        self.run_batch = run_batch
        self.goal_set = goal_set
        self.default_start = default_start
        self.budget = budget or Budget()
        self.ctx = ctx or ValidationContext(goal_set=goal_set)
        self.run_id = run_id
        self.out_dir = Path(out_dir)
        self.log = AgentLog(self.out_dir / AGENT_LOG, run_id)
        self.result = AgentResult(run_id=run_id)
        self.specs_by_hash: dict[str, ScenarioSpec] = {}

    # ------------------------------------------------------------------ LLM side

    def _call(self, messages: list[Message], round_no: int) -> str | None:
        if self.result.llm_calls >= self.budget.max_llm_calls:
            return None
        self.result.llm_calls += 1
        raw = self.llm.complete(messages)
        self.log.write(
            "llm_call",
            round_no,
            call_index=self.result.llm_calls,
            prompt=messages,
            raw_response=raw,
        )
        return raw

    def _validate(self, reply: Any, focus: str | None, round_no: int) -> Proposal:
        if not isinstance(reply, dict) or not isinstance(reply.get("families"), list):
            raise ValueError('expected an object with a "families" list')
        accepted: dict[str, list[ScenarioSpec]] = {}
        hypotheses: dict[str, str] = {}
        rejected, clamped, off_focus, duplicates = [], [], [], []
        seen: set[str] = set(self.specs_by_hash)
        for fi, block in enumerate(reply["families"]):
            if not isinstance(block, dict):
                rejected.append({"family_index": fi, "errors": ["family block is not an object"]})
                continue
            fam = str(block.get("family", ""))
            hypotheses[fam] = str(block.get("hypothesis", ""))
            for si, raw in enumerate(block.get("scenarios") or []):
                if isinstance(raw, dict):
                    raw = {"family": fam, **raw}
                out = validate_scenario(raw, self.ctx)
                where = {"family_index": fi, "scenario_index": si}
                clamped += [dict(where, **asdict(n)) for n in out.clamped]
                if out.spec is None:
                    rejected.append(dict(where, errors=out.errors))
                    continue
                spec = out.spec
                if focus is not None and spec.family != focus:
                    off_focus.append(dict(where, family=spec.family))
                    continue
                h = config_hash(spec)
                if h in seen:
                    duplicates.append(dict(where, config_hash=h))
                    continue
                seen.add(h)
                accepted.setdefault(spec.family, []).append(spec)
        chosen = _round_robin(accepted, self.budget.max_episodes_per_round)
        n_accepted = sum(len(v) for v in accepted.values())
        sweep = self._check_sweep(reply.get("sweep"), chosen, focus)
        log = {
            "accepted": [spec_to_dict(s) for s in chosen],
            "rejected": rejected,
            "clamped": clamped,
            "dropped_off_focus": off_focus,
            "dropped_duplicates": duplicates,
            "dropped_over_budget": n_accepted - len(chosen),
            "sweep": sweep,
        }
        return Proposal(chosen, hypotheses, sweep, log)

    def _check_sweep(
        self, sweep: Any, chosen: list[ScenarioSpec], focus: str | None
    ) -> dict | None:
        if focus is None or not isinstance(sweep, dict) or not chosen:
            return None
        param = sweep.get("parameter")
        if not isinstance(param, str):
            return {"ignored": "sweep.parameter missing"}
        try:
            values = {round(float(get_path(s, param)), 6) for s in chosen}
        except (KeyError, TypeError, ValueError) as exc:
            return {"ignored": f"{param} is not a numeric field of every scenario ({exc})"}
        if len(values) < 2:
            return {"ignored": f"{param} takes only one value across the scenarios"}
        return {"family": focus, "parameter": param, "values": sorted(values)}

    def _propose(self, round_no: int, focus: str | None, history, examples) -> Proposal | None:
        messages = round_messages(
            round_no,
            self.budget.max_rounds,
            self.budget.max_episodes_per_round,
            describe_schema(),
            {gid: self.goal_set.get(gid) for gid in self.goal_set.ids()},
            self.default_start,
            history,
            focus,
            examples,
        )
        while True:
            raw = self._call(messages, round_no)
            if raw is None:
                return None
            try:
                proposal = self._validate(extract_json(raw), focus, round_no)
            except ValueError as exc:  # includes json.JSONDecodeError
                problem = f"not usable JSON ({exc})"
                self.log.write("validation", round_no, error=problem)
            else:
                self.log.write("validation", round_no, **proposal.log)
                if proposal.scenarios:
                    return proposal
                problem = "no scenario passed validation. Errors: " + json.dumps(
                    proposal.log["rejected"][:5]
                )
            messages = [*messages, {"role": "assistant", "content": raw}, retry_message(problem)]

    # ------------------------------------------------------------------ main loop

    def run(self, prior_rows: list[dict] | None = None) -> AgentResult:
        history: dict[str, FamilySummary] = summarize(prior_rows) if prior_rows else {}
        last_rows: list[dict] = list(prior_rows or [])
        for round_no in range(1, self.budget.max_rounds + 1):
            focus, examples = None, []
            if round_no > 1:
                focus = weakest_family(history)
                if focus is None or history[focus].success_rate == 1.0:
                    return self._stop("no failing family to narrow onto")
                examples = self._worst_examples(focus, last_rows)
            proposal = self._propose(round_no, focus, history, examples)
            if proposal is None:
                return self._stop("LLM call budget exhausted before a usable proposal")
            for s in proposal.scenarios:
                self.specs_by_hash[config_hash(s)] = s
            batch_id = f"{self.run_id}_r{round_no}"
            self.log.write(
                "batch_start", round_no, batch_run_id=batch_id, episodes=len(proposal.scenarios)
            )
            rows = self.run_batch(proposal.scenarios, batch_id, f"agent round {round_no}")
            summaries = summarize(rows)
            record = RoundRecord(
                round=round_no,
                focus_family=focus,
                batch_run_id=batch_id,
                hypotheses=proposal.hypotheses,
                episodes=len(rows),
                summaries={k: v.to_dict() for k, v in summaries.items()},
                weakest_family=weakest_family(summaries),
                sweep=proposal.sweep,
            )
            if proposal.sweep and "parameter" in proposal.sweep:
                record.curve = sweep_curve(rows, self.specs_by_hash, proposal.sweep["parameter"])
                record.boundary = estimate_boundary(record.curve)
            self.result.rounds.append(record)
            self.log.write("summary", round_no, **asdict(record))
            # The next round narrows within what this round found.
            history = summaries
            last_rows = rows
        return self._stop("max rounds reached")

    def _worst_examples(self, family: str, rows: list[dict], n: int = 2) -> list[dict]:
        failed = [r for r in rows if r["family"] == family and not r["success"]]
        failed.sort(key=lambda r: (r["min_clearance_m"], r["scenario_id"]))
        out = []
        for r in failed[:n]:
            spec = self.specs_by_hash.get(r["config_hash"])
            if spec is not None:
                out.append(spec_to_dict(spec))
        return out

    def _stop(self, reason: str) -> AgentResult:
        self.result.stop_reason = reason
        round_no = len(self.result.rounds)
        self.log.write("stop", round_no, reason=reason, llm_calls=self.result.llm_calls)
        (self.out_dir / AGENT_SUMMARY).write_text(
            json.dumps(self.result.to_dict(), indent=2, sort_keys=True, default=str) + "\n"
        )
        return self.result
