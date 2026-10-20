"""Load runs from results/<run_id>/ exactly as the eval runner wrote them."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from navstress_eval.results_io import read_metadata, read_rows, read_scenario_dicts


@dataclass
class Run:
    run_id: str
    dir: Path
    metadata: dict
    rows: list[dict]
    specs_by_hash: dict[str, dict] = field(default_factory=dict)

    @property
    def label(self) -> str:
        return str(self.metadata.get("label", ""))

    @property
    def is_fakesim(self) -> bool:
        return self.metadata.get("simulator") == "fakesim"


def load_run(run_dir: str | Path) -> Run:
    run_dir = Path(run_dir)
    rows = read_rows(run_dir)
    meta = read_metadata(run_dir)
    specs = {s["config_hash"]: s for s in read_scenario_dicts(run_dir)}
    return Run(meta.get("run_id", run_dir.name), run_dir, meta, rows, specs)


@dataclass
class AgentRun:
    run_id: str
    dir: Path
    summary: dict
    batches: dict[str, Run]


def load_agent(agent_dir: str | Path, results_root: str | Path | None = None) -> AgentRun:
    """The agent's own summary plus each round's batch run, found next to it."""
    agent_dir = Path(agent_dir)
    summary = json.loads((agent_dir / "agent_summary.json").read_text())
    root = Path(results_root) if results_root else agent_dir.parent
    batches = {}
    for rnd in summary.get("rounds", []):
        bid = rnd["batch_run_id"]
        if (root / bid / "results.csv").exists():
            batches[bid] = load_run(root / bid)
    return AgentRun(summary.get("run_id", agent_dir.name), agent_dir, summary, batches)
