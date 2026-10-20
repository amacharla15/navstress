"""Report generation from sample results written to a temp dir.

The rows here are synthetic test fixtures produced by a fake batch function; they
exist only inside pytest's tmp_path and are labelled simulator=fakesim, so the
report under test must show the FAKESIM banner for them.
"""

import json
from pathlib import Path

import pytest

from navstress_agent.llm import MockLLM
from navstress_agent.loop import Budget, StressAgent
from navstress_eval.results_io import append_row, write_metadata, write_scenarios
from navstress_report.cli import build, main
from navstress_report.compare import compare
from navstress_report.data import load_run
from navstress_report.render import BEGIN, END
from navstress_scenarios.goals import GoalSet, NamedGoal
from navstress_scenarios.schema import config_hash, dump_jsonl

GOALS = GoalSet([NamedGoal(f"g{i:03d}", 2.0 + i, 1.0, 0.0) for i in range(4)])
PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def _outcome(spec, visible_height):
    low = any(o.height < visible_height for o in spec.obstacles)
    return ("collision", 0.0, 1) if low else ("", 0.5, 0)


def writing_batch(root: Path, visible_height: float = 0.25, label_prefix=""):
    def run_batch(specs, batch_id, label):
        run_dir = root / batch_id
        write_metadata(run_dir, {"run_id": batch_id, "label": label_prefix + label,
                                 "simulator": "fakesim", "simulator_version": "test",
                                 "git_commit": "0123456789abcdef", "ros_distro": "jazzy",
                                 "gpu": "none detected"})  # fmt: skip
        write_scenarios(run_dir, dump_jsonl(specs))
        rows = []
        for s in specs:
            reason, clearance, collisions = _outcome(s, visible_height)
            row = {
                "run_id": batch_id, "scenario_id": s.scenario_id, "family": s.family,
                "config_hash": config_hash(s), "seed": s.seed, "success": not reason,
                "failure_reason": reason, "nav2_error_code": 0, "nav2_error_msg": "",
                "time_to_goal_s": 30.0, "path_length_m": 6.0,
                "final_distance_to_goal_m": 2.0 if reason else 0.1,
                "min_clearance_m": clearance, "collisions": collisions,
                "recoveries_triggered": 1 if reason else 0,
                "bag_path": f"bags/{s.scenario_id.replace('/', '_')}", "wall_time_s": 31.0,
            }  # fmt: skip
            append_row(run_dir, row)
            rows.append(row)
        return rows

    return run_batch


@pytest.fixture
def results(tmp_path):
    root = tmp_path / "results"
    agent = StressAgent(
        MockLLM(), writing_batch(root), GOALS, (0.0, 0.0, 0.0), root / "agent1", "agent1",
        Budget(max_rounds=2, max_episodes_per_round=8, max_llm_calls=3),
    )  # fmt: skip
    agent.run()
    # "after the fix": same scenarios, sensors now see down to 0.05 m
    before = load_run(root / "agent1_r1")
    specs = [s for s in agent.specs_by_hash.values() if config_hash(s) in before.specs_by_hash]
    writing_batch(root, visible_height=0.05, label_prefix="fix ")(specs, "agent1_r1_fix", "rerun")
    return root


def test_full_report(results, tmp_path):
    md = tmp_path / "docs" / "RESULTS.md"
    site = tmp_path / "site"
    build(
        runs=[results / "agent1_r1"],
        agents=[results / "agent1"],
        before=results / "agent1_r1",
        after=results / "agent1_r1_fix",
        out_md=md,
        assets_dir=tmp_path / "docs" / "img",
        site_dir=site,
        title="Test report",
    )
    text = md.read_text()
    assert BEGIN in text and END in text
    assert "FAKESIM" in text
    # provenance next to the data
    assert "Source: `results/agent1_r1/results.csv`" in text and "git `0123456789`" in text
    # family table and failures with relative bag links
    assert "| low_obstacle | 2 | 0% (0/2) | collision 2 |" in text
    assert "](../results/agent1_r1/bags/low_obstacle_00)" in text
    # agent narrative: hypotheses, sweep table, bracketed boundary
    assert "narrowing on `low_obstacle`" in text
    assert "obstacles[0].height" in text and "crosses 50% between" in text
    # before/after by hash
    assert "8 scenarios matched by config hash; 0 unmatched" in text
    assert "Fixed (failed before, pass after): 2" in text
    assert "Regressions (passed before, fail after): 0" in text

    imgs = sorted((tmp_path / "docs" / "img").glob("*.png"))
    assert len(imgs) == 6  # run, sweep and before/after charts, each light + dark
    assert {p.name.endswith("-dark.png") for p in imgs} == {True, False}
    assert all(p.read_bytes().startswith(PNG_MAGIC) for p in imgs)

    page = (site / "index.html").read_text()
    assert "<table>" in page and "FAKESIM" in page and 'srcset="img/' in page
    for src in [s.split('"')[0] for s in page.split('src="img/')[1:]]:
        assert (site / "img" / src).exists()


def test_rerun_keeps_hand_written_sections(results, tmp_path):
    md = tmp_path / "RESULTS.md"
    md.write_text("# Results\n\n## Bring-up\n\nhand-written commands\n")
    kw = {
        "runs": [results / "agent1_r1"],
        "agents": [],
        "before": None,
        "after": None,
        "out_md": md,
        "assets_dir": tmp_path / "img",
        "site_dir": None,
        "title": "t",
    }
    build(**kw)
    build(**kw)
    text = md.read_text()
    assert "hand-written commands" in text
    assert text.count(BEGIN) == 1 and text.count(END) == 1


def test_compare_reports_regressions_and_unmatched(results):
    before = load_run(results / "agent1_r1_fix")
    after = load_run(results / "agent1_r1")
    after.rows = after.rows[:-1]
    cmp = compare(before, after)
    assert len(cmp.only_before) == 1 and not cmp.only_after
    assert {r["family"] for r in cmp.regressions} == {"low_obstacle"}


def test_compare_rejects_duplicate_hashes(results):
    run = load_run(results / "agent1_r1")
    run.rows.append(dict(run.rows[0]))
    with pytest.raises(ValueError, match="appears twice"):
        compare(run, run)


def test_cli_argument_checks(results, tmp_path, capsys):
    with pytest.raises(SystemExit):
        main(["--before", str(results / "agent1_r1")])
    with pytest.raises(SystemExit):
        main(["--out-md", str(tmp_path / "x.md")])
    assert main(["--run", str(results / "agent1_r1"), "--out-md", str(tmp_path / "x.md")]) == 0
    assert "wrote" in capsys.readouterr().out


def test_agent_log_is_complete(results):
    events = [json.loads(line)["event"] for line in (results / "agent1" / "agent_log.jsonl")
              .read_text().splitlines()]  # fmt: skip
    assert events.count("llm_call") == 2 and events[-1] == "stop"
