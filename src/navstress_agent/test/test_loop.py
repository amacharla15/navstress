"""Agent loop against the mock LLM and a deterministic fake batch runner.

The fake world below is a test double, not a model of Nav2: low obstacles cause
collisions, tall ones are avoided, heavy noise aborts half the seeds. It exists so
the loop's narrowing logic can be checked against a known answer.
"""

import json

import pytest

from navstress_agent.llm import MockLLM
from navstress_agent.loop import AGENT_LOG, AGENT_SUMMARY, Budget, StressAgent
from navstress_scenarios.goals import GoalSet, NamedGoal
from navstress_scenarios.schema import ScenarioSpec, config_hash

GOALS = GoalSet([NamedGoal(f"g{i:03d}", 2.0 + i, 1.0, 0.0) for i in range(6)])
START = (0.0, 0.0, 0.0)
VISIBLE_HEIGHT = 0.25


def fake_world(specs, batch_run_id, label, calls=None):
    if calls is not None:
        calls.append((list(specs), batch_run_id, label))
    rows = []
    for s in specs:
        assert isinstance(s, ScenarioSpec)  # only validated specs ever reach the runner
        low = any(o.height < VISIBLE_HEIGHT for o in s.obstacles)
        noisy = s.sensor_noise.gaussian_std_m > 0.05 and s.seed % 2 == 0
        reason = "collision" if low else ("nav2_aborted" if noisy else "")
        rows.append(
            {
                "run_id": batch_run_id,
                "scenario_id": s.scenario_id,
                "family": s.family,
                "config_hash": config_hash(s),
                "seed": s.seed,
                "success": not reason,
                "failure_reason": reason,
                "nav2_error_code": 0,
                "nav2_error_msg": "",
                "time_to_goal_s": 20.0,
                "path_length_m": 5.0,
                "final_distance_to_goal_m": 0.1 if not reason else 3.0,
                "min_clearance_m": 0.0 if low else 0.4,
                "collisions": 1 if low else 0,
                "recoveries_triggered": 0,
                "bag_path": "",
                "wall_time_s": 1.0,
            }
        )
    return rows


def make_agent(tmp_path, llm, budget=None, calls=None):
    return StressAgent(
        llm=llm,
        run_batch=lambda s, b, lbl: fake_world(s, b, lbl, calls),
        goal_set=GOALS,
        default_start=START,
        out_dir=tmp_path,
        run_id="agent_test",
        budget=budget or Budget(max_rounds=2, max_episodes_per_round=12, max_llm_calls=4),
    )


def read_log(tmp_path):
    return [json.loads(line) for line in (tmp_path / AGENT_LOG).read_text().splitlines()]


def test_two_rounds_narrow_onto_weakest_family_and_find_boundary(tmp_path):
    llm = MockLLM()
    calls = []
    result = make_agent(tmp_path, llm, calls=calls).run()

    assert result.stop_reason == "max rounds reached"
    assert len(result.rounds) == 2 and result.llm_calls == 2
    r1, r2 = result.rounds
    assert r1.focus_family is None
    assert set(r1.summaries) == {
        "crossing_obstacle",
        "lidar_degradation",
        "low_obstacle",
        "pallet_half_block",
    }
    assert r1.summaries["low_obstacle"]["success_rate"] == 0.0
    assert r1.weakest_family == "low_obstacle"

    # Round 2 runs only the weakest family, sweeping one parameter.
    assert r2.focus_family == "low_obstacle"
    round2_specs = calls[1][0]
    assert {s.family for s in round2_specs} == {"low_obstacle"}
    assert r2.sweep["parameter"] == "obstacles[0].height"
    heights = sorted(s.obstacles[0].height for s in round2_specs)
    assert len(set(heights)) == len(heights) >= 5
    # Everything except the swept height is held fixed.
    assert len({(s.goal_id, s.seed, s.obstacles[0].pose.json()) for s in round2_specs}) == 1

    lo, hi = r2.boundary["between"]
    assert r2.boundary["found"] and lo < VISIBLE_HEIGHT <= hi
    assert [p["success_rate"] for p in r2.curve] == [0.0, 0.0, 1.0, 1.0, 1.0, 1.0]

    # The second prompt tells the model which family to focus on.
    second_prompt = llm.calls[1][-1]["content"]
    assert "weakest family so far is 'low_obstacle'" in second_prompt


def test_log_records_prompts_replies_scenarios_and_summaries(tmp_path):
    make_agent(tmp_path, MockLLM()).run()
    events = [e["event"] for e in read_log(tmp_path)]
    assert events == [
        "llm_call", "validation", "batch_start", "summary",
        "llm_call", "validation", "batch_start", "summary",
        "stop",
    ]  # fmt: skip
    log = read_log(tmp_path)
    call = log[0]
    assert call["prompt"][0]["role"] == "system" and json.loads(call["raw_response"])
    accepted = log[1]["accepted"]
    assert accepted and all(len(s["config_hash"]) == 64 for s in accepted)
    assert log[3]["summaries"]["low_obstacle"]["episodes"] == 3
    saved = json.loads((tmp_path / AGENT_SUMMARY).read_text())
    assert saved["rounds"][1]["boundary"]["found"] is True


def test_episode_budget_truncates_round_robin(tmp_path):
    calls = []
    budget = Budget(max_rounds=1, max_episodes_per_round=6, max_llm_calls=2)
    agent = make_agent(tmp_path, MockLLM(), budget=budget, calls=calls)
    # The mock proposes max_scenarios // 4 = 1 per family; force a bigger proposal.
    reply = MockLLM()._explore(
        {
            "goals": {g: GOALS.get(g) for g in GOALS.ids()},
            "default_start": START,
            "max_scenarios": 20,
        }
    )
    agent.llm = MockLLM(scripted=[json.dumps(reply)])
    agent.run()
    specs = calls[0][0]
    assert len(specs) == 6
    families = [s.family for s in specs]
    assert all(families.count(f) >= 1 for f in set(families)) and len(set(families)) == 4
    validation = read_log(tmp_path)[1]
    assert validation["dropped_over_budget"] == 20 - 6


def test_invalid_reply_is_retried_then_budget_stops(tmp_path):
    llm = MockLLM(scripted=["not json at all", json.dumps({"families": []})])
    agent = make_agent(tmp_path, llm, budget=Budget(max_rounds=1, max_llm_calls=2))
    result = agent.run()
    assert result.stop_reason.startswith("LLM call budget exhausted")
    assert result.llm_calls == 2 and result.rounds == []
    # the retry carried the reason back to the model
    assert "not usable JSON" in llm.calls[1][-1]["content"]


def test_rejected_scenarios_are_logged_and_valid_ones_still_run(tmp_path):
    reply = {
        "families": [
            {
                "family": "mixed",
                "hypothesis": "h",
                "scenarios": [
                    {"scenario_id": "mixed/ok", "goal_id": "g000", "seed": 1},
                    {"scenario_id": "mixed/bad", "goal_id": "nowhere"},
                    {"scenario_id": "mixed/drive", "goal_id": "g001", "cmd_vel": 1.0},
                    {"scenario_id": "mixed/dup", "goal_id": "g000", "seed": 1},
                ],
            }
        ]
    }
    calls = []
    agent = make_agent(
        tmp_path, MockLLM(scripted=[json.dumps(reply)]), budget=Budget(max_rounds=1), calls=calls
    )
    agent.run()
    assert [s.scenario_id for s in calls[0][0]] == ["mixed/ok"]
    v = read_log(tmp_path)[1]
    assert len(v["rejected"]) == 2 and len(v["dropped_duplicates"]) == 1
    assert any("cmd_vel" in e for r in v["rejected"] for e in r["errors"])


def test_off_focus_scenarios_are_dropped_in_narrowing_rounds(tmp_path):
    first = MockLLM()._explore(
        {
            "goals": {g: GOALS.get(g) for g in GOALS.ids()},
            "default_start": START,
            "max_scenarios": 12,
        }
    )
    drifting = {
        "families": [
            {
                "family": "low_obstacle",
                "hypothesis": "h",
                "scenarios": [
                    dict(
                        first["families"][3]["scenarios"][0], scenario_id="low_obstacle/x", seed=77
                    )
                ],
            },
            {
                "family": "lidar_degradation",
                "hypothesis": "h",
                "scenarios": [
                    dict(first["families"][1]["scenarios"][0], scenario_id="lidar/x", seed=78)
                ],
            },
        ],
        "sweep": None,
    }
    calls = []
    agent = make_agent(
        tmp_path, MockLLM(scripted=[json.dumps(first), json.dumps(drifting)]), calls=calls
    )
    result = agent.run()
    assert result.rounds[1].focus_family == "low_obstacle"
    assert {s.family for s in calls[1][0]} == {"low_obstacle"}
    v = [e for e in read_log(tmp_path) if e["event"] == "validation"][1]
    assert v["dropped_off_focus"] == [
        {"family": "lidar_degradation", "family_index": 1, "scenario_index": 0}
    ]
    assert result.rounds[1].sweep is None  # no sweep declared, so no boundary claimed


def test_stops_when_nothing_failed(tmp_path):
    reply = {
        "families": [
            {
                "family": "easy",
                "hypothesis": "h",
                "scenarios": [{"scenario_id": "easy/0", "goal_id": "g000"}],
            }
        ]
    }
    agent = make_agent(tmp_path, MockLLM(scripted=[json.dumps(reply)]))
    result = agent.run()
    assert result.stop_reason == "no failing family to narrow onto"
    assert len(result.rounds) == 1


@pytest.mark.parametrize("bad", [[], {"families": "x"}, {"nope": 1}])
def test_malformed_structure_counts_as_unusable(tmp_path, bad):
    agent = make_agent(
        tmp_path, MockLLM(scripted=[json.dumps(bad)]), budget=Budget(max_rounds=1, max_llm_calls=1)
    )
    assert agent.run().stop_reason.startswith("LLM call budget exhausted")
