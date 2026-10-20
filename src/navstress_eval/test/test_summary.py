import math

import pytest

from navstress_eval.summary import estimate_boundary, summarize, sweep_curve, weakest_family


def r(family, success, reason="", clearance=0.5, collisions=0, sid="s", h="h", t=10.0):
    return {
        "family": family, "success": success, "failure_reason": reason,
        "min_clearance_m": clearance, "collisions": collisions, "scenario_id": sid,
        "config_hash": h, "time_to_goal_s": t,
    }  # fmt: skip


def test_summarize_counts_and_worst_cases():
    rows = [
        r("a", True, t=10.0),
        r("a", True, t=30.0),
        r("a", False, "timeout", clearance=0.4, sid="a3"),
        r("a", False, "collision", clearance=0.0, collisions=1, sid="a4"),
        r("b", False, "stuck", clearance=math.inf, sid="b1"),
    ]
    s = summarize(rows)
    a = s["a"]
    assert (a.episodes, a.successes, a.success_rate) == (4, 2, 0.5)
    assert a.failure_reasons == {"collision": 1, "timeout": 1}
    assert a.median_time_to_goal_s == 20.0 and a.min_clearance_m == 0.0
    assert [w["scenario_id"] for w in a.worst] == ["a4", "a3"]  # collision ranks first
    assert s["b"].min_clearance_m is None  # inf means no clearance data, not "very safe"


def test_weakest_family_tie_breaks():
    rows = [
        r("calm", False, "timeout"), r("calm", True),
        r("crashy", False, "collision", collisions=1, clearance=0.0), r("crashy", True),
        r("fine", True),
    ]  # fmt: skip
    assert weakest_family(summarize(rows)) == "crashy"
    assert weakest_family({}) is None


def test_sweep_curve_and_boundary():
    specs = {f"h{i}": {"obstacles": [{"height": v}]} for i, v in enumerate([0.1, 0.2, 0.3])}
    rows = [
        r("f", False, "collision", h="h0"),
        r("f", False, "collision", h="h0"),
        r("f", False, "collision", h="h1"),
        r("f", True, h="h1"),
        r("f", True, h="h2"),
        r("f", True, h="unknown-hash"),  # not in specs: ignored, not guessed
    ]
    curve = sweep_curve(rows, specs, "obstacles[0].height")
    assert [(p["value"], p["episodes"], p["success_rate"]) for p in curve] == [
        (0.1, 2, 0.0),
        (0.2, 2, 0.5),
        (0.3, 1, 1.0),
    ]
    b = estimate_boundary(curve)
    assert b["found"] and b["between"] == [0.1, 0.2]


@pytest.mark.parametrize("rates", [[1.0, 1.0], [0.0, 0.2, 0.4], []])
def test_no_crossing_reported_as_not_found(rates):
    curve = [{"value": i, "success_rate": x} for i, x in enumerate(rates)]
    assert estimate_boundary(curve) == {"found": False, "between": None, "threshold": 0.5}
