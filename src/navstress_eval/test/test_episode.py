import math

import pytest

from navstress_eval.bag_recorder import BagRecorder, qos_overrides
from navstress_eval.conversions import result_to_row, row_to_result
from navstress_eval.episode import PathIntegrator, StuckDetector, classify
from navstress_eval.results_io import RESULT_FIELDS


def test_path_integrator_ignores_jitter():
    p = PathIntegrator(min_step_m=0.01)
    p.add(0.0, 0.0)
    for _ in range(100):
        p.add(0.001, 0.0)  # sensor jitter around one spot
    p.add(3.0, 4.0)
    assert p.length_m == pytest.approx(5.0)


def test_stuck_needs_a_full_window_of_no_progress():
    s = StuckDetector(window_s=10.0, radius_m=0.25)
    for t in range(0, 10):
        s.add(float(t), 0.05 * (t % 2), 0.0)
    assert not s.stuck()  # only 9 s of data
    s.add(10.0, 0.0, 0.0)
    assert s.stuck()
    s.add(11.0, 1.0, 0.0)  # moved: the window now contains real progress
    assert not s.stuck()


def test_stuck_window_slides():
    s = StuckDetector(window_s=5.0, radius_m=0.25)
    for t in range(0, 20):
        x = t * 0.5 if t < 10 else 4.5  # drives, then stops at t=9
        s.add(float(t), x, 0.0)
    assert s.stuck()


@pytest.mark.parametrize(
    ("args", "expected"),
    [
        (("succeeded", 0), (True, "")),
        (("succeeded", 1), (False, "collision")),  # reaching the goal by hitting things fails
        (("aborted", 0), (False, "nav2_aborted")),
        (("rejected", 0), (False, "nav2_aborted")),
        (("canceled", 0, "stuck"), (False, "stuck")),
        (("canceled", 0, "timeout"), (False, "timeout")),
        (("", 0), (False, "timeout")),
        (("aborted", 2, "collision"), (False, "collision")),
    ],
)
def test_classify(args, expected):
    out = classify(*args)
    assert (out.success, out.failure_reason) == expected


def test_collision_can_be_counted_without_failing():
    assert classify("succeeded", 3, collision_fails=False).success


def test_result_row_round_trip():
    row = dict.fromkeys(RESULT_FIELDS, 0)
    row.update(run_id="r", scenario_id="s/1", family="f", config_hash="ab" * 32, success=True,
               failure_reason="", nav2_error_msg="", bag_path="bags/x", time_to_goal_s=1.5,
               path_length_m=2.0, final_distance_to_goal_m=0.1, min_clearance_m=math.inf,
               wall_time_s=3.0)  # fmt: skip
    assert result_to_row(row_to_result(row)) == row


def test_bag_command_uses_mcap_and_explicit_topics(tmp_path):
    rec = BagRecorder(["/tf", "", "/scan"], use_sim_time=True)
    cmd = rec.command(tmp_path / "b")
    assert cmd[:3] == ["ros2", "bag", "record"]
    assert cmd[cmd.index("-s") + 1] == "mcap"
    assert cmd[cmd.index("--topics") + 1 : cmd.index("--topics") + 3] == ["/tf", "/scan"]
    assert "--use-sim-time" in cmd
    assert "--use-sim-time" not in BagRecorder(["/tf"], use_sim_time=False).command(tmp_path)
    assert "--qos-profile-overrides-path" not in cmd
    with_qos = rec.command(tmp_path / "b", tmp_path / "q.yaml")
    assert with_qos[with_qos.index("--qos-profile-overrides-path") + 1] == str(tmp_path / "q.yaml")


def test_bag_qos_overrides_match_publishers():
    o = qos_overrides(["/scan", ""], ["/tf_static"])
    assert o["/scan"]["reliability"] == "best_effort" and o["/scan"]["durability"] == "volatile"
    assert o["/tf_static"] == {"history": "keep_last", "depth": 10, "reliability": "reliable",
                               "durability": "transient_local"}  # fmt: skip
    assert "" not in o
