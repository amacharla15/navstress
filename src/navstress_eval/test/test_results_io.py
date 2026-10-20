import datetime as dt
import math

import pytest

from navstress_eval.results_io import (
    RESULT_FIELDS,
    RESULTS_CSV,
    append_row,
    collect_metadata,
    make_run_id,
    read_metadata,
    read_rows,
    rows_by_hash,
    update_metadata,
    validate_row,
    write_metadata,
)


def row(**over):
    r = {
        "run_id": "r1",
        "scenario_id": "low_obstacle/000",
        "family": "low_obstacle",
        "config_hash": "ab" * 32,
        "seed": 3,
        "success": False,
        "failure_reason": "collision",
        "nav2_error_code": 0,
        "nav2_error_msg": "",
        "time_to_goal_s": 12.5,
        "path_length_m": 4.25,
        "final_distance_to_goal_m": 1.0,
        "min_clearance_m": float("inf"),
        "collisions": 1,
        "recoveries_triggered": 2,
        "bag_path": "bags/x",
        "wall_time_s": 13.0,
    }
    r.update(over)
    return r


def test_round_trip_keeps_types_and_infinity(tmp_path):
    append_row(tmp_path, row())
    append_row(tmp_path, row(success=True, failure_reason="", min_clearance_m=0.31))
    back = read_rows(tmp_path)
    assert back[0] == row()
    assert back[1]["success"] is True and back[1]["min_clearance_m"] == 0.31
    assert math.isinf(back[0]["min_clearance_m"])
    header = (tmp_path / RESULTS_CSV).read_text().splitlines()[0]
    assert header == ",".join(RESULT_FIELDS)


@pytest.mark.parametrize(
    ("over", "msg"),
    [
        ({"success": True, "failure_reason": "timeout"}, "successful episode"),
        ({"failure_reason": "exploded"}, "not one of"),
        ({"failure_reason": ""}, "not one of"),
    ],
)
def test_inconsistent_rows_rejected(over, msg):
    with pytest.raises(ValueError, match=msg):
        validate_row(row(**over))


def test_missing_field_rejected():
    r = row()
    del r["config_hash"]
    with pytest.raises(ValueError, match="missing"):
        validate_row(r)


def test_unexpected_header_rejected(tmp_path):
    (tmp_path / RESULTS_CSV).write_text("a,b\n1,2\n")
    with pytest.raises(ValueError, match="unexpected columns"):
        read_rows(tmp_path)


def test_run_id_is_sortable_and_filesystem_safe():
    rid = make_run_id("base line/1", dt.datetime(2026, 10, 20, 12, 0, 5, tzinfo=dt.UTC))
    assert rid == "20261020T120005Z_base_line_1"


def test_metadata_round_trip(tmp_path):
    params = tmp_path / "nav2.yaml"
    params.write_text("a: 1\n")
    meta = collect_metadata("r1", "baseline", "fakesim", "0.1.0", str(params))
    for key in ("git_commit", "ros_distro", "gpu", "nav2_params_sha256", "simulator"):
        assert key in meta
    assert len(meta["nav2_params_sha256"]) == 64
    write_metadata(tmp_path, meta)
    update_metadata(tmp_path, finished_utc="x", episodes=2)
    back = read_metadata(tmp_path)
    assert back["finished_utc"] == "x" and back["simulator"] == "fakesim"


def test_rows_by_hash_groups():
    groups = rows_by_hash([row(), row(seed=4), row(config_hash="cd" * 32)])
    assert sorted(len(v) for v in groups.values()) == [1, 2]
