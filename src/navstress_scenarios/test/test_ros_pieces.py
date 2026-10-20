"""Conversions, noise, motion and USD asset generation (no running ROS graph)."""

import math

import numpy as np
import pytest

from navstress_scenarios.conversions import (
    msg_to_spec,
    quaternion_to_yaw,
    spec_to_msg,
    yaw_to_quaternion,
)
from navstress_scenarios.motion import map_to_world, position_at
from navstress_scenarios.noise import noisy_points, noisy_ranges
from navstress_scenarios.schema import ScenarioValidationError, config_hash, validate_or_raise
from navstress_scenarios.usd_assets import ensure_asset, obstacle_usda, read_metadata

RAW = {
    "scenario_id": "crossing/01",
    "family": "crossing",
    "goal": {"x": 4.0, "y": -1.5, "yaw": 1.2},
    "goal_id": "g003",
    "start": {"x": 0.5, "y": 0.25, "yaw": -0.3},
    "obstacles": [
        {"name": "a", "asset": "box", "pose": {"x": 1.0, "y": 2.0, "yaw": 0.5},
         "size_x": 0.4, "size_y": 0.7, "height": 0.12},
        {"name": "b", "asset": "cylinder", "pose": {"x": 2.0, "y": -2.0, "yaw": 0.0},
         "size_x": 0.5, "size_y": 0.5, "height": 1.0,
         "motion": {"end_x": 2.0, "end_y": 2.0, "speed_mps": 0.6}},
    ],
    "sensor_noise": {"gaussian_std_m": 0.03, "dropout_prob": 0.1},
    "nav_overrides": {"max_vel_x": 0.5, "local_inflation_radius": 0.6},
    "seed": 42,
    "episode_timeout_s": 75.0,
}  # fmt: skip


def test_spec_msg_round_trip_preserves_hash():
    spec = validate_or_raise(RAW)
    msg = spec_to_msg(spec)
    assert msg.config_hash == config_hash(spec)
    assert msg.has_start and msg.obstacles[1].moving and not msg.obstacles[0].moving
    back = msg_to_spec(msg)
    assert config_hash(back) == config_hash(spec)
    assert back.dict() == spec.dict()


def test_msg_without_start_round_trips():
    raw = dict(RAW)
    del raw["start"]
    spec = validate_or_raise(raw)
    msg = spec_to_msg(spec)
    assert not msg.has_start
    assert msg_to_spec(msg).start is None


def test_tampered_message_is_rejected():
    msg = spec_to_msg(validate_or_raise(RAW))
    msg.obstacles[0].height = 0.5  # content changed, hash not updated
    with pytest.raises(ScenarioValidationError, match="config_hash mismatch"):
        msg_to_spec(msg)


@pytest.mark.parametrize("yaw", [0.0, 0.7, -2.5, math.pi - 1e-6])
def test_yaw_quaternion_round_trip(yaw):
    assert quaternion_to_yaw(yaw_to_quaternion(yaw)) == pytest.approx(yaw, abs=1e-9)


def test_scan_noise_statistics_and_dropout():
    rng = np.random.default_rng(0)
    ranges = np.full(20000, 5.0)
    out = noisy_ranges(ranges, 0.1, 30.0, 0.05, 0.0, rng)
    assert out.std() == pytest.approx(0.05, rel=0.05)
    assert out.mean() == pytest.approx(5.0, abs=0.005)
    dropped = noisy_ranges(ranges, 0.1, 30.0, 0.0, 0.25, np.random.default_rng(1))
    assert np.isinf(dropped).mean() == pytest.approx(0.25, abs=0.01)


def test_scan_noise_leaves_invalid_returns_and_stays_in_range():
    ranges = np.array([np.inf, np.nan, 0.01, 29.99, 0.11])
    out = noisy_ranges(ranges, 0.1, 30.0, 1.0, 0.0, np.random.default_rng(3))
    assert np.isinf(out[0]) and np.isnan(out[1]) and out[2] == pytest.approx(0.01)
    assert 0.1 <= out[3] <= 30.0 and 0.1 <= out[4] <= 30.0


def test_scan_noise_is_reproducible_per_seed():
    r = np.linspace(1, 9, 100)
    a = noisy_ranges(r, 0.1, 30.0, 0.1, 0.2, np.random.default_rng(7))
    b = noisy_ranges(r, 0.1, 30.0, 0.1, 0.2, np.random.default_rng(7))
    np.testing.assert_array_equal(a, b)


def test_point_noise_moves_points_along_their_ray():
    xyz = np.array([[3.0, 4.0, 0.0], [0.0, 0.0, 2.0], [0.0, 0.0, 0.0]])
    out, keep = noisy_points(xyz, 0.2, 0.0, np.random.default_rng(5))
    assert keep.all()
    for p, q in zip(xyz[:2], out[:2], strict=True):
        cos = np.dot(p, q) / (np.linalg.norm(p) * np.linalg.norm(q))
        assert cos == pytest.approx(1.0)
    np.testing.assert_array_equal(out[2], xyz[2])  # origin point untouched
    _, keep = noisy_points(np.ones((10000, 3)), 0.0, 0.3, np.random.default_rng(6))
    assert (~keep).mean() == pytest.approx(0.3, abs=0.02)


def test_shuttle_motion():
    a, b = (0.0, 0.0), (4.0, 0.0)
    assert position_at(a, b, 1.0, 0.0) == a
    assert position_at(a, b, 1.0, 1.0) == pytest.approx((1.0, 0.0))
    assert position_at(a, b, 1.0, 4.0) == pytest.approx((4.0, 0.0))
    assert position_at(a, b, 1.0, 5.0) == pytest.approx((3.0, 0.0))  # on the way back
    assert position_at(a, b, 1.0, 8.0) == pytest.approx((0.0, 0.0))
    assert position_at(a, b, 0.0, 3.0) == a
    assert position_at(a, a, 1.0, 3.0) == a


def test_map_to_world():
    assert map_to_world(1.0, 2.0, 0.1, (0.0, 0.0, 0.0)) == (1.0, 2.0, 0.1)
    x, y, yaw = map_to_world(1.0, 0.0, 0.0, (10.0, 5.0, math.pi / 2))
    assert (x, y, yaw) == pytest.approx((10.0, 6.0, math.pi / 2))


def test_usd_asset_content():
    box = obstacle_usda("box", 1.2, 1.0, 0.15, kinematic=False)
    assert 'def Cube "geom"' in box and "xformOp:scale = (1.2, 1, 0.15)" in box
    assert "translate = (0, 0, 0.075)" in box  # base sits on the floor
    assert "PhysicsCollisionAPI" in box and "PhysicsRigidBodyAPI" not in box
    # An empty apiSchemas list is a USDA parse error (found with usd-core).
    assert "apiSchemas = []" not in box
    cyl = obstacle_usda("cylinder", 0.5, 9.9, 1.0, kinematic=True)
    assert 'def Cylinder "geom"' in cyl and "xformOp:scale = (0.5, 0.5, 1)" in cyl
    assert "PhysicsRigidBodyAPI" in cyl and "physics:kinematicEnabled = 1" in cyl
    with pytest.raises(ValueError):
        obstacle_usda("forklift", 1, 1, 1, False)


def test_usd_assets_are_cached_by_content(tmp_path):
    a = ensure_asset(tmp_path, "box", 0.6, 0.6, 0.08, False)
    b = ensure_asset(tmp_path, "box", 0.6, 0.6, 0.08, False)
    c = ensure_asset(tmp_path, "box", 0.6, 0.6, 0.09, False)
    assert a == b != c and a.is_absolute()
    assert read_metadata(a) == {
        "asset": "box", "size_x": 0.6, "size_y": 0.6, "height": 0.08, "kinematic": False,
    }  # fmt: skip
    (tmp_path / "other.usda").write_text("#usda 1.0\n")
    with pytest.raises(ValueError):
        read_metadata(tmp_path / "other.usda")
