import json
import math

import pytest

from navstress_scenarios.goals import GoalSet, NamedGoal
from navstress_scenarios.schema import (
    ALLOWED_ASSETS,
    MAX_OBSTACLES,
    NAV_OVERRIDE_RANGES,
    ScenarioValidationError,
    ValidationContext,
    canonical_json,
    config_hash,
    describe_schema,
    dump_jsonl,
    get_path,
    load_jsonl,
    validate_or_raise,
    validate_scenario,
)

GOALS = GoalSet(
    [
        NamedGoal("g000", 1.0, 2.0, 0.0),
        NamedGoal("g001", -3.0, 4.5, 1.57),
    ]
)
CTX = ValidationContext(goal_set=GOALS, bounds=(-10.0, 10.0, -10.0, 10.0))


def base(**over):
    raw = {
        "scenario_id": "low_obstacle/001",
        "family": "low_obstacle",
        "goal_id": "g000",
        "obstacles": [
            {
                "name": "box1",
                "asset": "box",
                "pose": {"x": 0.5, "y": 1.0, "yaw": 0.0},
                "size_x": 0.6,
                "size_y": 0.4,
                "height": 0.1,
            }
        ],
        "sensor_noise": {"gaussian_std_m": 0.01, "dropout_prob": 0.05},
        "seed": 7,
        "episode_timeout_s": 90,
    }
    raw.update(over)
    return raw


def errors_of(raw, ctx=CTX):
    out = validate_scenario(raw, ctx)
    assert out.spec is None, "expected rejection"
    return " | ".join(out.errors)


# --- validation and clamping ---------------------------------------------------------


def test_valid_spec_resolves_goal_from_set():
    out = validate_scenario(base(), CTX)
    assert out.ok, out.errors
    assert out.clamped == []
    assert (out.spec.goal.x, out.spec.goal.y) == (1.0, 2.0)
    assert out.spec.goal_id == "g000"


@pytest.mark.parametrize(
    ("path", "value", "expected"),
    [
        ("obstacles[0].height", 5.0, 2.0),
        ("obstacles[0].height", 0.0, 0.02),
        ("obstacles[0].size_x", 40.0, 3.0),
        ("sensor_noise.dropout_prob", 1.5, 0.9),
        ("sensor_noise.gaussian_std_m", -0.2, 0.0),
        ("episode_timeout_s", 5, 10.0),
        ("episode_timeout_s", 10_000, 600.0),
        ("seed", -3, 0),
        ("obstacles[0].pose.x", 55.0, 10.0),
    ],
)
def test_numeric_fields_are_clamped_and_reported(path, value, expected):
    raw = base()
    _set_path(raw, path, value)
    out = validate_scenario(raw, CTX)
    assert out.ok, out.errors
    assert get_path(out.spec, path) == pytest.approx(expected)
    assert [(n.path, n.original, n.clamped) for n in out.clamped] == [
        (path, float(value), float(expected))
    ]


def test_motion_speed_and_endpoint_clamped():
    raw = base()
    raw["obstacles"][0]["motion"] = {"end_x": 99.0, "end_y": 0.0, "speed_mps": 10.0}
    out = validate_scenario(raw, CTX)
    assert out.ok, out.errors
    m = out.spec.obstacles[0].motion
    assert (m.end_x, m.speed_mps) == (10.0, 1.5)
    assert {n.path for n in out.clamped} == {
        "obstacles[0].motion.end_x",
        "obstacles[0].motion.speed_mps",
    }


def test_nav_override_value_clamped():
    out = validate_scenario(base(nav_overrides={"max_vel_x": 3.0}), CTX)
    assert out.ok, out.errors
    assert out.spec.nav_overrides == {"max_vel_x": NAV_OVERRIDE_RANGES["max_vel_x"].hi}


# --- allowlist rejection -------------------------------------------------------------


def test_unknown_asset_rejected_with_allowlist_in_message():
    raw = base()
    raw["obstacles"][0]["asset"] = "forklift"
    msg = errors_of(raw)
    assert "forklift" in msg and "allowlist" in msg
    for a in ALLOWED_ASSETS:
        assert a in msg


def test_unknown_nav_override_rejected():
    msg = errors_of(base(nav_overrides={"use_sim_time": 0.0}))
    assert "use_sim_time" in msg and "allowlist" in msg


def test_unknown_top_level_field_rejected():
    # The schema has no way to command the robot; extra keys can't sneak one in.
    msg = errors_of(base(cmd_vel={"linear": 1.0}))
    assert "cmd_vel" in msg


def test_unknown_obstacle_field_rejected():
    raw = base()
    raw["obstacles"][0]["mass_kg"] = 5
    assert "mass_kg" in errors_of(raw)


def test_too_many_obstacles_rejected():
    raw = base()
    ob = raw["obstacles"][0]
    raw["obstacles"] = [dict(ob, name=f"b{i}") for i in range(MAX_OBSTACLES + 1)]
    assert f"at most {MAX_OBSTACLES}" in errors_of(raw)


def test_duplicate_obstacle_names_rejected():
    raw = base()
    raw["obstacles"].append(dict(raw["obstacles"][0]))
    assert "unique" in errors_of(raw)


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
def test_non_finite_numbers_rejected(value):
    raw = base()
    raw["obstacles"][0]["height"] = value
    assert "not a finite number" in errors_of(raw)


@pytest.mark.parametrize("value", [True, "0.3"])
def test_bool_or_string_in_numeric_field_rejected(value):
    raw = base()
    raw["obstacles"][0]["height"] = value
    assert "expected a number" in errors_of(raw)


def test_bool_override_value_rejected():
    assert "expected a number" in errors_of(base(nav_overrides={"max_vel_x": True}))


@pytest.mark.parametrize(
    ("field", "value"),
    [("family", "Low Obstacle"), ("scenario_id", "../../etc/passwd x"), ("schema_version", 2)],
)
def test_bad_labels_and_version_rejected(field, value):
    assert field in errors_of(base(**{field: value}))


def test_non_object_rejected():
    assert "JSON object" in errors_of(["not", "a", "dict"])


# --- goals -----------------------------------------------------------------------------


def test_unknown_goal_id_rejected():
    assert "not in the reachable goal set" in errors_of(base(goal_id="g999"))


def test_literal_goal_matching_set_gets_goal_id():
    raw = base(goal_id="", goal={"x": -3.0, "y": 4.5, "yaw": 0.0})
    spec = validate_or_raise(raw, CTX)
    assert spec.goal_id == "g001"


def test_literal_goal_off_set_rejected():
    msg = errors_of(base(goal_id="", goal={"x": 0.0, "y": 0.0, "yaw": 0.0}))
    assert "goal" in msg


def test_goal_id_conflicting_with_literal_pose_rejected():
    msg = errors_of(base(goal={"x": 5.0, "y": 5.0, "yaw": 0.0}))
    assert "from goal_id" in msg


class _HalfPlane:
    def is_reachable(self, x, y):
        return x > 0


def test_free_space_check_for_sampled_goals():
    ctx = ValidationContext(free_space=_HalfPlane())
    ok = validate_scenario(base(goal_id="", goal={"x": 2.0, "y": 0.0}), ctx)
    assert ok.ok, ok.errors
    bad = validate_scenario(base(goal_id="", goal={"x": -2.0, "y": 0.0}), ctx)
    assert "reachable free space" in " ".join(bad.errors)


def test_yaw_is_normalized():
    spec = validate_or_raise(
        base(goal_id="", goal={"x": 1.0, "y": 2.0, "yaw": 4 * math.pi + 0.1}), CTX
    )
    assert spec.goal.yaw == pytest.approx(0.1)
    # without an explicit yaw the goal set's yaw is used
    assert validate_or_raise(base(goal_id="g001"), CTX).goal.yaw == pytest.approx(1.57)
    raw = base()
    raw["obstacles"][0]["pose"]["yaw"] = -math.pi
    spec = validate_or_raise(raw, CTX)
    assert spec.obstacles[0].pose.yaw == pytest.approx(math.pi)


# --- hashing ---------------------------------------------------------------------------

# Pinned so an accidental change to canonicalization (and therefore to every stored
# hash) fails loudly instead of silently breaking before/after matching.
GOLDEN_HASH = "e8ea3cea62090ac5f4cced09a91ad76389a734254b9259048f2a577dfed633ea"


def test_hash_is_pinned():
    assert config_hash(validate_or_raise(base(), CTX)) == GOLDEN_HASH


def test_hash_ignores_key_order_and_float_noise():
    a = validate_or_raise(base(), CTX)
    raw = json.loads(json.dumps(base()), object_pairs_hook=lambda kv: dict(reversed(kv)))
    raw["sensor_noise"]["gaussian_std_m"] = 0.1 * 0.1  # 0.010000000000000002
    b = validate_or_raise(raw, CTX)
    assert canonical_json(a) == canonical_json(b)
    assert config_hash(a) == config_hash(b)


def test_hash_ignores_labels_only():
    a = validate_or_raise(base(), CTX)
    b = validate_or_raise(base(scenario_id="other/9", family="renamed"), CTX)
    assert config_hash(a) == config_hash(b)


@pytest.mark.parametrize(
    ("path", "value"),
    [
        ("obstacles[0].height", 0.11),
        ("seed", 8),
        ("sensor_noise.dropout_prob", 0.06),
        ("goal_id", "g001"),
    ],
)
def test_hash_changes_with_any_physical_change(path, value):
    raw = base()
    _set_path(raw, path, value)
    assert config_hash(validate_or_raise(raw, CTX)) != config_hash(validate_or_raise(base(), CTX))


def test_jsonl_round_trip_preserves_hash_and_rejects_tampering():
    specs = [validate_or_raise(base(), CTX), validate_or_raise(base(seed=9), CTX)]
    text = dump_jsonl(specs)
    back = load_jsonl(text, CTX)
    assert [config_hash(s) for s in back] == [config_hash(s) for s in specs]
    # A stored config_hash is recomputed, never trusted.
    tampered = json.loads(text.splitlines()[0])
    tampered["config_hash"] = "0" * 64
    assert config_hash(load_jsonl(json.dumps(tampered), CTX)[0]) == config_hash(specs[0])
    with pytest.raises(ScenarioValidationError, match="line 1"):
        load_jsonl('{"family": 3}\n', CTX)


def test_get_path():
    spec = validate_or_raise(base(), CTX)
    assert get_path(spec, "obstacles[0].height") == pytest.approx(0.1)
    assert get_path(spec, "sensor_noise.dropout_prob") == pytest.approx(0.05)
    with pytest.raises(KeyError):
        get_path(spec, "obstacles[3].height")


def test_describe_schema_matches_enforced_tables():
    d = describe_schema()
    assert d["fields"]["obstacles"]["item"]["asset"] == list(ALLOWED_ASSETS)
    assert set(d["fields"]["nav_overrides"]) == set(NAV_OVERRIDE_RANGES)
    json.dumps(d)  # must be serializable for the prompt


def _set_path(raw, path, value):
    parts = path.replace("]", "").replace("[", ".").split(".")
    cur = raw
    for p in parts[:-1]:
        cur = cur[int(p)] if p.isdigit() else cur[p]
    cur[parts[-1]] = value
