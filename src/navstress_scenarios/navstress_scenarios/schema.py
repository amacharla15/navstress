"""ScenarioSpec schema: the only way LLM output can reach the simulator.

Numeric fields are clamped to safe ranges and every clamp is reported back.
Anything outside an allowlist (unknown keys, asset names, override names, too many
obstacles, non-finite numbers) is rejected with a message that says why.

The config hash is sha256 over a canonical JSON form that excludes the labels
(scenario_id, family), so two runs of the same physical setup share a hash even if
they were filed under different names.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import re
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from typing import Any, Protocol

try:
    # pydantic 2 (and 1.10.17+) expose the v1 API here. Ubuntu 24.04's python3-pydantic is
    # 1.10.14, which only has the top-level module; writing against the v1 API covers both.
    from pydantic import v1 as pyd
except ImportError:  # pragma: no cover - depends on the installed pydantic
    import pydantic as pyd

SCHEMA_VERSION = 1

# Fields that name a scenario but do not change what the simulator does.
LABEL_FIELDS = ("scenario_id", "family")

MAX_OBSTACLES = 8

# Simple shapes the scenario manager knows how to generate as USD. Pallet-sized boxes
# cover the "pallet in the aisle" family without depending on unverified asset paths.
ALLOWED_ASSETS = ("box", "cylinder")


@dataclass(frozen=True)
class Range:
    lo: float
    hi: float

    def clamp(self, value: float) -> float:
        return min(max(value, self.lo), self.hi)


# Ranges keep scenarios physically sensible for a Nova Carter in a warehouse and stop an
# LLM from asking for, say, a 40 m obstacle or 100% dropout.
OBSTACLE_RANGES = {
    "size_x": Range(0.05, 3.0),
    "size_y": Range(0.05, 3.0),
    "height": Range(0.02, 2.0),
}
MOTION_RANGES = {"speed_mps": Range(0.0, 1.5)}
NOISE_RANGES = {
    "gaussian_std_m": Range(0.0, 0.5),
    "dropout_prob": Range(0.0, 0.9),
}
SEED_RANGE = Range(0, 2**31 - 1)
TIMEOUT_RANGE = Range(10.0, 600.0)

# Nav2 parameters a scenario may override, with the range each is clamped to. Upper
# bounds stay at or below Carter's velocity_smoother limits (0.8 m/s, 0.7 rad/s) so an
# override can't ask for more than the smoother would pass through anyway. The mapping
# from these names to Nav2 node/parameter names lives in the scenario_manager config.
NAV_OVERRIDE_RANGES = {
    "max_vel_x": Range(0.1, 0.8),
    "max_vel_theta": Range(0.1, 0.7),
    "local_inflation_radius": Range(0.1, 1.5),
    "global_inflation_radius": Range(0.1, 2.0),
}

DEFAULT_BOUNDS = (-100.0, 100.0, -100.0, 100.0)

_NAME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,31}$")
_FAMILY_RE = re.compile(r"^[a-z][a-z0-9_]{0,39}$")
_SCENARIO_ID_RE = re.compile(r"^[A-Za-z0-9_][A-Za-z0-9_./-]{0,79}$")


class ScenarioValidationError(ValueError):
    def __init__(self, errors: list[str]):
        super().__init__("; ".join(errors))
        self.errors = errors


def _reject_non_numbers(where: str, v: Any) -> None:
    # pydantic v1 happily turns True into 1.0 and "0.3" into 0.3. A boolean or a string
    # in a numeric field means the producer misunderstood the schema, so say so.
    if isinstance(v, bool | str):
        raise ValueError(f"{where}: expected a number, got {type(v).__name__} {v!r}")


class _Frozen(pyd.BaseModel):
    class Config:
        extra = pyd.Extra.forbid
        # Specs are hashed after validation; freezing them means the hash can't go stale.
        allow_mutation = False

    @pyd.root_validator(pre=True)
    def _strict_numbers(cls, values: dict) -> dict:
        for name, f in cls.__fields__.items():
            if f.outer_type_ in (float, int) and name in values:
                _reject_non_numbers(name, values[name])
        return values


class PlanarPose(_Frozen):
    x: float
    y: float
    yaw: float = 0.0


class Motion(_Frozen):
    end_x: float
    end_y: float
    speed_mps: float


class Obstacle(_Frozen):
    name: str
    asset: str
    pose: PlanarPose
    size_x: float = 0.5
    size_y: float = 0.5
    height: float = 0.5
    motion: Motion | None = None

    @pyd.validator("name")
    def _name_ok(cls, v: str) -> str:
        if not _NAME_RE.match(v):
            raise ValueError(
                "must match [A-Za-z][A-Za-z0-9_]{0,31} (it becomes a sim entity name)"
            )
        return v

    @pyd.validator("asset")
    def _asset_allowed(cls, v: str) -> str:
        if v not in ALLOWED_ASSETS:
            raise ValueError(f"asset {v!r} is not in the allowlist {list(ALLOWED_ASSETS)}")
        return v


class SensorNoise(_Frozen):
    gaussian_std_m: float = 0.0
    dropout_prob: float = 0.0


class ScenarioSpec(_Frozen):
    schema_version: int = SCHEMA_VERSION
    scenario_id: str
    family: str
    goal: PlanarPose
    goal_id: str = ""
    start: PlanarPose | None = None
    obstacles: list[Obstacle] = []
    sensor_noise: SensorNoise = SensorNoise()
    nav_overrides: dict[str, float] = {}
    seed: int = 0
    episode_timeout_s: float = 120.0

    @pyd.validator("schema_version")
    def _version_ok(cls, v: int) -> int:
        if v != SCHEMA_VERSION:
            raise ValueError(f"schema_version {v} is not supported (expected {SCHEMA_VERSION})")
        return v

    @pyd.validator("scenario_id")
    def _id_ok(cls, v: str) -> str:
        if not _SCENARIO_ID_RE.match(v):
            raise ValueError("must be 1-80 chars of [A-Za-z0-9_./-] (it is used in file names)")
        return v

    @pyd.validator("family")
    def _family_ok(cls, v: str) -> str:
        if not _FAMILY_RE.match(v):
            raise ValueError("must be lower_snake_case, 1-40 chars")
        return v

    @pyd.validator("obstacles")
    def _obstacles_ok(cls, v: list[Obstacle]) -> list[Obstacle]:
        if len(v) > MAX_OBSTACLES:
            raise ValueError(f"at most {MAX_OBSTACLES} obstacles are allowed, got {len(v)}")
        names = [o.name for o in v]
        if len(set(names)) != len(names):
            raise ValueError("obstacle names must be unique within a scenario")
        return v

    @pyd.validator("nav_overrides", pre=True)
    def _override_values_numeric(cls, v: Any) -> Any:
        if isinstance(v, dict):
            for k, x in v.items():
                _reject_non_numbers(str(k), x)
        return v

    @pyd.validator("nav_overrides")
    def _overrides_allowed(cls, v: dict[str, float]) -> dict[str, float]:
        unknown = sorted(set(v) - set(NAV_OVERRIDE_RANGES))
        if unknown:
            raise ValueError(
                f"nav_overrides {unknown} are not in the allowlist {sorted(NAV_OVERRIDE_RANGES)}"
            )
        return v

    @property
    def config_hash(self) -> str:
        return config_hash(self)


class GoalLookup(Protocol):
    """What validation needs from a reachable goal set (see navstress_scenarios.goals)."""

    def get(self, goal_id: str) -> tuple[float, float, float] | None: ...

    def nearest(self, x: float, y: float) -> tuple[str, float] | None: ...


class FreeSpace(Protocol):
    def is_reachable(self, x: float, y: float) -> bool: ...


@dataclass
class ValidationContext:
    """Facts about the world that a scenario has to respect.

    With no goal set and no free-space map, goals are only clamped to the bounds; the
    agent always supplies a goal set so its goals are known to be reachable.
    """

    goal_set: GoalLookup | None = None
    free_space: FreeSpace | None = None
    bounds: tuple[float, float, float, float] = DEFAULT_BOUNDS  # xmin, xmax, ymin, ymax
    goal_tolerance_m: float = 0.05


@dataclass(frozen=True)
class ClampNote:
    path: str
    original: float
    clamped: float


@dataclass
class ValidationOutcome:
    spec: ScenarioSpec | None
    errors: list[str] = field(default_factory=list)
    clamped: list[ClampNote] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return self.spec is not None


def _is_number(v: Any) -> bool:
    # bool is an int subclass; True as a height would be a bug, not a number.
    return isinstance(v, int | float) and not isinstance(v, bool)


class _Clamper:
    def __init__(self) -> None:
        self.notes: list[ClampNote] = []
        self.errors: list[str] = []

    def apply(self, obj: Any, key: str, rng: Range, path: str, as_int: bool = False) -> None:
        if not isinstance(obj, dict) or key not in obj:
            return
        v = obj[key]
        if not _is_number(v):
            return  # pydantic reports the type error with the field path
        if not math.isfinite(v):
            self.errors.append(f"{path}: {v} is not a finite number")
            return
        c = rng.clamp(v)
        if as_int:
            c = int(c)
        if c != v:
            self.notes.append(ClampNote(path, float(v), float(c)))
            obj[key] = c

    def finite(self, obj: Any, key: str, path: str) -> None:
        if isinstance(obj, dict) and _is_number(obj.get(key)) and not math.isfinite(obj[key]):
            self.errors.append(f"{path}: {obj[key]} is not a finite number")


def _clamp_raw(raw: dict, bounds: tuple[float, float, float, float]) -> _Clamper:
    """Clamp known numeric fields in place, before pydantic sees them."""
    c = _Clamper()
    xr, yr = Range(bounds[0], bounds[1]), Range(bounds[2], bounds[3])

    def pose(p: Any, path: str) -> None:
        c.apply(p, "x", xr, f"{path}.x")
        c.apply(p, "y", yr, f"{path}.y")
        c.finite(p, "yaw", f"{path}.yaw")

    pose(raw.get("goal"), "goal")
    pose(raw.get("start"), "start")
    obstacles = raw.get("obstacles")
    if isinstance(obstacles, list):
        for i, ob in enumerate(obstacles):
            if not isinstance(ob, dict):
                continue
            p = f"obstacles[{i}]"
            pose(ob.get("pose"), f"{p}.pose")
            for k, r in OBSTACLE_RANGES.items():
                c.apply(ob, k, r, f"{p}.{k}")
            m = ob.get("motion")
            c.apply(m, "end_x", xr, f"{p}.motion.end_x")
            c.apply(m, "end_y", yr, f"{p}.motion.end_y")
            for k, r in MOTION_RANGES.items():
                c.apply(m, k, r, f"{p}.motion.{k}")
    noise = raw.get("sensor_noise")
    for k, r in NOISE_RANGES.items():
        c.apply(noise, k, r, f"sensor_noise.{k}")
    overrides = raw.get("nav_overrides")
    if isinstance(overrides, dict):
        for k in list(overrides):
            if k in NAV_OVERRIDE_RANGES:
                c.apply(overrides, k, NAV_OVERRIDE_RANGES[k], f"nav_overrides.{k}")
    c.apply(raw, "seed", SEED_RANGE, "seed", as_int=True)
    c.apply(raw, "episode_timeout_s", TIMEOUT_RANGE, "episode_timeout_s")
    return c


def normalize_yaw(yaw: float) -> float:
    y = math.remainder(yaw, 2.0 * math.pi)
    # remainder can return -pi; pick +pi so equal headings serialize identically.
    return math.pi if y <= -math.pi else y


def _pydantic_errors(exc: pyd.ValidationError) -> list[str]:
    out = []
    for e in exc.errors():
        loc = ".".join(str(p) if not isinstance(p, int) else f"[{p}]" for p in e["loc"])
        out.append(f"{loc.replace('.[', '[')}: {e['msg']}")
    return out


def _snap_to_known(raw: dict, goal_id: str, known: tuple[float, float, float]) -> None:
    # Position comes from the set so equal goals hash equally; an explicit yaw is kept
    # because the set's yaw is only a sampled default.
    goal = raw.get("goal")
    yaw = goal.get("yaw", known[2]) if isinstance(goal, dict) else known[2]
    raw["goal"] = {"x": known[0], "y": known[1], "yaw": yaw}
    raw["goal_id"] = goal_id


def _resolve_goal(raw: dict, ctx: ValidationContext, errors: list[str]) -> None:
    """Fill goal from goal_id or check a literal goal against the reachable set."""
    goal_id = raw.get("goal_id") or ""
    goal = raw.get("goal")
    if ctx.goal_set is not None and goal_id:
        known = ctx.goal_set.get(goal_id)
        if known is None:
            errors.append(f"goal_id: {goal_id!r} is not in the reachable goal set")
            return
        if isinstance(goal, dict) and _is_number(goal.get("x")) and _is_number(goal.get("y")):
            d = math.hypot(goal["x"] - known[0], goal["y"] - known[1])
            if d > ctx.goal_tolerance_m:
                errors.append(
                    f"goal: ({goal['x']}, {goal['y']}) is {d:.2f} m from goal_id {goal_id!r}"
                )
                return
        _snap_to_known(raw, goal_id, known)
        return
    if not isinstance(goal, dict):
        return  # pydantic reports the missing goal
    if not (_is_number(goal.get("x")) and _is_number(goal.get("y"))):
        return
    if ctx.goal_set is not None:
        hit = ctx.goal_set.nearest(goal["x"], goal["y"])
        if hit is not None and hit[1] <= ctx.goal_tolerance_m:
            known = ctx.goal_set.get(hit[0])
            assert known is not None
            _snap_to_known(raw, hit[0], known)
            return
    if ctx.free_space is not None:
        if ctx.free_space.is_reachable(goal["x"], goal["y"]):
            return
        errors.append(f"goal: ({goal['x']}, {goal['y']}) is not in reachable free space")
        return
    if ctx.goal_set is not None:
        errors.append("goal: not in the reachable goal set; use goal_id or a goal from the set")


def validate_scenario(raw: Any, ctx: ValidationContext | None = None) -> ValidationOutcome:
    """Clamp, validate and normalize one scenario. Never raises on bad input."""
    ctx = ctx or ValidationContext()
    if not isinstance(raw, Mapping):
        return ValidationOutcome(
            None, [f"scenario must be a JSON object, got {type(raw).__name__}"]
        )
    data = copy.deepcopy(dict(raw))
    if "config_hash" in data:
        # Accept specs read back from scenarios.jsonl; the hash is recomputed, never trusted.
        data.pop("config_hash")
    clamper = _clamp_raw(data, ctx.bounds)
    errors = list(clamper.errors)
    _resolve_goal(data, ctx, errors)
    if errors:
        return ValidationOutcome(None, errors, clamper.notes)
    try:
        spec = ScenarioSpec.parse_obj(data)
    except pyd.ValidationError as exc:
        return ValidationOutcome(None, _pydantic_errors(exc), clamper.notes)
    return ValidationOutcome(_normalize(spec), [], clamper.notes)


def validate_or_raise(raw: Any, ctx: ValidationContext | None = None) -> ScenarioSpec:
    out = validate_scenario(raw, ctx)
    if out.spec is None:
        raise ScenarioValidationError(out.errors)
    return out.spec


def _normalize(spec: ScenarioSpec) -> ScenarioSpec:
    d = spec.dict()
    for key in ("goal", "start"):
        if d.get(key) is not None:
            d[key]["yaw"] = normalize_yaw(d[key]["yaw"])
    for ob in d["obstacles"]:
        ob["pose"]["yaw"] = normalize_yaw(ob["pose"]["yaw"])
    return ScenarioSpec.parse_obj(d)


def _canonical_value(v: Any) -> Any:
    if isinstance(v, float):
        # Round so 0.1 + 0.2 and 0.3 hash the same; + 0.0 turns -0.0 into 0.0.
        return round(v, 6) + 0.0
    if isinstance(v, dict):
        return {k: _canonical_value(x) for k, x in v.items()}
    if isinstance(v, list):
        return [_canonical_value(x) for x in v]
    return v


def canonical_json(spec: ScenarioSpec) -> str:
    d = spec.dict()
    for k in LABEL_FIELDS:
        d.pop(k, None)
    return json.dumps(
        _canonical_value(d), sort_keys=True, separators=(",", ":"), ensure_ascii=True
    )


def config_hash(spec: ScenarioSpec) -> str:
    return hashlib.sha256(canonical_json(spec).encode("ascii")).hexdigest()


def spec_to_dict(spec: ScenarioSpec) -> dict:
    """Full form for scenarios.jsonl and the agent log: labels plus config_hash."""
    d = _canonical_value(spec.dict())
    d["config_hash"] = config_hash(spec)
    return d


def dump_jsonl(specs: Iterable[ScenarioSpec]) -> str:
    return "".join(json.dumps(spec_to_dict(s), sort_keys=True) + "\n" for s in specs)


def load_jsonl(text: str, ctx: ValidationContext | None = None) -> list[ScenarioSpec]:
    specs = []
    for n, line in enumerate(text.splitlines(), start=1):
        if not line.strip():
            continue
        try:
            raw = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ScenarioValidationError([f"line {n}: invalid JSON ({exc.msg})"]) from exc
        out = validate_scenario(raw, ctx)
        if out.spec is None:
            raise ScenarioValidationError([f"line {n}: {e}" for e in out.errors])
        specs.append(out.spec)
    return specs


_PATH_TOKEN = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)|\[(\d+)\]")


def get_path(spec: ScenarioSpec | Mapping, path: str) -> Any:
    """Read a value by a dotted path such as "obstacles[0].height".

    Sweeps name their parameter this way, so the agent and the report can find the
    swept value in any scenario without knowing the schema.
    """
    cur: Any = spec.dict() if isinstance(spec, ScenarioSpec) else spec
    for part in path.split("."):
        for m in _PATH_TOKEN.finditer(part):
            name, idx = m.group(1), m.group(2)
            if name is not None:
                if not isinstance(cur, Mapping) or name not in cur:
                    raise KeyError(f"{path}: no field {name!r}")
                cur = cur[name]
            else:
                i = int(idx)
                if not isinstance(cur, list) or i >= len(cur):
                    raise KeyError(f"{path}: index {i} out of range")
                cur = cur[i]
    return cur


def describe_schema() -> dict:
    """Machine-readable summary of fields, ranges and allowlists for the LLM prompt.

    Generated from the same tables the validator uses, so the prompt can't drift from
    what is actually enforced.
    """

    def rng(r: Range) -> list[float]:
        return [r.lo, r.hi]

    return {
        "schema_version": SCHEMA_VERSION,
        "required": ["scenario_id", "family", "goal_id"],
        "fields": {
            "scenario_id": "string, [A-Za-z0-9_./-], unique per scenario",
            "family": "lower_snake_case scenario family name",
            "goal_id": "id from goal_ids (goals are known-reachable map poses)",
            "start": "optional {x, y, yaw}; omit to start where the simulator resets",
            "obstacles": {
                "max_items": MAX_OBSTACLES,
                "item": {
                    "name": "unique [A-Za-z][A-Za-z0-9_]*",
                    "asset": list(ALLOWED_ASSETS),
                    "pose": "{x, y, yaw} in the map frame, centre of the obstacle",
                    **{k: rng(r) for k, r in OBSTACLE_RANGES.items()},
                    "motion": {
                        "optional": True,
                        "end_x": "map x of the far end of a straight back-and-forth path",
                        "end_y": "map y of the far end",
                        **{k: rng(r) for k, r in MOTION_RANGES.items()},
                    },
                },
            },
            "sensor_noise": {k: rng(r) for k, r in NOISE_RANGES.items()},
            "nav_overrides": {k: rng(r) for k, r in NAV_OVERRIDE_RANGES.items()},
            "seed": [int(SEED_RANGE.lo), int(SEED_RANGE.hi)],
            "episode_timeout_s": rng(TIMEOUT_RANGE),
        },
        "rules": [
            "Numbers outside a range are clamped to it.",
            "Unknown fields, assets or override names are rejected.",
        ],
    }
