"""Results files: one CSV row per episode plus run metadata.

Everything the report shows is read back from these files, never from memory, so
every reported number traces to results/<run_id>/ and the git commit recorded there.
"""

from __future__ import annotations

import csv
import datetime as dt
import hashlib
import json
import math
import os
import platform
import shutil
import subprocess
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any

# Column order is the CSV header. Keep in sync with navstress_interfaces/EpisodeResult.
RESULT_FIELDS: tuple[str, ...] = (
    "run_id",
    "scenario_id",
    "family",
    "config_hash",
    "seed",
    "success",
    "failure_reason",
    "nav2_error_code",
    "nav2_error_msg",
    "time_to_goal_s",
    "path_length_m",
    "final_distance_to_goal_m",
    "min_clearance_m",
    "collisions",
    "recoveries_triggered",
    "bag_path",
    "wall_time_s",
)
_INT_FIELDS = {"seed", "nav2_error_code", "collisions", "recoveries_triggered"}
_FLOAT_FIELDS = {
    "time_to_goal_s",
    "path_length_m",
    "final_distance_to_goal_m",
    "min_clearance_m",
    "wall_time_s",
}
_BOOL_FIELDS = {"success"}

FAILURE_REASONS = ("timeout", "nav2_aborted", "collision", "stuck", "setup_failed")

RESULTS_CSV = "results.csv"
METADATA_JSON = "run_metadata.json"
SCENARIOS_JSONL = "scenarios.jsonl"


def results_root() -> Path:
    return Path(os.environ.get("NAVSTRESS_RESULTS_DIR", "results"))


def make_run_id(label: str, now: dt.datetime | None = None) -> str:
    now = now or dt.datetime.now(dt.UTC)
    safe = "".join(ch if ch.isalnum() or ch in "-_" else "_" for ch in label) or "run"
    return f"{now:%Y%m%dT%H%M%SZ}_{safe}"


def _format(field: str, v: Any) -> str:
    if field in _BOOL_FIELDS:
        return "true" if v else "false"
    if field in _FLOAT_FIELDS:
        f = float(v)
        return "inf" if math.isinf(f) and f > 0 else repr(f)
    return str(v)


def _parse(field: str, s: str) -> Any:
    if field in _BOOL_FIELDS:
        if s not in ("true", "false"):
            raise ValueError(f"{field}: expected true/false, got {s!r}")
        return s == "true"
    if field in _INT_FIELDS:
        return int(s)
    if field in _FLOAT_FIELDS:
        return float(s)
    return s


def validate_row(row: Mapping[str, Any]) -> dict:
    missing = [f for f in RESULT_FIELDS if f not in row]
    if missing:
        raise ValueError(f"result row is missing {missing}")
    out = {f: row[f] for f in RESULT_FIELDS}
    reason = out["failure_reason"]
    if out["success"] and reason:
        raise ValueError(f"successful episode has failure_reason {reason!r}")
    if not out["success"] and reason not in FAILURE_REASONS:
        raise ValueError(f"failure_reason {reason!r} is not one of {FAILURE_REASONS}")
    return out


def append_row(run_dir: Path, row: Mapping[str, Any]) -> None:
    """Append one episode. Written per episode so a crash loses at most the current one."""
    row = validate_row(row)
    run_dir.mkdir(parents=True, exist_ok=True)
    path = run_dir / RESULTS_CSV
    new = not path.exists()
    with open(path, "a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(RESULT_FIELDS)
        w.writerow([_format(k, row[k]) for k in RESULT_FIELDS])


def read_rows(run_dir: Path) -> list[dict]:
    with open(Path(run_dir) / RESULTS_CSV, newline="") as f:
        reader = csv.DictReader(f)
        if tuple(reader.fieldnames or ()) != RESULT_FIELDS:
            raise ValueError(f"{run_dir}/{RESULTS_CSV} has unexpected columns {reader.fieldnames}")
        return [{k: _parse(k, r[k]) for k in RESULT_FIELDS} for r in reader]


def write_scenarios(run_dir: Path, jsonl: str) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / SCENARIOS_JSONL).write_text(jsonl)


def read_scenario_dicts(run_dir: Path) -> list[dict]:
    path = Path(run_dir) / SCENARIOS_JSONL
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def _run(cmd: list[str], cwd: str | None = None) -> str | None:
    if shutil.which(cmd[0]) is None:
        return None
    try:
        out = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=10, check=True)
    except (subprocess.SubprocessError, OSError):
        return None
    return out.stdout.strip()


def git_info(repo_dir: str | None = None) -> dict:
    commit = _run(["git", "rev-parse", "HEAD"], repo_dir)
    dirty = _run(["git", "status", "--porcelain"], repo_dir)
    return {
        "git_commit": commit or "unknown",
        "git_dirty": bool(dirty) if dirty is not None else None,
    }


def gpu_info() -> str:
    q = _run(
        ["nvidia-smi", "--query-gpu=name,driver_version,memory.total", "--format=csv,noheader"]
    )
    return q or "none detected"


def file_sha256(path: str | os.PathLike | None) -> str:
    if not path:
        return ""
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return ""


def collect_metadata(
    run_id: str,
    label: str,
    simulator: str,
    simulator_version: str,
    nav2_params_file: str = "",
    repo_dir: str | None = None,
    extra: Mapping[str, Any] | None = None,
) -> dict:
    """What produced this run. The report prints it next to every table."""
    meta = {
        "run_id": run_id,
        "label": label,
        "created_utc": dt.datetime.now(dt.UTC).isoformat(timespec="seconds"),
        "simulator": simulator,
        "simulator_version": simulator_version,
        "ros_distro": os.environ.get("ROS_DISTRO", "unknown"),
        "gpu": gpu_info(),
        "hostname": platform.node(),
        "nav2_params_file": nav2_params_file,
        "nav2_params_sha256": file_sha256(nav2_params_file),
        **git_info(repo_dir),
    }
    if extra:
        meta.update(extra)
    return meta


def write_metadata(run_dir: Path, meta: Mapping[str, Any]) -> None:
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / METADATA_JSON).write_text(json.dumps(dict(meta), indent=2, sort_keys=True) + "\n")


def read_metadata(run_dir: Path) -> dict:
    path = Path(run_dir) / METADATA_JSON
    return json.loads(path.read_text()) if path.exists() else {}


def update_metadata(run_dir: Path, **fields: Any) -> None:
    meta = read_metadata(run_dir)
    meta.update(fields)
    write_metadata(run_dir, meta)


def rows_by_hash(rows: Iterable[Mapping[str, Any]]) -> dict[str, list[dict]]:
    out: dict[str, list[dict]] = {}
    for r in rows:
        out.setdefault(r["config_hash"], []).append(dict(r))
    return out
