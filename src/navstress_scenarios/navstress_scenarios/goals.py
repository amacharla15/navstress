"""Reachable goals: a named goal set, and a sampler that builds one from a map.

SPEC requires every goal to be known-reachable. "Free" is not enough: a free cell
inside a closed-off area is free but unreachable. The sampler therefore inflates
obstacles by the robot radius and flood-fills from the start pose, and only samples
cells inside that connected region. NVIDIA's isaac_ros_navigation_goal samples free
cells but does not check connectivity, which is why this exists instead of reusing it.
"""

from __future__ import annotations

import argparse
import hashlib
import math
import os
import random
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class NamedGoal:
    id: str
    x: float
    y: float
    yaw: float


class GoalSet:
    """Named, known-reachable map poses. Satisfies schema.GoalLookup."""

    def __init__(self, goals: list[NamedGoal], meta: dict | None = None):
        self._goals = {g.id: g for g in goals}
        if len(self._goals) != len(goals):
            raise ValueError("goal ids must be unique")
        self.meta = meta or {}

    def __len__(self) -> int:
        return len(self._goals)

    def ids(self) -> list[str]:
        return sorted(self._goals)

    def get(self, goal_id: str) -> tuple[float, float, float] | None:
        g = self._goals.get(goal_id)
        return None if g is None else (g.x, g.y, g.yaw)

    def nearest(self, x: float, y: float) -> tuple[str, float] | None:
        best = None
        for g in self._goals.values():
            d = math.hypot(g.x - x, g.y - y)
            if best is None or d < best[1]:
                best = (g.id, d)
        return best

    def to_dict(self) -> dict:
        return {
            "meta": self.meta,
            "goals": [
                {"id": g.id, "x": g.x, "y": g.y, "yaw": g.yaw}
                for g in sorted(self._goals.values(), key=lambda g: g.id)
            ],
        }

    @classmethod
    def from_dict(cls, d: dict) -> GoalSet:
        goals = [
            NamedGoal(str(g["id"]), float(g["x"]), float(g["y"]), float(g.get("yaw", 0.0)))
            for g in d["goals"]
        ]
        return cls(goals, d.get("meta", {}))

    @classmethod
    def from_yaml(cls, path: str | os.PathLike) -> GoalSet:
        with open(path) as f:
            return cls.from_dict(yaml.safe_load(f))

    def to_yaml(self, path: str | os.PathLike) -> None:
        with open(path, "w") as f:
            yaml.safe_dump(self.to_dict(), f, sort_keys=False)


@dataclass
class OccupancyGrid:
    """A map_server-style occupancy grid. Row 0 is the top of the image (max y)."""

    free: np.ndarray  # bool [rows, cols]; True only for cells known to be free
    resolution: float
    origin_x: float
    origin_y: float
    source_sha256: str = ""

    @property
    def rows(self) -> int:
        return self.free.shape[0]

    @property
    def cols(self) -> int:
        return self.free.shape[1]

    def bounds(self) -> tuple[float, float, float, float]:
        return (
            self.origin_x,
            self.origin_x + self.cols * self.resolution,
            self.origin_y,
            self.origin_y + self.rows * self.resolution,
        )

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        col = math.floor((x - self.origin_x) / self.resolution)
        row_from_bottom = math.floor((y - self.origin_y) / self.resolution)
        row = self.rows - 1 - row_from_bottom
        if 0 <= row < self.rows and 0 <= col < self.cols:
            return row, col
        return None

    def cell_to_world(self, row: int, col: int) -> tuple[float, float]:
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (self.rows - 1 - row + 0.5) * self.resolution
        return x, y

    @classmethod
    def from_map_yaml(cls, path: str | os.PathLike) -> OccupancyGrid:
        """Load a map in the map_server YAML + image format (trinary mode).

        Pixel to occupancy follows map_server: p = (255 - v) / 255 (or v / 255 if
        negate), free if p < free_thresh. Unknown and occupied are both "not free".
        """
        from PIL import Image  # only needed when loading real maps

        path = Path(path)
        with open(path) as f:
            meta = yaml.safe_load(f)
        image_path = Path(meta["image"])
        if not image_path.is_absolute():
            image_path = path.parent / image_path
        raw = image_path.read_bytes()
        with Image.open(image_path) as img:
            v = np.asarray(img.convert("L"), dtype=np.float64)
        p = v / 255.0 if int(meta.get("negate", 0)) else (255.0 - v) / 255.0
        free = p < float(meta["free_thresh"])
        origin = meta["origin"]
        if len(origin) > 2 and abs(float(origin[2])) > 1e-9:
            raise ValueError("rotated map origins are not supported")
        return cls(
            free=free,
            resolution=float(meta["resolution"]),
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            source_sha256=hashlib.sha256(raw).hexdigest(),
        )


def _disk_offsets(radius_cells: int) -> list[tuple[int, int]]:
    r2 = radius_cells * radius_cells
    return [
        (dr, dc)
        for dr in range(-radius_cells, radius_cells + 1)
        for dc in range(-radius_cells, radius_cells + 1)
        if dr * dr + dc * dc <= r2
    ]


def inflate_blocked(free: np.ndarray, radius_cells: int) -> np.ndarray:
    """Cells within radius of any non-free cell (or the map edge) are blocked."""
    blocked = ~free
    if radius_cells <= 0:
        return blocked
    pad = radius_cells
    padded = np.pad(blocked, pad, constant_values=True)
    out = np.zeros_like(blocked)
    rows, cols = blocked.shape
    for dr, dc in _disk_offsets(radius_cells):
        out |= padded[pad + dr : pad + dr + rows, pad + dc : pad + dc + cols]
    return out


class ReachabilityMap:
    """Cells the robot centre can reach from the start without touching obstacles.

    Satisfies schema.FreeSpace.
    """

    def __init__(self, grid: OccupancyGrid, start_xy: tuple[float, float], robot_radius_m: float):
        self.grid = grid
        self.start_xy = start_xy
        self.robot_radius_m = robot_radius_m
        radius_cells = math.ceil(robot_radius_m / grid.resolution)
        blocked = inflate_blocked(grid.free, radius_cells)
        start = grid.world_to_cell(*start_xy)
        if start is None:
            raise ValueError(f"start {start_xy} is outside the map")
        if blocked[start]:
            raise ValueError(
                f"start {start_xy} is within {robot_radius_m} m of an obstacle or unknown space"
            )
        self.reachable = np.zeros_like(blocked)
        self.reachable[start] = True
        queue = deque([start])
        rows, cols = blocked.shape
        while queue:
            r, c = queue.popleft()
            for nr, nc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                inside = 0 <= nr < rows and 0 <= nc < cols
                if inside and not blocked[nr, nc] and not self.reachable[nr, nc]:
                    self.reachable[nr, nc] = True
                    queue.append((nr, nc))

    def is_reachable(self, x: float, y: float) -> bool:
        cell = self.grid.world_to_cell(x, y)
        return cell is not None and bool(self.reachable[cell])

    def sample(
        self, n: int, seed: int, min_separation_m: float = 0.0, min_start_distance_m: float = 0.0
    ) -> list[NamedGoal]:
        """Draw n goals from the reachable region, deterministic for a given seed."""
        rng = random.Random(seed)
        cells = np.argwhere(self.reachable)
        order = list(range(len(cells)))
        rng.shuffle(order)
        chosen: list[NamedGoal] = []
        for idx in order:
            if len(chosen) >= n:
                break
            x, y = self.grid.cell_to_world(*cells[idx])
            if math.hypot(x - self.start_xy[0], y - self.start_xy[1]) < min_start_distance_m:
                continue
            if any(math.hypot(x - g.x, y - g.y) < min_separation_m for g in chosen):
                continue
            yaw = rng.uniform(-math.pi, math.pi)
            chosen.append(
                NamedGoal(f"g{len(chosen):03d}", float(round(x, 3)), float(round(y, 3)), round(yaw, 4))
            )
        if len(chosen) < n:
            raise ValueError(f"only {len(chosen)} goals fit the constraints, asked for {n}")
        return chosen


def sample_goal_set(
    map_yaml: str | os.PathLike,
    start_xy: tuple[float, float],
    n: int,
    seed: int,
    robot_radius_m: float,
    min_separation_m: float,
    min_start_distance_m: float,
) -> GoalSet:
    grid = OccupancyGrid.from_map_yaml(map_yaml)
    reach = ReachabilityMap(grid, start_xy, robot_radius_m)
    goals = reach.sample(n, seed, min_separation_m, min_start_distance_m)
    meta = {
        "map_yaml": str(map_yaml),
        "map_image_sha256": grid.source_sha256,
        "start_xy": list(start_xy),
        "seed": seed,
        "robot_radius_m": robot_radius_m,
        "min_separation_m": min_separation_m,
        "min_start_distance_m": min_start_distance_m,
        "reachable_cells": int(reach.reachable.sum()),
    }
    return GoalSet(goals, meta)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="Sample known-reachable goals from a map_server map into a goal-set YAML."
    )
    ap.add_argument("--map", required=True, help="map_server YAML (e.g. carter_navigation)")
    ap.add_argument("--start", nargs=2, type=float, required=True, metavar=("X", "Y"))
    ap.add_argument("--n", type=int, default=60)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument(
        "--robot-radius",
        type=float,
        default=0.45,
        help="m of clearance the robot centre needs; Carter's Nav2 footprint is 0.75 x 0.50 m",
    )
    ap.add_argument("--min-separation", type=float, default=1.0)
    ap.add_argument("--min-start-distance", type=float, default=2.0)
    ap.add_argument("-o", "--output", required=True)
    args = ap.parse_args(argv)
    gs = sample_goal_set(
        args.map,
        tuple(args.start),
        args.n,
        args.seed,
        args.robot_radius,
        args.min_separation,
        args.min_start_distance,
    )
    gs.to_yaml(args.output)
    print(f"wrote {len(gs)} goals to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
