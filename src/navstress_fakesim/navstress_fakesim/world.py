"""A deliberately simple 2D world for CI. It is a test double, not a robot simulator.

What it models: walls, box and cylinder obstacles, a square robot that cannot move
into anything, a 2D lidar mounted at a fixed height that only sees obstacles at least
that tall, and a grid planner that only avoids what the lidar can see. That is just
enough for the pipeline to have real failures to find (low obstacles block a robot
that never planned around them). Nothing here says anything about Nav2 or Isaac Sim.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Wall:
    x0: float
    y0: float
    x1: float
    y1: float


@dataclass
class Thing:
    """An obstacle. Boxes use size_x/size_y; cylinders use size_x as diameter."""

    name: str
    asset: str
    x: float
    y: float
    yaw: float
    size_x: float
    size_y: float
    height: float

    def _local(self, px: float, py: float) -> tuple[float, float]:
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        dx, dy = px - self.x, py - self.y
        return c * dx - s * dy, s * dx + c * dy

    def distance(self, px: float, py: float) -> float:
        """Distance from a point to the obstacle's outline (0 inside)."""
        if self.asset == "cylinder":
            return max(0.0, math.hypot(px - self.x, py - self.y) - self.size_x / 2.0)
        lx, ly = self._local(px, py)
        ex = max(abs(lx) - self.size_x / 2.0, 0.0)
        ey = max(abs(ly) - self.size_y / 2.0, 0.0)
        return math.hypot(ex, ey)

    def ray(self, ox: float, oy: float, dx: float, dy: float) -> float | None:
        if self.asset == "cylinder":
            return _ray_circle(ox - self.x, oy - self.y, dx, dy, self.size_x / 2.0)
        lox, loy = self._local(ox, oy)
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        ldx, ldy = c * dx - s * dy, s * dx + c * dy
        hx, hy = self.size_x / 2.0, self.size_y / 2.0
        return _ray_box(lox, loy, ldx, ldy, -hx, -hy, hx, hy)


def _ray_box(ox, oy, dx, dy, x0, y0, x1, y1) -> float | None:
    tmin, tmax = -math.inf, math.inf
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    if tmax < max(tmin, 0.0):
        return None
    return tmin if tmin >= 0.0 else None  # origin inside: no hit


def _ray_circle(ox, oy, dx, dy, r) -> float | None:
    b = ox * dx + oy * dy
    c = ox * ox + oy * oy - r * r
    disc = b * b - c
    if disc < 0.0 or c < 0.0:
        return None
    t = -b - math.sqrt(disc)
    return t if t >= 0.0 else None


@dataclass
class WorldConfig:
    bounds: tuple[float, float, float, float]  # xmin, xmax, ymin, ymax
    walls: list[Wall]
    start: tuple[float, float, float]
    robot_half_size: float = 0.3  # square footprint, same as the monitor's in fakesim
    lidar_height: float = 0.2
    lidar_range: float = 10.0
    lidar_rays: int = 360
    grid_resolution: float = 0.1
    plan_margin: float = 0.15

    @classmethod
    def from_dict(cls, d: dict) -> WorldConfig:
        return cls(
            bounds=tuple(d["bounds"]),
            walls=[Wall(*w) for w in d.get("walls", [])],
            start=tuple(d["start"]),
            **{k: d[k] for k in ("robot_half_size", "lidar_height", "lidar_range",
                                 "lidar_rays", "grid_resolution", "plan_margin") if k in d},
        )  # fmt: skip


@dataclass
class World:
    cfg: WorldConfig
    things: dict[str, Thing] = field(default_factory=dict)
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def __post_init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.things.clear()
        self.x, self.y, self.yaw = self.cfg.start

    # ------------------------------------------------------------ geometry

    def _robot_corners(self, x: float, y: float, yaw: float) -> list[tuple[float, float]]:
        h = self.cfg.robot_half_size
        c, s = math.cos(yaw), math.sin(yaw)
        return [(x + c * px - s * py, y + s * px + c * py) for px, py in
                ((h, h), (h, -h), (-h, -h), (-h, h))]  # fmt: skip

    def _wall_distance(self, w: Wall, px: float, py: float) -> float:
        ex = max(w.x0 - px, 0.0, px - w.x1)
        ey = max(w.y0 - py, 0.0, py - w.y1)
        return math.hypot(ex, ey)

    def blocked(self, x: float, y: float, yaw: float) -> bool:
        """True if the robot at this pose would overlap a wall or any obstacle."""
        h = self.cfg.robot_half_size
        xmin, xmax, ymin, ymax = self.cfg.bounds
        if not (xmin + h <= x <= xmax - h and ymin + h <= y <= ymax - h):
            return True
        # Sample the footprint outline and centre; dense enough for 0.1 m features.
        pts = [(x, y), *self._robot_corners(x, y, yaw)]
        corners = self._robot_corners(x, y, yaw)
        for i in range(4):
            (ax, ay), (bx, by) = corners[i], corners[(i + 1) % 4]
            pts += [(ax + (bx - ax) * k / 6, ay + (by - ay) * k / 6) for k in range(1, 6)]
        for px, py in pts:
            if any(self._wall_distance(w, px, py) == 0.0 for w in self.cfg.walls):
                return True
            if any(t.distance(px, py) == 0.0 for t in self.things.values()):
                return True
        # An obstacle small enough to sit fully inside the footprint still blocks it.
        c, s = math.cos(-yaw), math.sin(-yaw)
        for t in self.things.values():
            lx, ly = c * (t.x - x) - s * (t.y - y), s * (t.x - x) + c * (t.y - y)
            if abs(lx) <= h and abs(ly) <= h:
                return True
        return False

    def visible(self, t: Thing) -> bool:
        return t.height >= self.cfg.lidar_height

    def scan(self) -> list[float]:
        """Ranges for rays at angles -pi + i * 2pi/n in the robot frame (inf = no return)."""
        n = self.cfg.lidar_rays
        out = []
        for i in range(n):
            a = self.yaw - math.pi + i * 2.0 * math.pi / n
            dx, dy = math.cos(a), math.sin(a)
            best = math.inf
            for w in self.cfg.walls:
                t = _ray_box(self.x, self.y, dx, dy, w.x0, w.y0, w.x1, w.y1)
                if t is not None:
                    best = min(best, t)
            xmin, xmax, ymin, ymax = self.cfg.bounds
            for t in (
                (xmax - self.x) / dx if dx > 1e-12 else None,
                (xmin - self.x) / dx if dx < -1e-12 else None,
                (ymax - self.y) / dy if dy > 1e-12 else None,
                (ymin - self.y) / dy if dy < -1e-12 else None,
            ):
                if t is not None and t >= 0.0:
                    best = min(best, t)
            for thing in self.things.values():
                if self.visible(thing):
                    t = thing.ray(self.x, self.y, dx, dy)
                    if t is not None:
                        best = min(best, t)
            out.append(best if best <= self.cfg.lidar_range else math.inf)
        return out

    # ------------------------------------------------------------ planning

    def _grid(self):
        res = self.cfg.grid_resolution
        xmin, xmax, ymin, ymax = self.cfg.bounds
        cols, rows = int((xmax - xmin) / res), int((ymax - ymin) / res)
        # Inflate by the footprint's half-diagonal so the robot fits in any heading.
        clear = self.cfg.robot_half_size * math.sqrt(2.0) + self.cfg.plan_margin
        seen = [t for t in self.things.values() if self.visible(t)]

        def free(r: int, c: int) -> bool:
            px, py = xmin + (c + 0.5) * res, ymin + (r + 0.5) * res
            if not (xmin + clear <= px <= xmax - clear and ymin + clear <= py <= ymax - clear):
                return False
            if any(self._wall_distance(w, px, py) < clear for w in self.cfg.walls):
                return False
            return all(t.distance(px, py) >= clear for t in seen)

        return rows, cols, res, free

    def _segment_free(self, free, res, a, b) -> bool:
        (ar, ac), (br, bc) = a, b
        n = max(abs(br - ar), abs(bc - ac)) * 2 + 1
        return all(
            free(round(ar + (br - ar) * k / n), round(ac + (bc - ac) * k / n))
            for k in range(n + 1)
        )

    def plan(self, gx: float, gy: float) -> list[tuple[float, float]] | None:
        """A* on the grid around visible obstacles, then straightened by line of sight."""
        rows, cols, res, free_fn = self._grid()
        xmin, _, ymin, _ = self.cfg.bounds
        cache: dict[tuple[int, int], bool] = {}

        def free(r, c):
            if not (0 <= r < rows and 0 <= c < cols):
                return False
            k = (r, c)
            if k not in cache:
                cache[k] = free_fn(r, c)
            return cache[k]

        def cell(x, y):
            return int((y - ymin) / res), int((x - xmin) / res)

        start, goal = cell(self.x, self.y), cell(gx, gy)
        if not free(*goal):
            return None
        open_heap = [(0.0, start)]
        came: dict = {start: None}
        cost = {start: 0.0}
        steps = [(dr, dc, math.hypot(dr, dc)) for dr in (-1, 0, 1) for dc in (-1, 0, 1)
                 if dr or dc]  # fmt: skip
        while open_heap:
            _, cur = heapq.heappop(open_heap)
            if cur == goal:
                break
            for dr, dc, w in steps:
                nxt = (cur[0] + dr, cur[1] + dc)
                # The start cell may be inside the inflation; let the robot leave it.
                if not free(*nxt):
                    continue
                c = cost[cur] + w
                if c < cost.get(nxt, math.inf):
                    cost[nxt] = c
                    came[nxt] = cur
                    h = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                    heapq.heappush(open_heap, (c + h, nxt))
        if goal not in came:
            return None
        path = [goal]
        while came[path[-1]] is not None:
            path.append(came[path[-1]])
        path.reverse()
        # String-pull: keep only the cells needed to keep every segment collision-free.
        pulled = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self._segment_free(free, res, path[i], path[j]):
                j -= 1
            pulled.append(path[j])
            i = j
        pts = [(xmin + (c + 0.5) * res, ymin + (r + 0.5) * res) for r, c in pulled[1:]]
        pts[-1] = (gx, gy)
        return pts
