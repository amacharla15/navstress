"""Episode bookkeeping that doesn't need ROS: path length, stuck detection, outcome."""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass


class PathIntegrator:
    """Sums distance between odometry samples, ignoring sub-centimetre jitter."""

    def __init__(self, min_step_m: float = 0.01):
        self.min_step_m = min_step_m
        self.length_m = 0.0
        self._last: tuple[float, float] | None = None

    def add(self, x: float, y: float) -> None:
        if self._last is None:
            self._last = (x, y)
            return
        d = math.hypot(x - self._last[0], y - self._last[1])
        if d >= self.min_step_m:
            self.length_m += d
            self._last = (x, y)


class StuckDetector:
    """Stuck = stayed within radius_m for the whole of the last window_s seconds.

    The window is longer than Nav2's own progress checker so Nav2's recoveries get
    their chance first; this catches the case where they loop without giving up.
    """

    def __init__(self, window_s: float, radius_m: float):
        self.window_s = window_s
        self.radius_m = radius_m
        self._samples: deque[tuple[float, float, float]] = deque()

    def add(self, t: float, x: float, y: float) -> None:
        self._samples.append((t, x, y))
        while len(self._samples) > 2 and self._samples[1][0] <= t - self.window_s:
            self._samples.popleft()

    def stuck(self) -> bool:
        if len(self._samples) < 2:
            return False
        t0, x0, y0 = self._samples[0]
        if self._samples[-1][0] - t0 < self.window_s:
            return False
        return all(math.hypot(x - x0, y - y0) <= self.radius_m for _, x, y in self._samples)


@dataclass
class Outcome:
    success: bool
    failure_reason: str


def classify(
    nav_status: str,
    collisions: int,
    stopped_for: str = "",
    collision_fails: bool = True,
) -> Outcome:
    """Decide the episode outcome.

    nav_status: "succeeded" | "aborted" | "canceled" | "rejected" | "" (no result).
    stopped_for: why the runner cancelled the goal itself ("collision", "stuck",
    "timeout"), if it did. Priority: a collision makes the episode a failure even if
    Nav2 later reports success, because reaching the goal by hitting something is not
    a pass.
    """
    if collision_fails and collisions > 0:
        return Outcome(False, "collision")
    if stopped_for:
        return Outcome(False, stopped_for)
    if nav_status == "succeeded":
        return Outcome(True, "")
    if nav_status in ("aborted", "rejected", "canceled"):
        return Outcome(False, "nav2_aborted")
    return Outcome(False, "timeout")
