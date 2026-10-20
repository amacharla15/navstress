"""Back-and-forth straight-line motion for dynamic obstacles."""

from __future__ import annotations

import math


def position_at(
    start: tuple[float, float], end: tuple[float, float], speed_mps: float, t: float
) -> tuple[float, float]:
    """Position after t seconds shuttling start -> end -> start at constant speed."""
    length = math.hypot(end[0] - start[0], end[1] - start[1])
    if length < 1e-9 or speed_mps <= 0.0 or t <= 0.0:
        return start
    s = (t * speed_mps) % (2.0 * length)
    d = s if s <= length else 2.0 * length - s
    f = d / length
    return (start[0] + (end[0] - start[0]) * f, start[1] + (end[1] - start[1]) * f)


def map_to_world(x: float, y: float, yaw: float, tf: tuple[float, float, float]):
    """Apply the map->world offset (tx, ty, theta). Identity when the frames coincide."""
    tx, ty, th = tf
    c, s = math.cos(th), math.sin(th)
    return (tx + c * x - s * y, ty + s * x + c * y, yaw + th)
