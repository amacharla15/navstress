"""Range noise and dropout, as pure functions so they can be tested without ROS."""

from __future__ import annotations

import numpy as np


def noisy_ranges(
    ranges: np.ndarray,
    range_min: float,
    range_max: float,
    gaussian_std_m: float,
    dropout_prob: float,
    rng: np.random.Generator,
) -> np.ndarray:
    """LaserScan ranges with Gaussian noise on valid returns and random dropouts.

    Invalid returns (NaN, inf, out of [range_min, range_max]) are left alone. A dropped
    return becomes +inf, which REP 117 defines as "no return".
    """
    out = np.asarray(ranges, dtype=np.float32).copy()
    valid = np.isfinite(out) & (out >= range_min) & (out <= range_max)
    if gaussian_std_m > 0.0 and valid.any():
        out[valid] += rng.normal(0.0, gaussian_std_m, int(valid.sum())).astype(np.float32)
        out[valid] = np.clip(out[valid], range_min, range_max)
    if dropout_prob > 0.0:
        out[rng.random(out.shape[0]) < dropout_prob] = np.inf
    return out


def noisy_points(
    xyz: np.ndarray, gaussian_std_m: float, dropout_prob: float, rng: np.random.Generator
) -> tuple[np.ndarray, np.ndarray]:
    """Point-cloud equivalent: move each point along its ray, then drop some.

    Returns the new (N, 3) coordinates and a keep mask. Noise is along the ray from the
    sensor origin because that is the direction a lidar's range error acts in.
    """
    xyz = np.asarray(xyz, dtype=np.float64)
    n = xyz.shape[0]
    out = xyz.copy()
    if gaussian_std_m > 0.0 and n:
        r = np.linalg.norm(xyz, axis=1)
        ok = np.isfinite(r) & (r > 1e-6)
        noisy_r = np.maximum(r[ok] + rng.normal(0.0, gaussian_std_m, int(ok.sum())), 0.0)
        out[ok] = xyz[ok] * (noisy_r / r[ok])[:, None]
    keep = np.ones(n, dtype=bool) if dropout_prob <= 0.0 else rng.random(n) >= dropout_prob
    return out, keep
