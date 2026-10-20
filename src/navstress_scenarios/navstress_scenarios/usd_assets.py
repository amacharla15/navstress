"""Obstacle assets written as small USDA files that SpawnEntities can load by URI.

isaacsim.ros2.sim_control spawns from a USD file URI only (it ignores resource
strings), so each obstacle size gets its own file. Files are named by their content,
so the same size is written once and reused across scenarios.
"""

from __future__ import annotations

import hashlib
import os
import re
import tempfile
from pathlib import Path

# Static obstacles are plain colliders; moving ones are kinematic rigid bodies, which
# is how PhysX expects externally driven objects to be represented.
_TEMPLATE = """#usda 1.0
(
    customLayerData = {{
        string navstress_asset = "{asset}"
        double navstress_size_x = {sx}
        double navstress_size_y = {sy}
        double navstress_height = {h}
        bool navstress_kinematic = {kin_bool}
    }}
    defaultPrim = "obstacle"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "obstacle"{root_meta}
{{
{root_attrs}    def {shape} "geom" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {{
{shape_attrs}        double3 xformOp:translate = (0, 0, {half_h})
        float3 xformOp:scale = ({scale_x}, {scale_y}, {h})
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
    }}
}}
"""


def _fmt(v: float) -> str:
    return f"{float(v):.6g}"


def obstacle_usda(asset: str, size_x: float, size_y: float, height: float, kinematic: bool) -> str:
    """USDA for a unit primitive scaled to size, with its base on the spawn plane."""
    if asset == "box":
        shape, shape_attrs = "Cube", "        double size = 1\n"
        scale_x, scale_y = size_x, size_y
    elif asset == "cylinder":
        shape = "Cylinder"
        shape_attrs = (
            '        uniform token axis = "Z"\n'
            "        double height = 1\n"
            "        double radius = 0.5\n"
        )
        scale_x = scale_y = size_x  # cylinders use size_x as the diameter
    else:
        raise ValueError(f"no USD generator for asset {asset!r}")
    if kinematic:
        root_meta = ' (\n    prepend apiSchemas = ["PhysicsRigidBodyAPI"]\n)'
        root_attrs = "    bool physics:kinematicEnabled = 1\n"
    else:
        # An empty apiSchemas list is a USDA parse error, so omit the block entirely.
        root_meta, root_attrs = "", ""
    return _TEMPLATE.format(
        asset=asset,
        sx=_fmt(size_x),
        sy=_fmt(size_y if asset == "box" else size_x),
        h=_fmt(height),
        half_h=_fmt(height / 2.0),
        kin_bool="1" if kinematic else "0",
        root_meta=root_meta,
        root_attrs=root_attrs,
        shape=shape,
        shape_attrs=shape_attrs,
        scale_x=_fmt(scale_x),
        scale_y=_fmt(scale_y),
    )


def ensure_asset(
    cache_dir: str | os.PathLike,
    asset: str,
    size_x: float,
    size_y: float,
    height: float,
    kinematic: bool,
) -> Path:
    """Write the asset if it doesn't exist yet and return its absolute path."""
    text = obstacle_usda(asset, size_x, size_y, height, kinematic)
    digest = hashlib.sha256(text.encode()).hexdigest()[:16]
    cache = Path(cache_dir).resolve()
    cache.mkdir(parents=True, exist_ok=True)
    path = cache / f"navstress_{asset}_{digest}.usda"
    if not path.exists():
        # Write-then-rename so a concurrent reader never sees half a file.
        fd, tmp = tempfile.mkstemp(dir=cache, suffix=".tmp")
        with os.fdopen(fd, "w") as f:
            f.write(text)
        os.replace(tmp, path)
    return path


_META = re.compile(r"(string|double|bool) navstress_(\w+) = (\"[^\"]*\"|[-0-9.eE+]+)")


def read_metadata(path: str | os.PathLike) -> dict:
    """Read back the navstress_* custom layer data (used by the fake simulator)."""
    out: dict = {}
    for kind, key, value in _META.findall(Path(path).read_text()):
        if kind == "string":
            out[key] = value.strip('"')
        elif kind == "bool":
            out[key] = value == "1"
        else:
            out[key] = float(value)
    if "asset" not in out:
        raise ValueError(f"{path} is not a NavStress obstacle asset")
    return out
