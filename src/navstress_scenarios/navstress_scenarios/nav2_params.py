"""Route Nav2's sensor inputs through the noise relays in a copy of its params file.

Carter's local costmap reads /front_3d_lidar/lidar_points straight from Isaac Sim, so
injecting noise on /scan alone wouldn't reach it. Rather than edit NVIDIA's file, the
launch writes a copy with only the observation-source topics changed. The original
stays the baseline; the copy is named by its content hash and recorded in run
metadata, so every result names the exact Nav2 config it ran with.
"""

from __future__ import annotations

import copy
import hashlib
import os
from collections.abc import Iterator, Mapping
from pathlib import Path

import yaml


def _observation_topics(params: dict) -> Iterator[tuple[str, dict, str]]:
    """Yield (path, source_dict, topic) for every costmap observation source."""
    for node_key, node in params.items():
        if not isinstance(node, dict):
            continue
        # Costmaps nest one level deeper: local_costmap: local_costmap: ros__parameters
        candidates = [node] + [v for v in node.values() if isinstance(v, dict)]
        for c in candidates:
            ros = c.get("ros__parameters")
            if not isinstance(ros, dict):
                continue
            for layer_name, layer in ros.items():
                if not isinstance(layer, dict):
                    continue
                sources = layer.get("observation_sources")
                if not isinstance(sources, str):
                    continue
                for src in sources.split():
                    s = layer.get(src)
                    if isinstance(s, dict) and isinstance(s.get("topic"), str):
                        yield f"{node_key}.{layer_name}.{src}", s, s["topic"]


def reroute(params: dict, topic_map: Mapping[str, str]) -> tuple[dict, list[str]]:
    """Return a copy with observation-source topics renamed, and what was changed."""
    out = copy.deepcopy(params)
    changed = []
    for path, src, topic in _observation_topics(out):
        if topic in topic_map:
            src["topic"] = topic_map[topic]
            changed.append(f"{path}.topic: {topic} -> {topic_map[topic]}")
    return out, changed


def write_rerouted(
    src_file: str | os.PathLike, topic_map: Mapping[str, str], out_dir: str | os.PathLike
) -> tuple[Path, list[str]]:
    """Write the rerouted copy as <out_dir>/nav2_params_<hash12>.yaml and return its path."""
    params = yaml.safe_load(Path(src_file).read_text())
    rerouted, changed = reroute(params, topic_map)
    missing = set(topic_map) - {c.split(": ")[1].split(" -> ")[0] for c in changed}
    if missing:
        raise ValueError(f"{src_file}: no observation source reads {sorted(missing)}")
    text = f"# Generated from {src_file}; only these topics changed:\n"
    text += "".join(f"#   {c}\n" for c in changed)
    text += yaml.safe_dump(rerouted, sort_keys=False)
    digest = hashlib.sha256(text.encode()).hexdigest()[:12]
    out = Path(out_dir) / f"nav2_params_{digest}.yaml"
    out.parent.mkdir(parents=True, exist_ok=True)
    if not out.exists():
        out.write_text(text)
    return out, changed
