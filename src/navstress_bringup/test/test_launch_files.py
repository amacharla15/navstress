"""Every launch file imports and builds its description without a running system.

The Isaac launch file can't run in CI (no GPU, no Isaac Sim), but this still catches
syntax errors, bad imports and renamed arguments before they reach the pod.
"""

import importlib.util
from pathlib import Path

import pytest
from launch.actions import DeclareLaunchArgument

LAUNCH = Path(__file__).resolve().parent.parent / "launch"


def _load(name: str):
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), LAUNCH / name)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.generate_launch_description()


def _args(ld) -> set[str]:
    return {e.name for e in ld.entities if isinstance(e, DeclareLaunchArgument)}


@pytest.mark.parametrize(
    ("name", "expected"),
    [
        ("navstress_nodes.launch.py", {"params_file", "results_dir", "use_sim_time",
                                       "nav2_params_file"}),
        ("fakesim.launch.py", {"results_dir"}),
        ("isaac.launch.py", {"params_file", "nav2_params", "map", "scene", "results_dir",
                             "isaac_sim_root", "launch_isaac", "isaac_args"}),
    ],
)  # fmt: skip
def test_launch_description_builds(name, expected):
    assert expected <= _args(_load(name))


def test_every_params_file_is_valid_yaml_with_ros_parameters():
    import yaml

    for f in (LAUNCH.parent / "params").glob("*.yaml"):
        data = yaml.safe_load(f.read_text())
        assert data and all("ros__parameters" in v for v in data.values()), f
