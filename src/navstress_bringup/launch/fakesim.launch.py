"""CI pipeline: the fake simulator plus the NavStress nodes. No GPU, no Isaac Sim, no Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("navstress_bringup")
    params = os.path.join(share, "params", "navstress_fakesim.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "results_dir",
                default_value=EnvironmentVariable(
                    "NAVSTRESS_RESULTS_DIR", default_value="results"
                ),
            ),
            Node(
                package="navstress_fakesim",
                executable="fake_sim",
                name="fake_sim",
                parameters=[params],
                output="screen",
                emulate_tty=True,
            ),  # fmt: skip
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "navstress_nodes.launch.py")
                ),
                launch_arguments={
                    "params_file": params,
                    "results_dir": LaunchConfiguration("results_dir"),
                    "use_sim_time": "false",
                }.items(),
            ),
        ]
    )
