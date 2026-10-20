"""Pod launch: Isaac Sim (headless) + Nav2 for Nova Carter + the NavStress nodes.

Mirrors carter_navigation's carter_navigation_isaacsim.launch.xml (IsaacSim-ros_workspaces
tag IsaacSim-6.2.0) with three differences: no RViz (the pod is headless), Isaac Sim
also loads isaacsim.ros2.sim_control, and Nav2's lidar inputs go through the NavStress
noise relays (pointcloud_to_laserscan publishes /scan_raw; the local costmap reads a
rerouted copy of Carter's params). NOT YET RUN ON A POD: Phase 4 validates it.

The Isaac-side packages (carter_navigation, isaacsim_bringup, nav2_bringup,
pointcloud_to_laserscan) are looked up when the launch runs, not declared in
package.xml, because carter_navigation and isaacsim_bringup are not released as debs
and CI's rosdep could not install them.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

from navstress_scenarios.nav2_params import write_rerouted

# Copied from carter_navigation_isaacsim.launch.xml (IsaacSim-6.2.0).
SCENE_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2/"
    "Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd"
)
CLOUD_RELAY = {"/front_3d_lidar/lidar_points": "/navstress/front_3d_lidar/lidar_points"}


def _setup(context):
    lc = LaunchConfiguration
    share = get_package_share_directory("navstress_bringup")
    carter = get_package_share_directory("carter_navigation")
    navstress_params = lc("params_file").perform(context) or os.path.join(
        share, "params", "navstress_isaac.yaml"
    )
    nav2_src = lc("nav2_params").perform(context) or os.path.join(
        carter, "params", "carter_navigation_params.yaml"
    )
    nav2_map = lc("map").perform(context) or os.path.join(
        carter, "maps", "carter_warehouse_navigation.yaml"
    )
    results_dir = lc("results_dir").perform(context)
    nav2_params, changed = write_rerouted(
        nav2_src, CLOUD_RELAY, os.path.join(results_dir, "nav2_params")
    )
    actions = [
        LogInfo(msg=f"Nav2 params: {nav2_params} (from {nav2_src}; {'; '.join(changed)})"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("nav2_bringup"), "launch", "bringup_launch.py"
                )
            ),  # fmt: skip
            launch_arguments={
                "map": nav2_map,
                "use_sim_time": "True",
                "params_file": str(nav2_params),
                "autostart": "True",
            }.items(),
        ),
        # Same settings as carter_navigation.launch.xml; only the output topic differs.
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            remappings=[("cloud_in", "/front_3d_lidar/lidar_points"), ("scan", "/scan_raw")],
            parameters=[
                {
                    "target_frame": "front_3d_lidar",
                    "transform_tolerance": 0.01,
                    "min_height": -0.4,
                    "max_height": 1.5,
                    "angle_min": -1.5708,
                    "angle_max": 1.5708,
                    "angle_increment": 0.0087,
                    "scan_time": 0.3333,
                    "range_min": 0.05,
                    "range_max": 100.0,
                    "use_inf": True,
                    "inf_epsilon": 1.0,
                    "use_sim_time": True,
                }
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", "navstress_nodes.launch.py")
            ),
            launch_arguments={
                "params_file": navstress_params,
                "results_dir": results_dir,
                "use_sim_time": "true",
                "nav2_params_file": str(nav2_params),
            }.items(),
        ),
        # Second relay: the 3D cloud the local costmap's voxel layer reads.
        Node(
            package="navstress_scenarios",
            executable="noise_injector",
            name="noise_injector_cloud",
            parameters=[navstress_params, {"use_sim_time": True}],
            output="screen",
        ),  # fmt: skip
    ]
    return actions


def _isaac(context):
    lc = LaunchConfiguration
    custom = "--no-window --enable isaacsim.ros2.sim_control " + lc("isaac_args").perform(context)
    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("isaacsim_bringup"),
                    "launch",
                    "run_isaacsim.launch.xml",
                )
            ),  # fmt: skip
            launch_arguments={
                "version": "6.2.0",
                "install_path": lc("isaac_sim_root").perform(context),
                "gui": lc("scene").perform(context),
                "play_sim_on_start": "True",
                "custom_args": custom.strip(),
            }.items(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value="",
                                  description="NavStress params (default: navstress_isaac.yaml)"),
            DeclareLaunchArgument("nav2_params", default_value="",
                                  description="Nav2 params to start from (default: Carter's). "
                                  "Phase 6 passes the fix here."),
            DeclareLaunchArgument("map", default_value="",
                                  description="map YAML (default: Carter warehouse map)"),
            DeclareLaunchArgument("scene", default_value=SCENE_URL),
            DeclareLaunchArgument(
                "results_dir",
                default_value=EnvironmentVariable("NAVSTRESS_RESULTS_DIR",
                                                  default_value="results"),
            ),
            DeclareLaunchArgument(
                "isaac_sim_root",
                default_value=EnvironmentVariable("NAVSTRESS_ISAAC_SIM_ROOT",
                                                  default_value="/isaac-sim"),
            ),
            DeclareLaunchArgument("launch_isaac", default_value="true",
                                  description="false if Isaac Sim is already running"),
            DeclareLaunchArgument("isaac_args", default_value="",
                                  description="extra Isaac Sim command-line arguments"),
            OpaqueFunction(
                function=_isaac, condition=IfCondition(LaunchConfiguration("launch_isaac"))
            ),
            OpaqueFunction(function=_setup),
        ]
    )  # fmt: skip
