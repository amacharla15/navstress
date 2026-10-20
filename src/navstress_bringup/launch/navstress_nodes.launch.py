"""The NavStress nodes, simulator-agnostic. Pass the params file for the target."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    params = LaunchConfiguration("params_file")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    common = [params, {"use_sim_time": use_sim_time}]

    def node(pkg: str, exe: str, name: str, extra: list | None = None) -> Node:
        return Node(
            package=pkg, executable=exe, name=name, parameters=common + (extra or []),
            output="screen", emulate_tty=True,
        )  # fmt: skip

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", description="NavStress parameters YAML"),
            DeclareLaunchArgument(
                "results_dir",
                default_value=EnvironmentVariable(
                    "NAVSTRESS_RESULTS_DIR", default_value="results"
                ),
                description="where results/<run_id>/ are written (use an absolute path)",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value="",
                description="Nav2 params in use; its sha256 goes into run metadata",
            ),  # fmt: skip
            node("navstress_scenarios", "scenario_manager", "scenario_manager"),
            node("navstress_scenarios", "noise_injector", "noise_injector"),
            node("navstress_scenarios", "obstacle_mover", "obstacle_mover"),
            node("navstress_monitor", "collision_monitor", "navstress_collision_monitor"),
            node(
                "navstress_eval",
                "eval_runner",
                "eval_runner",
                [
                    {
                        "results_dir": LaunchConfiguration("results_dir"),
                        "nav2_params_file": LaunchConfiguration("nav2_params_file"),
                    }
                ],
            ),  # fmt: skip
        ]
    )
