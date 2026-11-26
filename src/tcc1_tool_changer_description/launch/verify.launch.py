"""Open the A0509 TCC1 description through the workspace's simulation launches."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


VIEWERS = {
    "rviz": ("sdml_a0509_bringup", "rviz_only.launch.py"),
    "mujoco": ("a0509_mujoco_sim", "mujoco_sim.launch.py"),
}


def _viewer(context):
    package, filename = VIEWERS[LaunchConfiguration("viewer").perform(context)]
    path = Path(get_package_share_directory(package)) / "launch" / filename
    arguments = {name: LaunchConfiguration(name) for name in (
        "gripper_type", "tool_changer_dynamics_model", "tool_rack_enabled", "tcc1_config"
    )}
    arguments["tool_changer_type"] = "tcc1"
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)), launch_arguments=arguments.items())]


def generate_launch_description():
    config = Path(get_package_share_directory("tcc1_tool_changer_description")) / "config/tcc1_parameters.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="rviz", choices=list(VIEWERS)),
        DeclareLaunchArgument("gripper_type", default_value="ag145", choices=["none", "ag145", "mdpg_1s1"]),
        DeclareLaunchArgument("tool_changer_dynamics_model", default_value="visual", choices=["visual", "dynamics"]),
        DeclareLaunchArgument("tool_rack_enabled", default_value="false", choices=["false", "true"]),
        DeclareLaunchArgument("tcc1_config", default_value=str(config)),
        OpaqueFunction(function=_viewer),
    ])
