"""Start the J1-fixed forward practice with a visible and collidable rear wall."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    practice = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("sim_real_gap_getting"), "launch", "practice.launch.py"])),
        launch_arguments={
            "sim_domain_id": LaunchConfiguration("sim_domain_id"),
            "real_domain_id": LaunchConfiguration("real_domain_id"),
            "gripper_type": "none",
            "trajectory_file": LaunchConfiguration("trajectory_file"),
            "mujoco_model_path": PathJoinSubstitution([
                FindPackageShare("dsr_description2"), "mujoco_models", "a0509", "scene_forward_wall.xml"]),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("sim_domain_id", default_value="173"),
        DeclareLaunchArgument("real_domain_id", default_value="0"),
        DeclareLaunchArgument("trajectory_file", default_value=PathJoinSubstitution([
            FindPackageShare("sim_real_gap_getting"), "examples", "forward_150mm_wall.json"])),
        practice,
    ])
