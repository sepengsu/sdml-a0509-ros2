"""Open the two-step right-angle JSON in the walled simulation GUI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    wall_practice = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("sim_real_gap_getting"), "launch", "forward_wall.launch.py"])),
        launch_arguments={
            "sim_domain_id": LaunchConfiguration("sim_domain_id"),
            "real_domain_id": LaunchConfiguration("real_domain_id"),
            "trajectory_file": PathJoinSubstitution([
                FindPackageShare("sim_real_gap_getting"), "examples", "right_angle_20cm_twice.json"]),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("sim_domain_id", default_value="173"),
        DeclareLaunchArgument("real_domain_id", default_value="0"),
        wall_practice,
    ])
