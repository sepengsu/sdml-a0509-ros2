"""Explicit read-only real RT bringup in a domain separate from MuJoCo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    upstream = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("a0509_dynamics_experiment"), "launch", "real_rt.launch.py"])),
        launch_arguments={
            "host": LaunchConfiguration("host"),
            "rt_host": LaunchConfiguration("rt_host"),
            "gripper_type": LaunchConfiguration("gripper_type"),
            "mdpg_mount_xyz": LaunchConfiguration("mdpg_mount_xyz"),
            "mdpg_mount_rpy": LaunchConfiguration("mdpg_mount_rpy"),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("real_domain_id", default_value="0"),
        DeclareLaunchArgument("host", default_value="192.168.137.100"),
        DeclareLaunchArgument("rt_host", default_value="192.168.137.50"),
        DeclareLaunchArgument("gripper_type", default_value="none"),
        DeclareLaunchArgument("mdpg_mount_xyz", default_value="TODO_MEASURE"),
        DeclareLaunchArgument("mdpg_mount_rpy", default_value="TODO_MEASURE"),
        GroupAction([
            SetEnvironmentVariable("ROS_DOMAIN_ID", LaunchConfiguration("real_domain_id")),
            upstream,
        ]),
    ])
