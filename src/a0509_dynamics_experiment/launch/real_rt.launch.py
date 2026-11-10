"""Explicit real-robot RT data bringup. Never launched by run_experiment.

Uses the same Doosan controller stack as a0509_real.launch.py, with RT
publication enabled and without motion scenario/jogging nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    host = LaunchConfiguration("host")
    rt_host = LaunchConfiguration("rt_host")
    robot_description = {"robot_description": Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("dsr_description2"), "xacro", "a0509.urdf.xacro"]),
        " name:= host:=", host, " rt_host:=", rt_host,
        " port:=12345 mode:=real model:=a0509 update_rate:=100",
    ])}
    controller_file = PathJoinSubstitution([
        FindPackageShare("a0509_dynamics_experiment"), "config", "dsr_controller2_rt.yaml"
    ])
    robot_connection = Node(package="dsr_bringup2", executable="run_emulator", output="screen",
                            parameters=[{"name": "", "rate": 100, "standby": 5000, "command": True,
                                         "host": host, "port": 12345, "mode": "real", "model": "a0509",
                                         "gripper": "none", "mobile": "none", "rt_host": rt_host}])
    controller = Node(package="controller_manager", executable="ros2_control_node", output="screen",
                      parameters=[robot_description, controller_file])
    state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher",
                           parameters=[robot_description], output="screen")
    jsb = Node(package="controller_manager", executable="spawner", output="screen",
               arguments=["joint_state_broadcaster", "-c", "controller_manager",
                          "--controller-manager-timeout", "120"])
    dsr = Node(package="controller_manager", executable="spawner", output="screen",
               arguments=["dsr_controller2", "-c", "controller_manager",
                          "--controller-manager-timeout", "120"])
    return LaunchDescription([
        DeclareLaunchArgument("host", default_value="192.168.137.100"),
        DeclareLaunchArgument("rt_host", default_value="192.168.137.50"),
        robot_connection, state_publisher, controller,
        RegisterEventHandler(OnProcessStart(target_action=controller,
                                             on_start=[TimerAction(period=5.0, actions=[jsb])])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[dsr])),
    ])
