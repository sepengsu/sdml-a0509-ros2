"""RViz-only visualization launch for A0509 + AG145."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_model = LaunchConfiguration("robot_model")
    rviz_config = LaunchConfiguration("rviz_config")
    use_gui = LaunchConfiguration("use_gui")

    declare_robot_model = DeclareLaunchArgument(
        "robot_model",
        default_value="a0509",
        description="Robot model xacro name used by dsr_moveit_config_<model>.",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("dsr_moveit_config_a0509"), "launch", "moveit.rviz"]
        ),
        description="RViz config file path.",
    )
    declare_use_gui = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Start joint_state_publisher_gui.",
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("dsr_moveit_config_a0509"),
                    "config",
                    robot_model,
                ]
            ),
            ".urdf.xacro",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(use_gui),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        condition=UnlessCondition(use_gui),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[robot_description],
    )

    # Keep a single launch file for both GUI and non-GUI usage via use_gui argument.
    return LaunchDescription(
        [
            declare_robot_model,
            declare_rviz_config,
            declare_use_gui,
            robot_state_publisher,
            joint_state_publisher_gui,
            joint_state_publisher,
            rviz2,
        ]
    )
