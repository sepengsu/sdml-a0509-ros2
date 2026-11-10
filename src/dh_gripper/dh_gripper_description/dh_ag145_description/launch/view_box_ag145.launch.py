from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    box_ag145.urdf.xacro 만 로드합니다.
    gripper/box 의 xyz·rpy 는 xacro 기본값·직접 편집으로 고정하세요.
    """
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="dh_ag145_description",
            description="Package containing URDF/XACRO.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="box_ag145.urdf.xacro",
            description="XACRO for AG145 + Gripperbox1/2.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Joint/link prefix for multi-robot setups.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true.",
        ),
    ]

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            TextSubstitution(text=' prefix:="'),
            prefix,
            TextSubstitution(text='"'),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", rviz_config_file],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
