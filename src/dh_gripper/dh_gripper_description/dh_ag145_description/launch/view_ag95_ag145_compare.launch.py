from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ag95_prefix",
            default_value="ag95_",
            description="Prefix for AG95 joint/link names.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ag145_prefix",
            default_value="ag145_",
            description="Prefix for AG145 joint/link names.",
        )
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    ag95_prefix = LaunchConfiguration("ag95_prefix")
    ag145_prefix = LaunchConfiguration("ag145_prefix")

    description_package = "dh_ag145_description"
    description_file = "ag95_ag145_compare.urdf.xacro"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "ag95_prefix:=",
            ag95_prefix,
            " ",
            "ag145_prefix:=",
            ag145_prefix,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(
            value=robot_description_content,
            value_type=str,
        )
    }

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_robot.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        declared_arguments
        + [
            joint_state_publisher_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )

