"""Real mode launch: A0509 + real robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    host = LaunchConfiguration("host")
    gui = LaunchConfiguration("gui")
    jog_string_topic = LaunchConfiguration("jog_string_topic")
    jog_multi_axis_topic = LaunchConfiguration("jog_multi_axis_topic")

    declare_host = DeclareLaunchArgument(
        "host",
        default_value="192.168.137.100",
        description="Robot controller IP",
    )

    declare_gui = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start RViz2",
    )

    declare_jog_string_topic = DeclareLaunchArgument(
        "jog_string_topic",
        default_value="/a0509/jog_multi_axis",
        description="Topic Unity/ROS clients publish std_msgs/String JSON jog commands to",
    )

    declare_jog_multi_axis_topic = DeclareLaunchArgument(
        "jog_multi_axis_topic",
        default_value="/a0509/jog_multi_axis_cmd",
        description="Internal dsr_msgs2/JogMultiAxis topic consumed by the jogging bridge",
    )

    pick_and_place_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("sdml_a0509_bringup"),
                "launch",
                "pick_and_place.launch.py",
            ])
        ),
        launch_arguments={
            "mode": "real",
            "robot_model": "a0509",
            "host": host,
            "gui": gui,
        }.items(),
    )

    jog_string_bridge = Node(
        package="sdml_a0509_control",
        executable="jog_string_bridge",
        name="jog_string_bridge",
        output="screen",
        parameters=[{
            "input_topic": jog_string_topic,
            "output_topic": jog_multi_axis_topic,
        }],
    )

    jogging_node = Node(
        package="sdml_a0509_control",
        executable="jogging",
        name="jogging",
        output="screen",
        parameters=[{
            "jog_multi_axis_topic": jog_multi_axis_topic,
            "jog_multi_service": "/motion/jog_multi",
            "move_stop_service": "/motion/move_stop",
        }],
    )

    return LaunchDescription([
        declare_host,
        declare_gui,
        declare_jog_string_topic,
        declare_jog_multi_axis_topic,

        pick_and_place_launch,
        jog_string_bridge,
        jogging_node,
    ])
