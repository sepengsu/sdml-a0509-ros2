"""Virtual mode launch: A0509 + emulator."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    host = LaunchConfiguration("host", default="host.docker.internal")
    rt_host = LaunchConfiguration("rt_host", default="robot-emulator")
    gui = LaunchConfiguration("gui", default="true")
    jog_string_topic = LaunchConfiguration("jog_string_topic", default="/a0509/jog_multi_axis")
    jog_multi_axis_topic = LaunchConfiguration("jog_multi_axis_topic", default="/a0509/jog_multi_axis_cmd")

    declare_host = DeclareLaunchArgument(
        "host",
        default_value="127.0.0.1",
        description="Robot/emulator host",
    )
    declare_rt_host = DeclareLaunchArgument(
        "rt_host",
        default_value="robot-emulator",
        description="Robot/emulator RT host",
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
            "mode": "virtual",
            "robot_model": "a0509",
            "host": host,
            "rt_host": rt_host,
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
        output="screen",
        parameters=[{
            "jog_multi_axis_topic": jog_multi_axis_topic,
            "jog_multi_service": "/motion/jog_multi",
            "move_stop_service": "/motion/move_stop",
        }],
    )

    return LaunchDescription([
        declare_host,
        declare_rt_host,
        declare_gui,
        declare_jog_string_topic,
        declare_jog_multi_axis_topic,
        pick_and_place_launch,
        jog_string_bridge,
        jogging_node,
    ])
