"""A0509 Gazebo (GZ) + 시뮬 RGB-D 카메라 + RViz / RQt 이미지 뷰."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from dsr_bringup2.utils import read_update_rate


def generate_launch_description():
    update_rate = str(read_update_rate())
    xacro_path = os.path.join(get_package_share_directory("dsr_description2"), "xacro")

    ARGUMENTS = [
        DeclareLaunchArgument("name", default_value="dsr01", description="ROS namespace"),
        DeclareLaunchArgument("host", default_value="127.0.0.1", description="Robot / emulator IP"),
        DeclareLaunchArgument("port", default_value="12345", description="Robot port"),
        DeclareLaunchArgument("mode", default_value="virtual", description="Operation mode"),
        DeclareLaunchArgument("model", default_value="a0509", description="Robot model (xacro)"),
        DeclareLaunchArgument("color", default_value="white", description="Robot color"),
        DeclareLaunchArgument("gui", default_value="true", description="Start RViz2 after controllers"),
        DeclareLaunchArgument("gz", default_value="true", description="Enable Gazebo spawn pipeline"),
        DeclareLaunchArgument("x", default_value="0", description="Spawn x"),
        DeclareLaunchArgument("y", default_value="0", description="Spawn y"),
        DeclareLaunchArgument("z", default_value="0", description="Spawn z"),
        DeclareLaunchArgument("R", default_value="0", description="Spawn roll"),
        DeclareLaunchArgument("P", default_value="0", description="Spawn pitch"),
        DeclareLaunchArgument("Y", default_value="0", description="Spawn yaw"),
        DeclareLaunchArgument("rt_host", default_value="robot-emulator", description="RT host (virtual mode)"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
        DeclareLaunchArgument(
            "show_rqt_rgb",
            default_value="true",
            description="RQt image view for RGB (/rgbd_camera/image)",
        ),
        DeclareLaunchArgument(
            "show_rqt_depth",
            default_value="true",
            description="RQt image view for depth (/rgbd_camera/depth_image)",
        ),
        DeclareLaunchArgument(
            "rgb_topic",
            default_value="/rgbd_camera/image",
            description="ROS topic for RQt RGB viewer",
        ),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/rgbd_camera/depth_image",
            description="ROS topic for RQt depth viewer",
        ),
    ]

    set_use_sim_time = SetLaunchConfiguration(name="use_sim_time", value="true")

    gui = LaunchConfiguration("gui")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("dsr_description2"), "xacro", LaunchConfiguration("model")]
            ),
            ".urdf.xacro",
            " name:=", LaunchConfiguration("name"),
            " host:=", LaunchConfiguration("host"),
            " rt_host:=", LaunchConfiguration("rt_host"),
            " port:=", LaunchConfiguration("port"),
            " mode:=", LaunchConfiguration("mode"),
            " model:=", LaunchConfiguration("model"),
            " update_rate:=", update_rate,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("dsr_controller2"), "config", "dsr_controller2.yaml"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("dsr_description2"), "rviz", "default.rviz"]
    )
    jsb_config = PathJoinSubstitution(
        [FindPackageShare("dsr_controller2"), "config", "joint_state_broadcaster.yaml"]
    )

    run_emulator_node = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        namespace=LaunchConfiguration("name"),
        parameters=[
            {"name": LaunchConfiguration("name")},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": LaunchConfiguration("host")},
            {"port": LaunchConfiguration("port")},
            {"mode": LaunchConfiguration("mode")},
            {"model": LaunchConfiguration("model")},
            {"gripper": "none"},
            {"mobile": "none"},
            {"rt_host": LaunchConfiguration("rt_host")},
        ],
        output="screen",
    )

    gazebo_connection_node = Node(
        package="dsr_bringup2",
        executable="gazebo_connection",
        namespace=LaunchConfiguration("name"),
        parameters=[{"model": LaunchConfiguration("model")}],
        output="log",
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration("name"),
        parameters=[robot_description, robot_controllers],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=LaunchConfiguration("name"),
        output="both",
        parameters=[
            {
                "robot_description": Command(
                    [
                        "xacro",
                        " ",
                        xacro_path,
                        "/",
                        LaunchConfiguration("model"),
                        ".urdf.xacro color:=",
                        LaunchConfiguration("color"),
                    ]
                ),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    # GZ world with rgbd_camera model (Doosan visual_servoing SDF) + ros_gz bridge + optional RQt from upstream
    rgbd_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("dsr_visualservoing"), "/launch/rgbd_camera.launch.py"]
        ),
        launch_arguments={
            "rqt": LaunchConfiguration("show_rqt_rgb"),
            "image_topic": LaunchConfiguration("rgb_topic"),
        }.items(),
    )

    rqt_depth = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        arguments=[LaunchConfiguration("depth_topic")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("show_rqt_depth")),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=LaunchConfiguration("name"),
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration("name"),
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
        parameters=[jsb_config],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        namespace=LaunchConfiguration("name"),
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )

    delay_robot_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    included_launch_file_path = os.path.join(
        get_package_share_directory("dsr_visualservoing"),
        "launch",
        "dsr_gazebo_visual_servoing.launch.py",
    )
    included_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(included_launch_file_path),
        launch_arguments={
            "use_gazebo": LaunchConfiguration("gz"),
            "name": LaunchConfiguration("name"),
            "model": LaunchConfiguration("model"),
            "color": LaunchConfiguration("color"),
            "x": LaunchConfiguration("x"),
            "y": LaunchConfiguration("y"),
            "z": LaunchConfiguration("z"),
            "R": LaunchConfiguration("R"),
            "P": LaunchConfiguration("P"),
            "Y": LaunchConfiguration("Y"),
        }.items(),
    )

    delay_spawn_and_rviz_after_robot_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[included_launch, rviz_node],
        )
    )

    return LaunchDescription(
        ARGUMENTS
        + [
            set_use_sim_time,
            rgbd_camera_launch,
            rqt_depth,
            run_emulator_node,
            gazebo_connection_node,
            robot_state_pub_node,
            control_node,
            joint_state_broadcaster_spawner,
            delay_robot_controller_after_jsb,
            delay_spawn_and_rviz_after_robot_controller,
        ]
    )
