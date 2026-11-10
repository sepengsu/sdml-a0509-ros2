"""MuJoCo bringup launch for A0509 (optional RViz)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    model = LaunchConfiguration("model")
    color = LaunchConfiguration("color")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    gripper_type = LaunchConfiguration("gripper_type")
    mdpg_mount_xyz = LaunchConfiguration("mdpg_mount_xyz")
    mdpg_mount_rpy = LaunchConfiguration("mdpg_mount_rpy")

    declare_model = DeclareLaunchArgument("model", default_value="a0509", description="ROBOT_MODEL")
    declare_color = DeclareLaunchArgument("color", default_value="white", description="ROBOT_COLOR")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time."
    )
    declare_start_rviz = DeclareLaunchArgument(
        "start_rviz", default_value="true", description="Start RViz2 with robot model."
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("sdml_a0509_description"), "rviz", "a0509_tool.rviz"]
        ),
        description="RViz config path.",
    )

    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("a0509_mujoco_sim"), "launch", "mujoco_sim.launch.py"])
        ),
        launch_arguments={
            "model": model,
            "color": color,
            "use_mujoco": "true",
            "use_sim_time": use_sim_time,
            "gripper_type": gripper_type,
            "mdpg_mount_xyz": mdpg_mount_xyz,
            "mdpg_mount_rpy": mdpg_mount_rpy,
        }.items(),
    )

    # Mirror the same robot_description used by MuJoCo.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("sdml_a0509_description"), "urdf", "a0509_with_gripper.urdf.xacro"]),
            " use_mujoco:=true",
            " color:=",
            color,
            " namespace:=",
            " gripper_type:=", gripper_type,
            " mdpg_mount_xyz:='", mdpg_mount_xyz, "'",
            " mdpg_mount_rpy:='", mdpg_mount_rpy, "'",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(start_rviz),
        arguments=["-d", rviz_config],
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_model,
            DeclareLaunchArgument("gripper_type", default_value="none", description="none | ag145 | mdpg_1s1"),
            DeclareLaunchArgument("mdpg_mount_xyz", default_value="TODO_MEASURE"),
            DeclareLaunchArgument("mdpg_mount_rpy", default_value="TODO_MEASURE"),
            declare_color,
            declare_use_sim_time,
            declare_start_rviz,
            declare_rviz_config,
            mujoco_launch,
            robot_state_publisher,
            rviz_node,
        ]
    )
