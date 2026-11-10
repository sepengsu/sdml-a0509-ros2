"""RViz-only visualization launch for the selected A0509 end effector."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import math


def _validate_tool(context):
    if LaunchConfiguration("robot_model").perform(context) != "a0509":
        raise RuntimeError("rviz_only tool description supports only robot_model:=a0509")
    selected = LaunchConfiguration("gripper_type").perform(context)
    if selected not in ("none", "ag145", "mdpg_1s1"):
        raise RuntimeError(f"Unsupported gripper_type: {selected}")
    dynamics_model = LaunchConfiguration("mdpg_dynamics_model").perform(context)
    if dynamics_model not in ("visual", "box_approx") or (selected != "mdpg_1s1" and dynamics_model != "visual"):
        raise RuntimeError("mdpg_dynamics_model must be visual or box_approx with gripper_type:=mdpg_1s1")
    if selected == "mdpg_1s1":
        for key in ("mdpg_mount_xyz", "mdpg_mount_rpy"):
            try:
                values = [float(part) for part in LaunchConfiguration(key).perform(context).split()]
                if len(values) != 3 or not all(math.isfinite(part) for part in values):
                    raise ValueError()
            except ValueError as error:
                raise RuntimeError(f"{key} is TODO; supply three measured values") from error
    return []


def generate_launch_description():
    robot_model = LaunchConfiguration("robot_model")
    rviz_config = LaunchConfiguration("rviz_config")
    use_gui = LaunchConfiguration("use_gui")
    gripper_type = LaunchConfiguration("gripper_type")
    mdpg_mount_xyz = LaunchConfiguration("mdpg_mount_xyz")
    mdpg_mount_rpy = LaunchConfiguration("mdpg_mount_rpy")
    mdpg_dynamics_model = LaunchConfiguration("mdpg_dynamics_model")

    declare_robot_model = DeclareLaunchArgument(
        "robot_model",
        default_value="a0509",
        description="Robot model xacro name used by dsr_moveit_config_<model>.",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("sdml_a0509_description"), "rviz", "a0509_tool.rviz"]
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
                    FindPackageShare("sdml_a0509_description"),
                    "urdf",
                    "a0509_with_gripper.urdf.xacro",
                ]
            ),
            " gripper_type:=", gripper_type,
            " mdpg_mount_xyz:='", mdpg_mount_xyz, "'",
            " mdpg_mount_rpy:='", mdpg_mount_rpy, "'",
            " mdpg_dynamics_model:=", mdpg_dynamics_model,
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
            DeclareLaunchArgument("gripper_type", default_value="none", description="none | ag145 | mdpg_1s1"),
            DeclareLaunchArgument("mdpg_mount_xyz", default_value="TODO_MEASURE", description="Measured link_6 mount xyz (m)"),
            DeclareLaunchArgument("mdpg_mount_rpy", default_value="TODO_MEASURE", description="Measured link_6 mount rpy (rad)"),
            DeclareLaunchArgument("mdpg_dynamics_model", default_value="visual", description="visual | box_approx"),
            declare_rviz_config,
            declare_use_gui,
            OpaqueFunction(function=_validate_tool),
            robot_state_publisher,
            joint_state_publisher_gui,
            joint_state_publisher,
            rviz2,
        ]
    )
