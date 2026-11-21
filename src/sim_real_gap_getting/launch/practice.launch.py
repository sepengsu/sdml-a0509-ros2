"""Start the A0509 MuJoCo scene and the JSON trajectory GUI in one ROS domain."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _validate_domains(context):
    sim = int(LaunchConfiguration("sim_domain_id").perform(context))
    real = int(LaunchConfiguration("real_domain_id").perform(context))
    if not 0 <= sim <= 232 or not 0 <= real <= 232 or sim == real:
        raise RuntimeError("sim_domain_id and real_domain_id must be distinct values in 0..232")
    return []


def generate_launch_description():
    sim = LaunchConfiguration("sim_domain_id")
    real = LaunchConfiguration("real_domain_id")
    trajectory = LaunchConfiguration("trajectory_file")
    gripper = LaunchConfiguration("gripper_type")
    mount_xyz = LaunchConfiguration("mdpg_mount_xyz")
    mount_rpy = LaunchConfiguration("mdpg_mount_rpy")
    dynamics = LaunchConfiguration("mdpg_dynamics_model")
    model_path = LaunchConfiguration("mujoco_model_path")
    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("a0509_mujoco_sim"), "launch", "mujoco_sim.launch.py"])),
        launch_arguments={
            "gripper_type": gripper,
            "mdpg_mount_xyz": mount_xyz,
            "mdpg_mount_rpy": mount_rpy,
            "mdpg_dynamics_model": dynamics,
            "mujoco_model_path": model_path,
            "use_sim_time": "true",
        }.items(),
    )
    gui = Node(
        package="sim_real_gap_getting", executable="trajectory_gui", output="screen",
        arguments=["--trajectory", trajectory, "--real-domain-id", real,
                   "--gripper-type", gripper],
    )
    return LaunchDescription([
        DeclareLaunchArgument("sim_domain_id", default_value="173"),
        DeclareLaunchArgument("real_domain_id", default_value="0"),
        DeclareLaunchArgument("trajectory_file", default_value=PathJoinSubstitution([
            FindPackageShare("sim_real_gap_getting"), "examples", "small_joint_1.json"])),
        DeclareLaunchArgument("gripper_type", default_value="none"),
        DeclareLaunchArgument("mdpg_mount_xyz", default_value="TODO_MEASURE"),
        DeclareLaunchArgument("mdpg_mount_rpy", default_value="TODO_MEASURE"),
        DeclareLaunchArgument("mdpg_dynamics_model", default_value="visual"),
        DeclareLaunchArgument("mujoco_model_path", default_value="auto"),
        OpaqueFunction(function=_validate_domains),
        GroupAction([
            SetEnvironmentVariable("ROS_DOMAIN_ID", sim),
            simulator,
            gui,
        ]),
    ])
