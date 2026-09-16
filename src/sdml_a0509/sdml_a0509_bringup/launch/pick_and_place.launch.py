from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

ARM_JOINT_STATES_TOPIC = "/a0509/joint_states"
GRIPPER_JOINT_STATES_TOPIC = "/ag145_gripper/joint_states"
GRIPPER_COMMAND_TOPIC = "/ag145/forward_position_controller/commands"
DO_PICK_AND_PLACE_TOPIC = "/do_pick_and_place"


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    robot_model = LaunchConfiguration("robot_model")
    host = LaunchConfiguration("host")
    rt_host = LaunchConfiguration("rt_host")
    gui = LaunchConfiguration("gui")

    declare_mode = DeclareLaunchArgument(
        "mode",
        default_value="virtual",
        description="dsr bringup mode",
    )

    declare_robot_model = DeclareLaunchArgument(
        "robot_model",
        default_value="a0509",
        description="robot model",
    )
    declare_host = DeclareLaunchArgument(
        "host",
        default_value="127.0.0.1",
        description="robot controller host ip",
    )
    declare_rt_host = DeclareLaunchArgument(
        "rt_host",
        default_value="robot-emulator",
        description="robot controller RT host",
    )
    declare_gui = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start RViz2",
    )

    declare_use_merged_joint_states = DeclareLaunchArgument(
        "use_merged_joint_states",
        default_value="false",
        description="Use merged joint_states topic for MoveIt",
    )
    declare_arm_joint_states_topic = DeclareLaunchArgument(
        "arm_joint_states_topic",
        default_value=ARM_JOINT_STATES_TOPIC,
        description="Topic for arm joint states (remapped from /joint_states)",
    )
    declare_joint_states_topic = DeclareLaunchArgument(
        "joint_states_topic",
        default_value="/moveit/joint_states",
        description="Topic MoveIt/robot_state_publisher use for joint_states",
    )
    declare_gripper_joint_states_topic = DeclareLaunchArgument(
        "gripper_joint_states_topic",
        default_value=GRIPPER_JOINT_STATES_TOPIC,
        description="Expanded gripper joint_states topic for the merger",
    )
    declare_gripper_command_topic = DeclareLaunchArgument(
        "gripper_command_topic",
        default_value=GRIPPER_COMMAND_TOPIC,
        description="AG145 command topic",
    )
    declare_do_pick_and_place_topic = DeclareLaunchArgument(
        "do_pick_and_place_topic",
        default_value=DO_PICK_AND_PLACE_TOPIC,
        description="Scenario trigger topic",
    )
    declare_move_line_service = DeclareLaunchArgument(
        "move_line_service",
        default_value="/motion/move_line",
        description="DSR MoveLine service",
    )
    declare_move_joint_service = DeclareLaunchArgument(
        "move_joint_service",
        default_value="/motion/move_joint",
        description="DSR MoveJoint service",
    )
    declare_move_circle_service = DeclareLaunchArgument(
        "move_circle_service",
        default_value="/motion/move_circle",
        description="DSR MoveCircle service",
    )
    declare_scenario_type = DeclareLaunchArgument(
        "scenario_type",
        default_value="pick_place",
        description="pick_place | list",
    )
    dsr_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("dsr_bringup2"),
                "launch",
                "dsr_bringup2_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "mode": mode,
            "model": robot_model,
            "host": host,
            "rt_host": rt_host,
            "gui": gui,
            "joint_states_topic": LaunchConfiguration("joint_states_topic"),
        }.items(),
    )

    gripper_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("dh_gripper_driver"),
                "launch",
                "gripper_control.launch.py",
            ])
        ),
        launch_arguments={
            "mode": mode,
            "gripper_model": "ag145",
            "gripper_ns": "ag145",
            "expanded_joint_states_topic": LaunchConfiguration("gripper_joint_states_topic"),
        }.items(),
    )

    pick_and_place_node = Node(
        package="sdml_a0509_control",
        executable="pick_and_place_node",
        name="pick_and_place_node",
        parameters=[
            {"mode": mode, "model": robot_model, "host": host},
            {"joint_states_topic": LaunchConfiguration("joint_states_topic")},
            {"scenario_type": LaunchConfiguration("scenario_type")},
            {"do_pick_and_place_topic": LaunchConfiguration("do_pick_and_place_topic")},
            {"gripper_command_topic": LaunchConfiguration("gripper_command_topic")},
            {"move_line_service": LaunchConfiguration("move_line_service")},
            {"move_joint_service": LaunchConfiguration("move_joint_service")},
            {"move_circle_service": LaunchConfiguration("move_circle_service")},
        ],
    )

    # /joint_states (arm, no namespace) → /a0509/joint_states
    relay_arm_joints = Node(
        package="topic_tools",
        executable="relay",
        name="arm_joint_relay",
        arguments=["/joint_states", LaunchConfiguration("arm_joint_states_topic")],
        output="screen",
    )

    merge_node = Node(
        package="sdml_a0509_control",
        executable="merge_node",
        name="joint_state_merger",
        parameters=[
            {"arm_joint_states_topic": LaunchConfiguration("arm_joint_states_topic")},
            {"gripper_joint_states_topic": LaunchConfiguration("gripper_joint_states_topic")},
            {"merged_topic": LaunchConfiguration("joint_states_topic")},
        ],
    )

    return LaunchDescription([
        declare_mode,
        declare_robot_model,
        declare_host,
        declare_rt_host,
        declare_gui,
        declare_scenario_type,
        declare_use_merged_joint_states,
        declare_arm_joint_states_topic,
        declare_joint_states_topic,
        declare_gripper_joint_states_topic,
        declare_gripper_command_topic,
        declare_do_pick_and_place_topic,
        declare_move_line_service,
        declare_move_joint_service,
        declare_move_circle_service,
        dsr_moveit_launch,
        gripper_control_launch,
        relay_arm_joints,
        merge_node,
        pick_and_place_node,
    ])
