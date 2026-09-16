"""Launch jogging node (sdml_a0509_control) + position getter GUI + robot_mode_node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_dsr_set_robot_mode_service = DeclareLaunchArgument(
        "dsr_set_robot_mode_service",
        default_value="/system/set_robot_mode",
        description="DSR controller SetRobotMode service",
    )
    declare_gripper_command_topic = DeclareLaunchArgument(
        "gripper_command_topic",
        default_value="/ag145/forward_position_controller/commands",
        description="AG145 gripper command topic",
    )
    declare_do_pick_and_place_topic = DeclareLaunchArgument(
        "do_pick_and_place_topic",
        default_value="/do_pick_and_place",
        description="Pick and place trigger topic",
    )
    declare_get_posx_service = DeclareLaunchArgument(
        "get_posx_service",
        default_value="/aux_control/get_current_posx",
        description="DSR GetCurrentPosx service",
    )
    declare_get_posj_service = DeclareLaunchArgument(
        "get_posj_service",
        default_value="/aux_control/get_current_posj",
        description="DSR GetCurrentPosj service",
    )
    declare_get_solution_space_service = DeclareLaunchArgument(
        "get_solution_space_service",
        default_value="/aux_control/get_current_solution_space",
        description="DSR GetCurrentSolutionSpace service",
    )
    declare_jog_service = DeclareLaunchArgument(
        "jog_service",
        default_value="/motion/jog",
        description="DSR Jog service",
    )
    declare_jog_multi_service = DeclareLaunchArgument(
        "jog_multi_service",
        default_value="/motion/jog_multi",
        description="DSR JogMulti service",
    )
    declare_move_joint_service = DeclareLaunchArgument(
        "move_joint_service",
        default_value="/motion/move_joint",
        description="DSR MoveJoint service",
    )
    declare_move_line_service = DeclareLaunchArgument(
        "move_line_service",
        default_value="/motion/move_line",
        description="DSR MoveLine service",
    )
    declare_move_stop_service = DeclareLaunchArgument(
        "move_stop_service",
        default_value="/motion/move_stop",
        description="DSR MoveStop service",
    )
    declare_parallel_axis2_service = DeclareLaunchArgument(
        "parallel_axis2_service",
        default_value="/force/parallel_axis2",
        description="DSR ParallelAxis2 service",
    )
    declare_jog_string_topic = DeclareLaunchArgument(
        "jog_string_topic",
        default_value="/a0509/jog_multi_axis",
        description="Unity/ROSSharp std_msgs/String JSON jog command topic",
    )
    declare_jog_multi_axis_topic = DeclareLaunchArgument(
        "jog_multi_axis_topic",
        default_value="/a0509/jog_multi_axis_cmd",
        description="Internal dsr_msgs2/JogMultiAxis topic consumed by the jogging bridge",
    )
    declare_start_jogging_bridge = DeclareLaunchArgument(
        "start_jogging_bridge",
        default_value="false",
        description="Start jog_string_bridge and jogging here. Keep false when a0509_virtual/real launch is already running.",
    )

    jog_string_bridge = Node(
        package="sdml_a0509_control",
        executable="jog_string_bridge",
        name="jog_string_bridge",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_jogging_bridge")),
        parameters=[{
            "input_topic": LaunchConfiguration("jog_string_topic"),
            "output_topic": LaunchConfiguration("jog_multi_axis_topic"),
        }],
    )

    jogging_node = Node(
        package="sdml_a0509_control",
        executable="jogging",
        name="jogging",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_jogging_bridge")),
        parameters=[{
            "jog_multi_axis_topic": LaunchConfiguration("jog_multi_axis_topic"),
            "jog_multi_service": LaunchConfiguration("jog_multi_service"),
            "move_stop_service": LaunchConfiguration("move_stop_service"),
        }],
    )
    robot_mode_node = Node(
        package="a0509_position_getter",
        executable="robot_mode_node",
        name="robot_mode_node",
        output="screen",
        parameters=[{
            "dsr_set_robot_mode_service": LaunchConfiguration("dsr_set_robot_mode_service"),
        }],
    )
    getter_node = Node(
        package="a0509_position_getter",
        executable="position_getter_gui",
        name="position_getter_node",
        output="screen",
        parameters=[{
            "get_posx_service": LaunchConfiguration("get_posx_service"),
            "get_posj_service": LaunchConfiguration("get_posj_service"),
            "get_solution_space_service": LaunchConfiguration("get_solution_space_service"),
            "jog_service": LaunchConfiguration("jog_service"),
            "jog_multi_service": LaunchConfiguration("jog_multi_service"),
            "move_joint_service": LaunchConfiguration("move_joint_service"),
            "move_line_service": LaunchConfiguration("move_line_service"),
            "move_stop_service": LaunchConfiguration("move_stop_service"),
            "set_robot_mode_service": "set_robot_mode",
            "parallel_axis2_service": LaunchConfiguration("parallel_axis2_service"),
            "do_pick_and_place_topic": LaunchConfiguration("do_pick_and_place_topic"),
            "gripper_command_topic": LaunchConfiguration("gripper_command_topic"),
        }],
    )
    return LaunchDescription([
        declare_dsr_set_robot_mode_service,
        declare_gripper_command_topic,
        declare_do_pick_and_place_topic,
        declare_get_posx_service,
        declare_get_posj_service,
        declare_get_solution_space_service,
        declare_jog_service,
        declare_jog_multi_service,
        declare_move_joint_service,
        declare_move_line_service,
        declare_move_stop_service,
        declare_parallel_axis2_service,
        declare_jog_string_topic,
        declare_jog_multi_axis_topic,
        declare_start_jogging_bridge,
        jog_string_bridge,
        jogging_node,
        robot_mode_node,
        getter_node,
    ])
