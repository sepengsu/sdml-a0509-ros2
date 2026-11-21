"""Real mode launch: A0509 + real robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import math


def _validate_tool(context):
    selected = LaunchConfiguration("gripper_type").perform(context)
    if selected not in ("ag145", "mdpg_1s1"):
        raise RuntimeError(f"Unsupported real gripper_type: {selected}")
    experiment_only = LaunchConfiguration("experiment_only").perform(context)
    if experiment_only not in ("true", "false"):
        raise RuntimeError("experiment_only must be true or false")
    if selected == "mdpg_1s1":
        for key in ("mdpg_mount_xyz", "mdpg_mount_rpy"):
            value = LaunchConfiguration(key).perform(context)
            try:
                values = [float(part) for part in value.split()]
                if len(values) != 3 or not all(math.isfinite(part) for part in values):
                    raise ValueError()
            except ValueError as error:
                raise RuntimeError(f"{key} is TODO; supply three measured values") from error
    return []


def generate_launch_description():
    host = LaunchConfiguration("host")
    rt_host = LaunchConfiguration("rt_host")
    gui = LaunchConfiguration("gui")
    jog_string_topic = LaunchConfiguration("jog_string_topic")
    jog_multi_axis_topic = LaunchConfiguration("jog_multi_axis_topic")
    gripper_type = LaunchConfiguration("gripper_type")
    experiment_only = LaunchConfiguration("experiment_only")
    mdpg_mount_xyz = LaunchConfiguration("mdpg_mount_xyz")
    mdpg_mount_rpy = LaunchConfiguration("mdpg_mount_rpy")
    ag145_scenario = IfCondition(PythonExpression([
        "'", gripper_type, "' == 'ag145' and '", experiment_only, "' == 'false'"
    ]))
    rt_state_only = IfCondition(PythonExpression([
        "'", gripper_type, "' == 'mdpg_1s1' or '", experiment_only, "' == 'true'"
    ]))

    declare_host = DeclareLaunchArgument(
        "host",
        default_value="192.168.137.100",
        description="Robot controller IP",
    )

    declare_gui = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start the real collection GUI in experiment mode; RViz otherwise",
    )
    declare_rt_host = DeclareLaunchArgument(
        "rt_host", default_value="192.168.137.50", description="Robot RT interface host IP"
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
            "rt_host": rt_host,
            "gui": gui,
        }.items(),
        condition=ag145_scenario,
    )

    # Research mode brings up RT state for either tool without pick-and-place,
    # gripper commands or jogging. Motion is started only from the collector GUI.
    rt_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("a0509_dynamics_experiment"), "launch", "real_rt.launch.py"
        ])),
        launch_arguments={
            "host": host, "rt_host": rt_host, "gripper_type": gripper_type,
            "mdpg_mount_xyz": mdpg_mount_xyz, "mdpg_mount_rpy": mdpg_mount_rpy,
        }.items(),
        condition=rt_state_only,
    )
    experiment_rviz = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="screen",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("sdml_a0509_description"), "rviz", "a0509_tool.rviz"
        ])],
        condition=IfCondition(PythonExpression([
            "'", gripper_type, "' == 'mdpg_1s1' and '", experiment_only,
            "' == 'false' and '", gui, "' == 'true'"
        ])),
    )
    front_real_collector = Node(
        package="sim_real_gap_getting", executable="front_real_gui",
        name="front_real_collector_gui", output="screen",
        arguments=["--tool", gripper_type, "--lock-tool",
                   "--mount-xyz", mdpg_mount_xyz, "--mount-rpy", mdpg_mount_rpy],
        condition=IfCondition(PythonExpression([
            "'", experiment_only, "' == 'true' and '", gui, "' == 'true'"
        ])),
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
        condition=ag145_scenario,
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
        condition=ag145_scenario,
    )

    return LaunchDescription([
        declare_host,
        declare_rt_host,
        DeclareLaunchArgument("gripper_type", default_value="ag145", description="ag145 | mdpg_1s1"),
        DeclareLaunchArgument("experiment_only", default_value="false",
                              description="true: RT state only, no pick-and-place or jogging nodes"),
        DeclareLaunchArgument("mdpg_mount_xyz", default_value="TODO_MEASURE"),
        DeclareLaunchArgument("mdpg_mount_rpy", default_value="TODO_MEASURE"),
        declare_gui,
        declare_jog_string_topic,
        declare_jog_multi_axis_topic,

        OpaqueFunction(function=_validate_tool),

        pick_and_place_launch,
        rt_state_launch,
        experiment_rviz,
        front_real_collector,
        jog_string_bridge,
        jogging_node,
    ])
