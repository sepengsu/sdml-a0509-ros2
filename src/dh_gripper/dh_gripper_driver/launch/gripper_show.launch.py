from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_gripper_show(context):
    model = LaunchConfiguration("model").perform(context).strip().lower()
    com_port = LaunchConfiguration("com_port").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)
    use_mock_hardware = LaunchConfiguration("use_mock_hardware").perform(context)
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands").perform(context)
    rviz_config_arg = LaunchConfiguration("rviz_config").perform(context).strip()
    gui = LaunchConfiguration("gui")

    if model not in ("ag95", "ag145"):
        raise ValueError(f"Unsupported gripper model: {model}")

    description_package = f"dh_{model}_description"
    description_file = f"dh_{model}.urdf.xacro"
    gripper_ns_default = f"/{model}/joint_states"
    rviz_config = (
        rviz_config_arg
        if rviz_config_arg
        else str(PathJoinSubstitution(
            [FindPackageShare("dh_gripper_driver"), "rviz", f"dh_{model}_control.rviz"]
        ).perform(context))
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "com_port:=",
            TextSubstitution(text=com_port),
            " ",
            "prefix:=",
            TextSubstitution(text=prefix),
            " ",
            "use_mock_hardware:=",
            TextSubstitution(text=use_mock_hardware),
            " ",
            "mock_sensor_commands:=",
            TextSubstitution(text=mock_sensor_commands),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        remappings=[
            ("/joint_states", gripper_ns_default),
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(gui),
    )

    return [
        robot_state_pub_node,
        rviz_node,
    ]


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "model",
            default_value="ag95",
            description="Gripper model: ag95 or ag145.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "com_port",
            default_value="/dev/ttyACM0",
            description="COM port of the gripper (for xacro args only).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of joint names (for xacro args only).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Passed to xacro (visualization only).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Passed to xacro (visualization only).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="",
            description="RViz config file path. Empty uses the model-specific default.",
        )
    )

    return LaunchDescription(
        declared_arguments
        + [OpaqueFunction(function=_launch_gripper_show)]
    )
