from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_gripper_control(context):
    """mode에 따라 mock / real 인터페이스 선택. mode:=virtual -> mock, mode:=real -> real."""
    mode = LaunchConfiguration("mode").perform(context)
    com_port = LaunchConfiguration("com_port").perform(context)
    prefix = LaunchConfiguration("prefix").perform(context)
    gripper_ns = LaunchConfiguration("gripper_ns").perform(context)
    expanded_joint_states_topic = LaunchConfiguration("expanded_joint_states_topic").perform(context)
    model = LaunchConfiguration("gripper_model").perform(context).strip().lower()
    if not model:
        model = LaunchConfiguration("model").perform(context).strip().lower()

    if model not in ("ag95", "ag145"):
        raise ValueError(f"Unsupported gripper model: {model}")

    use_mock = mode.strip().lower() == "virtual"
    use_mock_str = "true" if use_mock else "false"

    description_package = f"dh_{model}_description"
    description_file = f"dh_{model}.urdf.xacro"
    controllers_file = f"dh_{model}_controllers.yaml"

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "com_port:=", TextSubstitution(text=com_port),
            " ",
            "prefix:=", TextSubstitution(text=prefix),
            " ",
            "use_mock_hardware:=", TextSubstitution(text=use_mock_str),
            " ",
            "mock_sensor_commands:=", TextSubstitution(text=use_mock_str),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("dh_gripper_driver"), "config", controllers_file]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=gripper_ns,
        parameters=[robot_description, robot_controllers],
        arguments=[
            "--ros-args",
            "--log-level",
            f"{gripper_ns}.controller_manager:=error",
        ],
        output="both",
        remappings=[("controller_manager/robot_description", "robot_description")],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=gripper_ns,
        output="both",
        parameters=[robot_description],
    )

    controller_manager_uri = "/" + gripper_ns + "/controller_manager"
    mimic_expander_node = Node(
        package="dh_gripper_driver",
        executable="joint_state_mimic_expander",
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "input_topic": "/" + gripper_ns + "/joint_states",
            "output_topic": expanded_joint_states_topic,
            "publish_only_mimics": True,
            "keep_incoming_order": True,
        }],
    )

    def controller_spawner(name, *args):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                name,
                "--controller-manager",
                controller_manager_uri,
            ] + list(args),
        )

    active_spawners = [
        controller_spawner("joint_state_broadcaster"),
        controller_spawner("forward_position_controller"),
    ]

    return [control_node, robot_state_pub_node] + active_spawners + [mimic_expander_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("mode", default_value="real", description="real: 실제 그리퍼, virtual: mock 인터페이스"),
        DeclareLaunchArgument("gripper_model", default_value="ag145", description="Gripper model: ag95 or ag145"),
        DeclareLaunchArgument("model", default_value="", description="(deprecated) use gripper_model"),
        DeclareLaunchArgument("com_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("prefix", default_value='""'),
        DeclareLaunchArgument("gripper_ns", default_value="ag145"),
        DeclareLaunchArgument("expanded_joint_states_topic", default_value="/ag145_gripper/joint_states"),
    ]

    return LaunchDescription(
        declared_arguments
        + [OpaqueFunction(function=_launch_gripper_control)]
    )
