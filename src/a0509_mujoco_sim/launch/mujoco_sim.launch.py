#
# a0509_mujoco_sim - MuJoCo simulation launch (dsr_mujoco style, no namespace)
#

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='a0509', description='ROBOT_MODEL'),
    DeclareLaunchArgument('color', default_value='white', description='ROBOT_COLOR'),
    DeclareLaunchArgument('use_mujoco', default_value='true', description='Start Mujoco'),
    DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
]


def generate_launch_description():
    # URDF via xacro from dsr_description2 (use_mujoco=true, namespace empty)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('dsr_description2'),
                'xacro',
                LaunchConfiguration('model'),
            ]),
            '.urdf.xacro',
            ' ',
            'use_mujoco:=',
            LaunchConfiguration('use_mujoco'),
            ' ',
            'color:=',
            LaunchConfiguration('color'),
            ' ',
            'namespace:=',
            '',
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    mujoco_model_path = PathJoinSubstitution([
        FindPackageShare('dsr_description2'), 'mujoco_models', 'a0509', 'scene.xml'
    ])
    controller_param_file = PathJoinSubstitution([
        FindPackageShare('a0509_mujoco_sim'), 'config', 'a0509_mujoco_controller.yaml'
    ])

    # MuJoCo node (no namespace)
    node_mujoco = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_param_file,
            {'mujoco_model_path': mujoco_model_path},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # Joint state broadcaster (no namespace)
    # The pinned mujoco_ros2_control creates /controller_manager.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '-c', '/controller_manager',
        ],
        output='screen',
    )

    # Position controller (no namespace)
    dsr_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'dsr_position_controller',
            '-c', '/controller_manager',
        ],
        output='screen',
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_mujoco,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[joint_state_broadcaster_spawner]
                )
            ]
        )
    )

    delay_dsr_position_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_mujoco,
            on_start=[
                TimerAction(
                    period=1.0,
                    actions=[dsr_position_controller_spawner]
                )
            ]
        )
    )

    return LaunchDescription(ARGUMENTS + [
        node_mujoco,
        delay_joint_state_broadcaster,
        delay_dsr_position_controller,
    ])
