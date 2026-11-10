#
# a0509_mujoco_sim - MuJoCo simulation launch (dsr_mujoco style, no namespace)
#

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction, DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET


def _select_scene(context):
    if LaunchConfiguration('model').perform(context) != 'a0509':
        raise RuntimeError('a0509_mujoco_sim supports only model:=a0509')
    gripper = LaunchConfiguration('gripper_type').perform(context)
    if gripper not in ('none', 'ag145', 'mdpg_1s1'):
        raise RuntimeError(f'Unsupported gripper_type: {gripper}')
    requested = LaunchConfiguration('mujoco_model_path').perform(context)
    mount_xyz = LaunchConfiguration('mdpg_mount_xyz').perform(context)
    mount_rpy = LaunchConfiguration('mdpg_mount_rpy').perform(context)
    dynamics_model = LaunchConfiguration('mdpg_dynamics_model').perform(context)
    if dynamics_model not in ('visual', 'box_approx'):
        raise RuntimeError(f'Unsupported mdpg_dynamics_model: {dynamics_model}')
    if gripper != 'mdpg_1s1' and dynamics_model != 'visual':
        raise RuntimeError('mdpg_dynamics_model requires gripper_type:=mdpg_1s1')
    if gripper == 'mdpg_1s1' and ('TODO' in mount_xyz or 'TODO' in mount_rpy):
        raise RuntimeError('mDPG mounting transform is TODO; pass measured mdpg_mount_xyz and mdpg_mount_rpy')
    if requested != 'auto':
        root = ET.parse(requested).getroot()
        present = {name for name in ('ag145_tcp', 'mdpg_tcp')
                   if root.find(f".//site[@name='{name}']") is not None}
        expected = {'ag145_tcp'} if gripper == 'ag145' else (
            {'mdpg_tcp'} if gripper == 'mdpg_1s1' else set())
        if present != expected:
            raise RuntimeError(f'MuJoCo scene tool {present} differs from gripper_type {gripper}')
        if gripper == 'mdpg_1s1':
            has_inertia = root.find(".//body[@name='mdpg_base_link']/inertial") is not None
            if has_inertia != (dynamics_model == 'box_approx'):
                raise RuntimeError('mDPG scene inertia differs from mdpg_dynamics_model')
        return [SetLaunchConfiguration('resolved_mujoco_model_path', requested)]
    if gripper == 'none':
        from ament_index_python.packages import get_package_share_directory
        scene = Path(get_package_share_directory('dsr_description2')) / 'mujoco_models/a0509/scene.xml'
    else:
        folder = Path(tempfile.mkdtemp(prefix='a0509_tool_scene_'))
        scene = folder / 'scene.xml'
        if gripper == 'ag145':
            from a0509_dynamics_experiment.ag145_model import build_ag145_scene
            build_ag145_scene(scene)
        else:
            from mdpg_1s1_description.model import build_mdpg_scene
            build_mdpg_scene(scene, mount_xyz, mount_rpy, dynamics_model=dynamics_model)
    return [SetLaunchConfiguration('resolved_mujoco_model_path', str(scene))]

ARGUMENTS = [
    DeclareLaunchArgument('model', default_value='a0509', description='ROBOT_MODEL'),
    DeclareLaunchArgument('color', default_value='white', description='ROBOT_COLOR'),
    DeclareLaunchArgument('use_mujoco', default_value='true', description='Start Mujoco'),
    DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
    DeclareLaunchArgument('gripper_type', default_value='none', description='none | ag145 | mdpg_1s1'),
    DeclareLaunchArgument('mdpg_mount_xyz', default_value='TODO_MEASURE', description='Measured link_6 to mDPG xyz in metres'),
    DeclareLaunchArgument('mdpg_mount_rpy', default_value='TODO_MEASURE', description='Measured link_6 to mDPG rpy in radians'),
    DeclareLaunchArgument('mdpg_dynamics_model', default_value='visual', description='visual | box_approx (simulation estimate)'),
    DeclareLaunchArgument('mujoco_model_path', default_value='auto', description='Optional explicit MuJoCo scene XML'),
]


def generate_launch_description():
    # URDF via xacro from dsr_description2 (use_mujoco=true, namespace empty)
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution([
                FindPackageShare('sdml_a0509_description'),
                'urdf',
                'a0509_with_gripper.urdf.xacro',
            ]),
            ' ',
            'use_mujoco:=',
            LaunchConfiguration('use_mujoco'),
            ' ',
            'color:=',
            LaunchConfiguration('color'),
            ' ',
            'namespace:=',
            '',
            ' gripper_type:=', LaunchConfiguration('gripper_type'),
            " mdpg_mount_xyz:='", LaunchConfiguration('mdpg_mount_xyz'), "'",
            " mdpg_mount_rpy:='", LaunchConfiguration('mdpg_mount_rpy'), "'",
            ' mdpg_dynamics_model:=', LaunchConfiguration('mdpg_dynamics_model'),
        ]
    )
    robot_description = {'robot_description': robot_description_content}

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
            {'mujoco_model_path': LaunchConfiguration('resolved_mujoco_model_path')},
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
        OpaqueFunction(function=_select_scene),
        node_mujoco,
        delay_joint_state_broadcaster,
        delay_dsr_position_controller,
    ])
