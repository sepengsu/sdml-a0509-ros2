# a0509_mujoco_sim

Doosan A0509 로봇의 MuJoCo 시뮬레이션 패키지입니다. (namespace 미사용, dsr_mujoco 스타일)

## 의존성

- ROS 2 (Humble 권장)
- `dsr_description2` (Doosan 로봇 메쉬/URDF)
- `mujoco_ros2_control`
- `controller_manager`, `ros2_control`, `ros2_controllers`, `xacro`

## 빌드

워크스페이스 루트에서:

```bash
cd /path/to/sdml-doosan-robot
colcon build --packages-select a0509_mujoco_sim
source install/setup.bash
```

전체 워크스페이스 빌드:

```bash
colcon build
source install/setup.bash
```

## 시뮬레이션 실행

**기본 실행 (A0509, white, 시뮬 시간 사용):**

```bash
ros2 launch a0509_mujoco_sim mujoco_sim.launch.py
```

**선택 인자:**

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `model` | a0509 | 로봇 모델 (dsr_description2 xacro 이름) |
| `color` | white | 로봇 색상 (white / blue) |
| `use_mujoco` | true | MuJoCo 사용 여부 |
| `use_sim_time` | true | 시뮬레이션 시간 사용 여부 |

예: 파란색 로봇으로 실행

```bash
ros2 launch a0509_mujoco_sim mujoco_sim.launch.py color:=blue
```

## 실행 후 사용 가능한 토픽/인터페이스

- **Joint state:** `/joint_states` (sensor_msgs/JointState)
- **Position 제어:** `dsr_position_controller` (joint_1 ~ joint_6)
  - 명령: `/dsr_position_controller/joint_trajectory` 또는 해당 인터페이스

조인트 이름: `joint_1`, `joint_2`, `joint_3`, `joint_4`, `joint_5`, `joint_6`

## 제어 예시

position 명령으로 제어할 때 (예: CLI):

```bash
# joint state 확인
ros2 topic echo /joint_states

# controller 목록 확인
ros2 control list_controllers
```

다른 노드에서 `ForwardCommandController`에 position 명령을 보내면 MuJoCo 시뮬레이션이 해당 목표 위치로 움직입니다.

## 참고

- namespace를 사용하지 않으므로 모든 토픽/서비스는 루트에 있습니다.
- 씬 및 로봇 MJCF는 `dsr_description2`의 `mujoco_models/a0509/`를 사용합니다.

### Tool selection

`gripper_type:=none|ag145|mdpg_1s1` selects both the active A0509 Xacro in `sdml_a0509_description` and the MuJoCo scene. AG145 uses the existing experiment scene builder. mDPG uses the ZIP-derived CAD meshes attached as a massless visual frame on `link_6`; it has no modeled suction, contact force, mass or inertia. This scene is suitable for visualization only, not gripper dynamics data or PKAN training.

The mDPG mounting pose is unknown. Supply measured `mdpg_mount_xyz` (metres) and `mdpg_mount_rpy` (radians); leaving either as `TODO_MEASURE` causes launch to fail. `mdpg_tcp` is the uncalibrated CAD-bounds estimate. See `mdpg_1s1_description/config/dynamics_values_to_measure.yaml` for remaining physical and pneumatic IO TODOs.
