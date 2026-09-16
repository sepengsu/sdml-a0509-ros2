# sdml-doosan-robot

Doosan A0509 로봇 제어, Pick & Place 시나리오, AG145 gripper, Intel RealSense 카메라 연동을 위한 ROS 2 워크스페이스입니다.

---

## 구조 변경 요약

- 기존 `camera_description` 패키지는 제거하고, D435/D435i 카메라 description, mesh, RViz 설정을 `src/sdml_a0509/sdml_a0509_camera`로 통합했습니다.
- `dh_ag145_description`의 AG145 + Gripperbox + D435i 조립 매크로는 이제 `sdml_a0509_camera`의 카메라 URDF를 참조합니다.
- `sdml_a0509_bringup`의 virtual/real launch는 로봇 bringup만 담당합니다. 카메라는 별도 터미널에서 `ros2 run`으로 실행합니다.
- Virtual/Real 모두 최종 이미지 토픽은 `/a0509/camera/image_raw`를 기준으로 사용합니다.
- `dsr_visualservoing`은 Doosan 예제 성격으로 두고, 이 워크스페이스의 A0509 카메라 기능은 `sdml_a0509_camera`를 기준으로 관리합니다.

---

## 패키지 구조와 역할

### SDML A0509 패키지

| 패키지 | 경로 | 역할 |
|---|---|---|
| `sdml_a0509_bringup` | `src/sdml_a0509/sdml_a0509_bringup` | A0509 로봇 실행 진입점입니다. virtual/real bringup, Pick & Place, RViz, MuJoCo 실행을 조립합니다. 카메라는 별도 실행합니다. |
| `sdml_a0509_control` | `src/sdml_a0509/sdml_a0509_control` | 로봇 제어 노드입니다. Pick & Place 실행, jogging, joint state merge, Unity/ROS 문자열 jog 명령 변환을 담당합니다. |
| `sdml_a0509_description` | `src/sdml_a0509/sdml_a0509_description` | SDML용 A0509 로봇 description입니다. A0509 arm, AG145 gripper, Gripperbox, D435i 장착 구조를 하나의 URDF/xacro 트리로 구성합니다. |
| `sdml_a0509_camera` | `src/sdml_a0509/sdml_a0509_camera` | A0509 카메라 전용 패키지입니다. D435/D435i URDF와 mesh를 보관하고, `ros2 run`으로 켜고 끄는 virtual 테스트 이미지 publisher와 real 카메라 토픽 relay를 제공합니다. |
| `sdml_a0509_moveit_config` | `src/sdml_a0509/sdml_a0509_moveit_config` | SDML용 MoveIt 래퍼입니다. upstream `dsr_moveit_config_a0509` 실행을 우리 워크스페이스에서 쉽게 호출하기 위한 진입점입니다. |

### 보조 패키지

| 패키지 | 경로 | 역할 |
|---|---|---|
| `a0509_position_getter` | `src/a0509_position_getter` | 현재 posx/posj 확인, 저장/로드, homing, GUI 기반 디버그 조작을 제공합니다. |
| `a0509_mujoco_sim` | `src/a0509_mujoco_sim` | A0509 MuJoCo 시뮬레이션과 관련 launch/config를 제공합니다. |
| `server_bridge` | `src/server_bridge` | 외부 JSON/WebSocket 계층과 ROS 2 사이를 잇는 서버 브리지입니다. `rosbridge_server`와는 별도 역할입니다. |
| `dh_ag145_description` | `src/dh_gripper/dh_gripper_description/dh_ag145_description` | AG145 gripper, Gripperbox, 카메라 마운트 조립 xacro를 제공합니다. 카메라 본체 URDF/mesh는 `sdml_a0509_camera`를 참조합니다. |
| `dh_gripper_driver` | `src/dh_gripper/dh_gripper_driver` | AG95/AG145 gripper 제어용 ROS 2 driver, controller, launch, RViz 설정을 제공합니다. |
| `dh_gripper_modbus` | `src/dh_gripper_modbus` | Modbus 기반 gripper 제어 보조 패키지입니다. |
| `doosan-robot2` | `src/doosan-robot2` | Doosan upstream ROS 2 패키지 묶음입니다. DSR bringup/controller/description/MoveIt/messages/examples를 포함합니다. 가능한 한 직접 수정 범위를 작게 유지합니다. |

### 카메라 토픽 기준

| 모드 | 실행 주체 | 기본 토픽 | 설명 |
|---|---|---|---|
| Virtual | `sdml_a0509_camera/virtual_camera_publisher` | `/a0509/camera/image_raw` | RViz Image display 확인용 임시 이미지 토픽을 publish합니다. |
| Real | `realsense2_camera_node` + `sdml_a0509_camera/camera_topic_relay` | `/a0509/camera/image_raw` | Intel RealSense color image를 공용 A0509 카메라 토픽으로 relay합니다. `serial_no` 또는 `usb_port_id`로 장치를 지정할 수 있습니다. |

카메라 토픽은 로봇 launch와 분리되어 있습니다. `ros2 run` 프로세스를 켜면 토픽이 열리고, 해당 프로세스를 `Ctrl+C`로 종료하면 토픽도 닫힙니다.

---

## 빌드

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

카메라 description 통합 이후 관련 패키지만 다시 빌드하려면:

```bash
/scripts/ros2_colcon_docker.sh --clean-pkgs sdml_a0509_camera dh_ag145_description sdml_a0509_description sdml_a0509_bringup
source /ros2_ws/install/setup.bash
```

---

## 실행

### Virtual

```bash
ros2 launch sdml_a0509_bringup a0509_virtual.launch.py host:=robot-emulator rt_host:=robot-emulator
```

- 이 명령은 로봇/제어 노드만 실행합니다.
- 카메라는 아래 `카메라 실행` 절에서 별도 터미널로 실행합니다.

### Real

```bash
ros2 launch sdml_a0509_bringup a0509_real.launch.py host:=192.168.137.100
```

이 명령은 로봇/제어 노드만 실행합니다. RealSense 카메라는 아래 `카메라 실행` 절에서 별도 터미널로 실행합니다.

### 카메라 실행

Virtual 카메라:

```bash
ros2 run sdml_a0509_camera virtual_camera_publisher
```

RealSense 카메라는 터미널 2개로 실행합니다. 첫 번째 터미널에서 RealSense driver를 실행합니다.

```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/a0509 \
  -r __node:=camera \
  -p enable_color:=true \
  -p enable_depth:=true \
  -p align_depth.enable:=true \
  -p pointcloud.enable:=false
```

두 번째 터미널에서 공용 토픽 relay를 실행합니다.

```bash
ros2 run sdml_a0509_camera camera_topic_relay
```

특정 RealSense 장치를 지정해야 하면 driver 실행 명령에 파라미터를 추가합니다.

```bash
-p serial_no:=123456789
-p usb_port_id:=2-1.4
```

Virtual/Real 모두 최종 소비 토픽은 `/a0509/camera/image_raw`입니다.

### RViz 시각화 전용

```bash
# joint_state_publisher_gui + robot_state_publisher + rviz2
ros2 launch sdml_a0509_bringup rviz_only.launch.py

# GUI 슬라이더 없이 실행
ros2 launch sdml_a0509_bringup rviz_only.launch.py use_gui:=false
```

### MuJoCo

```bash
# MuJoCo + RViz
ros2 launch sdml_a0509_bringup mujoco.launch.py

# MuJoCo만 실행
ros2 launch sdml_a0509_bringup mujoco.launch.py start_rviz:=false
```

### Position Getter + Jogging

```bash
ros2 launch a0509_position_getter getter_and_jogging.launch.py
```

- 내부 jogging 노드는 `sdml_a0509_control/jogging`를 사용합니다.

---

## Pick & Place 시나리오

`/do_pick_and_place` 토픽에 `std_msgs/String`으로 `go`를 보내면 시나리오가 실행됩니다. GUI **Do Scenario** 버튼으로도 트리거할 수 있습니다.

```bash
ros2 topic pub --once /do_pick_and_place std_msgs/msg/String "{data: 'go'}"
```

### 시나리오 타입

| 타입 | 기본 | 사용 JSON | 설명 |
|---|---|---|---|
| `pick_place` | yes | `pick_place_config.json` + `pick_place_times.json` | 집는 위치에서 놓는 위치까지 1사이클을 실행합니다. posj/posx, z_up, gripper, 단계별 시간을 사용합니다. |
| `list` | no | `do_scenario.json` | posx/posj/movec/gripper 스텝 배열을 순차 실행합니다. 형식은 `json/HowToMake.md`를 참고합니다. |

```bash
# 기본: pick_place 시나리오
ros2 launch sdml_a0509_bringup pick_and_place.launch.py mode:=virtual

# 리스트형 시나리오
ros2 launch sdml_a0509_bringup pick_and_place.launch.py mode:=virtual scenario_type:=list
```

---

## JSON 저장 경로

포지션과 시나리오 JSON은 `~/.ros`가 아니라 워크스페이스의 `json` 폴더에 저장/로드됩니다.

- 기본 경로: 워크스페이스 루트에서 실행 시 `./json/`
- 직접 지정: `export JSON_DIR=/원하는/경로`

| 파일 | 용도 |
|---|---|
| `json/saved_positions.json` | Position Getter GUI 저장 목록입니다. posx/posj/gripper 값을 저장합니다. |
| `json/pick_place_config.json` | Pick & Place 위치, z_up, gripper open/close 값을 저장합니다. |
| `json/pick_place_times.json` | Pick & Place 접근/하강/이탈 등 각 동작별 시간을 저장합니다. |
| `json/do_scenario.json` | 리스트형 시나리오 스텝 배열입니다. `scenario_type:=list`에서 사용합니다. |

---

## 개별 노드 실행

```bash
ros2 run sdml_a0509_control jogging
ros2 run sdml_a0509_control pick_and_place_node
ros2 run sdml_a0509_control merge_node
```

---

## 원격/브리지 연동

- SSH: `ssh sdml@115.145.167.47` 후 `source ~/.bashrc`
- SSH 내부 rosbridge: `roslaunch rosbridge_server rosbridge_websocket.launch`
- 로컬 JSON bridge: `ros2 run server_bridge server_bridge_node`
- Docker rosbridge WebSocket: `ws://localhost:9090`
- `server_bridge` uplink 토픽: `/joint_states`, `/a0509/joint_states`, `/ag145_gripper/joint_states`, `/ag145/joint_states`, `/a0509/camera/image_raw`

---

## 기타 참고 실행

```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=real model:=a0509 host:=192.168.137.100
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=a0509 host:=127.0.0.1
ros2 launch dh_gripper_driver gripper_control.launch.py
ros2 launch dh_gripper_driver gripper_show.launch.py
```
