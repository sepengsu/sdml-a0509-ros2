# 빠른 시작 가이드

`dsr2` CLI를 사용한 로봇 제어 환경 실행 방법입니다.

---

## Virtual 모드 (시뮬레이션)

하드웨어 없이 에뮬레이터로 테스트합니다.

### 방법 1: 자동 실행 (권장)

`.env` 파일에서 `AUTO_LAUNCH=true`로 설정하면 컨테이너 시작 시 ROS2가 자동 실행됩니다.

```bash
# .env 수정
AUTO_LAUNCH=true

# 컨테이너 시작 (ROS2 자동 실행)
dsr2 virtual
```

VNC(`http://localhost:6080`)에서 RViz가 바로 표시됩니다.

### 방법 2: 수동 실행

```bash
# 컨테이너 시작
dsr2 virtual

# 백그라운드 실행
dsr2 virtual -d
```

기본 `dsr2 virtual`은 호스트 `Gripper Server`를 시작하지 않습니다. TCP 기반 그리퍼 클라이언트까지 같이 테스트하려면 `--gripper-port`를 지정합니다.

```bash
# Mock Gripper Server 자동 시작
dsr2 virtual --gripper-port mock

# 지정 포트의 Gripper Server 자동 시작
dsr2 virtual --gripper-port COM3
dsr2 virtual --gripper-port /dev/ttyACM0
```

#### 컨테이너 내부에서 ROS2 실행

```bash
# 컨테이너 접속
docker exec -it ros-control bash

# 패키지 빌드 (최초 1회)
colcon build
source install/setup.bash

# ROS2 launch 실행 (환경변수 자동 적용)
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py host:=$ROBOT_HOST model:=$ROBOT_MODEL

# 또는 MoveIt 실행
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py host:=$ROBOT_HOST model:=$ROBOT_MODEL
```

### GUI 접속 (noVNC)

브라우저에서 GUI에 접속합니다:

```
http://localhost:6080
```

- **Connect** 클릭 → Fluxbox 데스크톱 표시
- 터미널에서 `rviz2 &` 또는 `rqt &` 실행

### 이동 기능 테스트

ROS2 launch 실행 후 로봇 모션 서비스를 테스트합니다:

```bash
# 컨테이너 접속
docker exec -it ros-control bash

# ROS2 launch 실행 (RViz 포함)
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py host:=$ROBOT_HOST model:=$ROBOT_MODEL

# 다른 터미널에서 테스트 (또는 같은 컨테이너에서 & 사용)
```

**현재 위치 조회:**
```bash
# 관절 위치 (Joint space)
ros2 service call /dsr01/aux_control/get_current_posj dsr_msgs2/srv/GetCurrentPosj

# 작업 위치 (Task space)
ros2 service call /dsr01/aux_control/get_current_posx dsr_msgs2/srv/GetCurrentPosx
```

**관절 이동 (move_joint):**
```bash
ros2 service call /dsr01/motion/move_joint dsr_msgs2/srv/MoveJoint \
  '{pos: [0.0, 0.0, 90.0, 0.0, 90.0, 0.0], vel: 30.0, acc: 30.0}'
```

**직선 이동 (move_line):**
```bash
ros2 service call /dsr01/motion/move_line dsr_msgs2/srv/MoveLine \
  '{pos: [400.0, 0.0, 500.0, 45.0, 180.0, 45.0], vel: [50.0, 10.0], acc: [100.0, 20.0]}'
```

**홈 위치로 이동:**
```bash
ros2 service call /dsr01/motion/move_home dsr_msgs2/srv/MoveHome
```

**사용 가능한 모션 서비스 목록:**
```bash
ros2 service list | grep motion
```

### 종료

```bash
docker compose --profile virtual down
```

`dsr2 virtual -d --gripper-port ...`로 host `Gripper Server`를 같이 띄웠다면 `dsr2 gripper --stop`으로 정리합니다. VS Code의 `Docker: Stop ...` 태스크는 이 cleanup까지 같이 수행합니다.

---

## Real 모드 (실제 하드웨어)

### Linux (직접 USB)

```bash
dsr2 real --ip 192.168.137.100
```

### Windows / macOS (TCP 서버)

그리퍼 서버가 자동으로 시작됩니다.

```bash
# 기본 (포트 자동 감지)
dsr2 real --ip 192.168.137.100

# 특정 포트 지정
dsr2 real --gripper-port COM3 --ip 192.168.137.100

# Mock 모드 (하드웨어 없이)
dsr2 real --gripper-port mock --ip 192.168.137.100
```

### 수동으로 그리퍼 서버 실행

```bash
# 터미널 1: 그리퍼 서버
dsr2 gripper --gripper-port COM3

# 터미널 2: 컨테이너
dsr2 real --with-gripper false --ip 192.168.137.100
```

### 종료

```bash
# Ctrl+C로 종료하거나:
docker compose --profile real down
docker compose --profile real-external down
```

`dsr2 real -d`가 host `Gripper Server`를 자동 시작한 경우에는 `dsr2 gripper --stop`도 함께 실행해야 합니다. VS Code의 `Docker: Stop ...` 태스크는 이 cleanup까지 같이 수행합니다.

---

## CLI 명령어 레퍼런스

| 명령어 | 설명 |
|--------|------|
| `dsr2 virtual` | 에뮬레이터 모드 시작 |
| `dsr2 virtual -d` | 백그라운드 실행 |
| `dsr2 virtual --no-build` | Docker 빌드 건너뛰기 |
| `dsr2 virtual --gripper-port mock` | Virtual + Mock 그리퍼 서버 |
| `dsr2 virtual --gripper-port <PORT>` | Virtual + 지정 포트 그리퍼 서버 |
| `dsr2 real` | 실제 로봇 연결 (OS 자동 감지) |
| `dsr2 real --ip <IP>` | 로봇 IP 지정 |
| `dsr2 real --gripper-port COM3` | 그리퍼 포트 지정 |
| `dsr2 real --gripper-port mock` | Mock 그리퍼 사용 |
| `dsr2 real --external false` | 직접 USB 모드 강제 (Linux) |
| `dsr2 real --with-gripper false` | 그리퍼 서버 자동 시작 비활성화 |
| `dsr2 gripper` | 그리퍼 서버만 시작 (자동 감지) |
| `dsr2 gripper --gripper-port <PORT>` | 특정 포트로 그리퍼 서버 |
| `dsr2 gripper --stop` | managed background gripper server 종료 |
| `dsr2 --help` | 도움말 및 현재 설정 표시 |

---

## VS Code Tasks

VS Code에서는 `dsr2: Virtual`, `dsr2: Real`, `dsr2: Gripper` 세 개의 태스크만 제공합니다.

`Docker: Stop ...` 태스크는 관련 컨테이너를 내린 뒤 managed host `Gripper Server`까지 같이 정리합니다.

- 추가 옵션은 실행 시 표시되는 모달에서 입력합니다.
- `default`를 입력하면 CLI 기본값을 그대로 사용합니다.
- `dsr2: Virtual`의 `gripper option`에 `mock`, `auto`, `COM3`, `/dev/ttyACM0` 등을 입력하면 해당 `Gripper Server`를 함께 시작합니다.

---

## 컨테이너 내부 명령어

```bash
# 컨테이너 접속
docker exec -it ros-control bash

# ROS2 토픽 확인
ros2 topic list

# Joint states 확인
ros2 topic echo /joint_states

# 발행 주기 확인
ros2 topic hz /joint_states
```

---

## 다음 단계

- [XSERVER.md](./XSERVER.md) - GUI 접속 가이드 (noVNC)
- [GRIPPER.md](./GRIPPER.md) - 그리퍼 상세 설정
- [TROUBLESHOOTING.md](./TROUBLESHOOTING.md) - 문제 해결
