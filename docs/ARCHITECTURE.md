# 시스템 아키텍처

Docker 기반 두산 로봇 제어 환경의 구조입니다.

---

## 전체 구조

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Host (Windows/Linux/macOS)                    │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────────────────┐     ┌──────────────────────────────────┐  │
│  │   Doosan Emulator    │     │      ros-control Container       │  │
│  │  (Virtual Mode Only) │     │                                  │  │
│  │                      │     │  ┌────────────────────────────┐  │  │
│  │ doosanrobot/         │     │  │  ROS 2 Control Stack       │  │  │
│  │ dsr_emulator:3.0.1   │────►│  │  - ros2_control            │  │  │
│  │                      │     │  │  - MoveIt 2                │  │  │
│  │ alias: robot-emulator│     │  │  - Gazebo                  │  │  │
│  └──────────────────────┘     │  └────────────────────────────┘  │  │
│           │                   │                                  │  │
│           │                   │  ┌────────────────────────────┐  │  │
│           │                   │  │  VNC Stack                 │  │  │
│           │                   │  │  - Xvfb (:99)              │  │  │
│           │                   │  │  - x11vnc (:5900)          │  │  │
│           │                   │  │  - noVNC (:6080)           │  │  │
│           │                   │  │  - Fluxbox (WM)            │  │  │
│           │                   │  └────────────────────────────┘  │  │
│           │                   │                                  │  │
│           └───────────────────┼──── Bridge Network (ros_net) ────┤  │
│                               └──────────────────────────────────┘  │
│                                            │                         │
│                                     localhost:6080                   │
│                                            ▼                         │
│                                    ┌───────────────┐                │
│                                    │   Browser     │                │
│                                    │  (noVNC GUI)  │                │
│                                    └───────────────┘                │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 컨테이너 구성

### Doosan Emulator (`emulator`)

| 항목 | 값 |
|------|-----|
| 이미지 | `doosanrobot/dsr_emulator:3.0.1` |
| 프로파일 | `virtual` |
| 포트 | `12345` |
| 용도 | 하드웨어 없이 로봇 시뮬레이션 |

### ROS 2 Control Stack (`ros-control`)

| 항목 | 값 |
|------|-----|
| Dockerfile | `docker/Dockerfile.ros` |
| 베이스 이미지 | `osrf/ros:humble-desktop` |
| 네트워크 | Bridge (`ros_net`) |
| 마운트 | `./src:/ros2_ws/src` |

**주요 패키지:**
- `dsr_bringup2` - 런치 파일
- `dsr_hardware2` - 하드웨어 인터페이스
- `dsr_controller2` - 로봇 컨트롤러
- `dsr_moveit_config_a0509` - MoveIt 설정
- `a0509_control_cyw` - 커스텀 제어 노드

---

## 그리퍼 연결 모드

### Mock 모드 (Virtual)

```
GripperController ──→ [Mock] ──→ 로그 출력만
```

- 하드웨어 연결 없음
- `[MOCK]` 접두사로 로그 출력
- 항상 성공 반환

### Direct 모드 (Linux Real)

```
GripperController ──→ Modbus RTU ──→ /dev/ttyACM0 ──→ 그리퍼
```

- USB 장치 직접 전달
- 115200 baud, 8N1
- 최저 지연시간 (~0ms)

### External 모드 (Windows/macOS Real)

```
┌─────────────┐     TCP      ┌─────────────┐    USB     ┌─────────┐
│ Container   │◄───:9999────►│ Gripper     │◄──────────►│ Gripper │
│ (Client)    │              │ Server      │            │ (H/W)   │
└─────────────┘              └─────────────┘            └─────────┘
```

- 호스트에서 그리퍼 서버 실행
- TCP 소켓 통신
- 지연시간 ~0.1-0.3ms

---

## Docker Compose 프로파일

### `virtual` 프로파일

```yaml
services:
  emulator:
    profiles: ["virtual"]
  ros-control:
    profiles: ["virtual"]
```

시작: `dsr2 virtual` 또는 `docker compose --profile virtual up`

### `real` 프로파일

```yaml
services:
  ros-control-real:
    profiles: ["real"]
    devices:
      - /dev/ttyACM0:/dev/ttyACM0
```

시작: `dsr2 real` (Linux)

### `real-external` 프로파일

```yaml
services:
  ros-control-real-external:
    profiles: ["real-external"]
    environment:
      - GRIPPER_EXTERNAL=true
```

시작: `dsr2 real` (Windows/macOS)

---

## 네트워크 구성

모든 컨테이너는 Bridge 네트워크 (`ros_net`)를 사용합니다.

**변경 이유 (host → bridge):**
- 크로스 플랫폼 지원 (Windows/macOS에서 host 모드 미지원)
- 컨테이너 격리 및 보안 향상
- 컨테이너 간 DNS 자동 해석 (`robot-emulator` 호스트명)

**컨테이너 간 통신:**
- Emulator: `robot-emulator:12345`
- Host 접근: `host.docker.internal`

**ROS_DOMAIN_ID:**
- 기본값: `29`
- 동일 네트워크의 다른 ROS 시스템과 분리

---

## 볼륨 마운트

```yaml
volumes:
  - ./src:/ros2_ws/src:rw       # 소스 코드
  - ./scripts:/scripts:rw       # 테스트 스크립트
```

**참고:** X11 소켓 마운트는 더 이상 필요 없음 (VNC 사용)

**주의:** 소스만 마운트되므로 컨테이너 내에서 `colcon build` 필요

---

## ROS 2 토픽 구조

```
/joint_states                    # 조인트 상태 (100Hz)
/joint_trajectory_controller/    # 궤적 명령
/robot_description              # URDF
/tf, /tf_static                 # 좌표 변환
/move_group/                    # MoveIt 인터페이스
```

---

## 파일 구조

```
sdml-doosan-robot/
├── docker/
│   ├── Dockerfile.ros          # ROS 컨테이너 이미지
│   ├── entrypoint.sh           # 컨테이너 시작 스크립트
│   └── start-vnc.sh            # VNC 서비스 시작 스크립트
├── src/
│   ├── doosan-robot2/          # 두산 공식 패키지
│   ├── a0509_control_cyw/      # 커스텀 제어
│   └── dh_gripper_modbus/      # 그리퍼 드라이버
├── dsr2/
│   ├── cli.py                  # CLI 메인
│   ├── gripper_server.py       # TCP 그리퍼 서버
│   ├── gripper_client.py       # TCP 클라이언트
│   └── gripper_protocol.py     # 통신 프로토콜
├── docs/                       # 문서
├── docker-compose.yml          # 컨테이너 구성
├── .env.example                # 환경 변수 템플릿
└── pyproject.toml              # Python 패키지 설정
```
