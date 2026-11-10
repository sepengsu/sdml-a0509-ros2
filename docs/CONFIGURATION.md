# 환경 설정 가이드

`.env` 파일을 통한 환경 변수 설정 방법입니다.

---

## 기본 설정

프로젝트 루트에 `.env` 파일이 없으면 `.env.example`에서 자동 복사됩니다.

```bash
cp .env.example .env
```

---

## 환경 변수 레퍼런스

### 로봇 설정

| 변수 | 기본값 | 설명 |
|------|--------|------|
| `ROBOT_MODEL` | `A0509` | 로봇 모델 |
| `ROBOT_COLOR` | `white` | 로봇 색상 |
| `ROBOT_IP` | `192.168.137.100` | 실제 로봇 IP |
| `ROBOT_RT_IP` | `192.168.137.100` | 실시간 제어 IP |
| `ROBOT_PORT` | `12345` | 로봇 제어 포트 |

### 그리퍼 설정

| 변수 | 기본값 | 설명 |
|------|--------|------|
| `GRIPPER_PORT` | `/dev/ttyACM0` | 시리얼 포트 (Linux) |
| `GRIPPER_BAUDRATE` | `115200` | 시리얼 통신 속도 |
| `GRIPPER_EXTERNAL` | `false` | TCP 서버 모드 사용 여부 |
| `GRIPPER_HOST` | `host.docker.internal` | TCP 서버 호스트 |
| `GRIPPER_PORT_TCP` | `9999` | TCP 서버 포트 |

> `ROBOT_MODE`와 `GRIPPER_MOCK`은 CLI가 자동 설정합니다.

### 디스플레이 설정

| 변수 | 기본값 | 설명 |
|------|--------|------|
| `DISPLAY` | `:0` | X11 디스플레이 |

Windows에서 VcXsrv 사용 시:
```bash
DISPLAY=host.docker.internal:0.0
```

### ROS 설정

| 변수 | 기본값 | 설명 |
|------|--------|------|
| `ROS_DOMAIN_ID` | `29` | ROS 2 DDS 도메인 ID (0-232) |

### GPU 설정

| 변수 | 기본값 | 설명 |
|------|--------|------|
| `ENABLE_GPU` | `false` | NVIDIA GPU 지원 |

---

## 플랫폼별 예제

### Linux (직접 USB)

```bash
ROBOT_MODEL=A0509
ROBOT_IP=192.168.137.100
ROBOT_RT_IP=192.168.137.100
ROBOT_PORT=12345

GRIPPER_PORT=/dev/ttyACM0
GRIPPER_BAUDRATE=115200
GRIPPER_EXTERNAL=false

ROS_DOMAIN_ID=29
DISPLAY=:0
```

### Windows / macOS (TCP 서버)

```bash
ROBOT_MODEL=A0509
ROBOT_IP=192.168.137.100
ROBOT_RT_IP=192.168.137.100
ROBOT_PORT=12345

GRIPPER_EXTERNAL=true
GRIPPER_HOST=host.docker.internal
GRIPPER_PORT_TCP=9999

ROS_DOMAIN_ID=29
DISPLAY=host.docker.internal:0.0
```

---

## Docker Compose 프로파일

| 프로파일 | 명령어 | 설명 |
|---------|--------|------|
| `virtual` | `dsr2 virtual` | 에뮬레이터 + ROS 컨트롤 |
| `real` | `dsr2 real` (Linux) | 실제 로봇 + 직접 USB |
| `real-external` | `dsr2 real` (Win/Mac) | 실제 로봇 + TCP 그리퍼 |

수동 실행:
```bash
docker compose --profile virtual up
docker compose --profile real up
docker compose --profile real-external up
```

---

## CLI 옵션 vs 환경 변수

CLI 옵션이 `.env` 설정보다 우선합니다.

```bash
# .env의 ROBOT_IP를 무시하고 지정된 IP 사용
dsr2 real --ip 192.168.1.100
```

---

## 현재 설정 확인

```bash
dsr2 --help
```

출력 예시:
```
Current config (from .env):
    ROBOT_IP:      192.168.137.100
    ROBOT_MODEL:   A0509
    GRIPPER_PORT:  /dev/ttyACM0
    ROS_DOMAIN_ID: 29
```
