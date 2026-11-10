# 그리퍼 사용 가이드

DH-Robotics 그리퍼를 다양한 플랫폼에서 사용하는 방법입니다.

---

## 연결 방식 개요

| 플랫폼 | 연결 방식 | 설명 |
|--------|----------|------|
| **Linux** | 직접 USB | USB 장치를 컨테이너에 직접 전달 |
| **Windows** | TCP 서버 | 호스트에서 그리퍼 서버 실행 |
| **macOS** | TCP 서버 | 호스트에서 그리퍼 서버 실행 |

> CLI가 OS를 감지하여 자동으로 적절한 모드를 선택합니다.

---

## Virtual 모드 (Mock 그리퍼)

모든 플랫폼에서 동일합니다. 그리퍼는 자동으로 Mock 모드로 동작합니다.

```bash
dsr2 virtual
```

기본 `dsr2 virtual`은 컨테이너 내부에서 Mock 그리퍼 모드만 활성화하며, 호스트 `Gripper Server`는 자동 시작하지 않습니다.

TCP 클라이언트 경로까지 함께 테스트하려면 `--gripper-port`를 지정합니다.

```bash
# Virtual + Mock Gripper Server
dsr2 virtual --gripper-port mock

# Virtual + 실제 포트의 Gripper Server
dsr2 virtual --gripper-port COM3          # Windows
dsr2 virtual --gripper-port /dev/ttyACM0  # Linux
dsr2 virtual --gripper-port auto          # 자동 감지
```

---

## Real 모드 - Linux

USB 장치를 Docker 컨테이너에 직접 전달합니다.

### 사전 확인

```bash
# 그리퍼 USB 연결 확인
ls /dev/ttyACM*
```

### 실행

```bash
dsr2 real --ip 192.168.137.100
```

### 다른 USB 포트 사용

```bash
dsr2 real --gripper-port /dev/ttyUSB0 --ip 192.168.137.100
```

---

## Real 모드 - Windows / macOS

USB를 Docker에 직접 전달할 수 없으므로, 호스트에서 **그리퍼 TCP 서버**를 실행합니다.

### 아키텍처

```
┌─────────────────────────────────────────────────┐
│              호스트 (Windows/macOS)              │
│                                                 │
│  ┌─────────────────┐    ┌───────────────────┐  │
│  │ dsr2-gripper    │    │ Docker Container  │  │
│  │                 │    │                   │  │
│  │ USB: COM3       │◄──►│ ROS 2 + MoveIt    │  │
│  │ TCP: :9999      │    │ (TCP 클라이언트)   │  │
│  └─────────────────┘    └───────────────────┘  │
└─────────────────────────────────────────────────┘
```

### 방법 A: 자동 실행 (권장)

그리퍼 서버가 자동으로 시작됩니다.

```bash
# 포트 자동 감지
dsr2 real --ip 192.168.137.100

# 특정 포트 지정
dsr2 real --gripper-port COM3 --ip 192.168.137.100

# Mock 모드 (하드웨어 없이)
dsr2 real --gripper-port mock --ip 192.168.137.100
```

### 방법 B: 수동 실행

**터미널 1 - 그리퍼 서버:**
```bash
dsr2 gripper --gripper-port COM3     # Windows
dsr2 gripper --gripper-port auto     # 자동 감지
dsr2 gripper --gripper-port mock     # Mock 모드
```

**터미널 2 - 컨테이너:**
```bash
dsr2 real --with-gripper false --ip 192.168.137.100
```

### 종료

**방법 A:** `Ctrl+C` 한 번으로 모두 종료

**방법 B:**
1. 컨테이너: `Ctrl+C` 또는 `docker compose --profile real-external down`
2. 그리퍼 서버: `Ctrl+C`

`dsr2 real -d` 또는 `dsr2 virtual -d --gripper-port ...`가 host `Gripper Server`를 자동 시작한 경우에는 `dsr2 gripper --stop`으로 managed background server를 정리합니다.

---

## 그리퍼 포트 찾기

### Windows

1. 장치 관리자 열기 (`devmgmt.msc`)
2. "포트(COM & LPT)" 섹션 확인
3. DH-Robotics 또는 USB Serial Device의 COM 번호 확인

### macOS

```bash
ls /dev/tty.usb*
# 예: /dev/tty.usbmodem14101
```

### Linux

```bash
ls /dev/ttyACM* /dev/ttyUSB*
# 예: /dev/ttyACM0
```

---

## 명령어 요약

| 명령어 | 설명 |
|--------|------|
| `dsr2 virtual` | Virtual + 내부 Mock 그리퍼만 사용 |
| `dsr2 virtual --gripper-port mock` | Virtual + Mock 그리퍼 서버 |
| `dsr2 virtual --gripper-port <PORT>` | Virtual + 지정 포트 그리퍼 서버 |
| `dsr2 real` | 실제 로봇 연결 (OS 자동 감지) |
| `dsr2 real --gripper-port COM3` | 특정 포트로 그리퍼 |
| `dsr2 real --gripper-port mock` | Mock 그리퍼 |
| `dsr2 real --external false` | 직접 USB 모드 강제 (Linux) |
| `dsr2 real --with-gripper false` | 그리퍼 서버 자동 시작 비활성화 |
| `dsr2 gripper` | 그리퍼 서버만 시작 (자동 감지) |
| `dsr2 gripper --gripper-port COM3` | 특정 포트로 그리퍼 서버 |
| `dsr2 gripper --gripper-port mock` | Mock 그리퍼 서버 |
| `dsr2 gripper --stop` | managed background gripper server 종료 |

---

## 환경 변수

`.env` 파일에서 기본값 설정:

```bash
# Linux 직접 연결
GRIPPER_PORT=/dev/ttyACM0
GRIPPER_BAUDRATE=115200
GRIPPER_EXTERNAL=false

# Windows/macOS TCP 서버
GRIPPER_EXTERNAL=true
GRIPPER_HOST=host.docker.internal
GRIPPER_PORT_TCP=9999
```

---

## 문제 해결

### 포트를 찾을 수 없음

- 그리퍼 전원 및 USB 연결 확인
- 드라이버 설치 확인 (Windows)
- `--gripper-port mock`으로 먼저 테스트

### 그리퍼 서버 연결 실패

1. 그리퍼 서버 실행 확인
2. 방화벽에서 9999 포트 허용
3. `dsr2 gripper --gripper-port mock` 또는 `dsr2 virtual --gripper-port mock`으로 테스트

### 그리퍼가 응답하지 않음

1. 그리퍼 전원 확인
2. Modbus 설정 확인 (115200 baud, 8N1)
3. USB 케이블 교체 시도
