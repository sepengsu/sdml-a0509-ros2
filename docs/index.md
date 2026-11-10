# Doosan A0509 Robot Control Environment

Docker 기반 두산 A0509 로봇 제어 환경입니다. Windows, macOS, Linux를 모두 지원합니다.

## 문서 목차

| 문서 | 설명 |
|------|------|
| [INSTALLATION.md](./INSTALLATION.md) | 설치 가이드 (Docker, WSL2, 사전 요구사항) |
| [QUICKSTART.md](./QUICKSTART.md) | 빠른 시작 및 CLI 명령어 |
| [XSERVER.md](./XSERVER.md) | GUI 접속 가이드 (noVNC, 브라우저 기반) |
| [GRIPPER.md](./GRIPPER.md) | 그리퍼 사용 가이드 (플랫폼별) |
| [CONFIGURATION.md](./CONFIGURATION.md) | 환경 변수 및 설정 |
| [ARCHITECTURE.md](./ARCHITECTURE.md) | 시스템 아키텍처 |
| [TROUBLESHOOTING.md](./TROUBLESHOOTING.md) | 문제 해결 |

---

## 빠른 시작

### 1. 레포지토리 클론 및 실행

```bash
git clone https://github.com/c0sogi/sdml-doosan-robot --recurse-submodules
cd sdml-doosan-robot
uv run dsr2 virtual
```

```bash
# Virtual + Mock Gripper Server
uv run dsr2 virtual --gripper-port mock
```

### 2. uv가 설치되어 있지 않은 경우

**Windows (PowerShell):**
```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

**Linux/macOS:**
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### 3. 완료

도커 이미지 빌드, colcon 빌드, ROS2 launch까지 자동으로 수행됩니다.

완료 후 **http://localhost:6080** 접속 → RViz GUI 확인

자세한 내용은 [QUICKSTART.md](./QUICKSTART.md)를 참조하세요.

---

## 플랫폼 지원

| 플랫폼 | Virtual 모드 | Real 모드 | GUI (RViz) |
|--------|-------------|-----------|------------|
| **Linux** | ✅ | ✅ 직접 USB | ✅ noVNC |
| **Windows** | ✅ | ✅ TCP 서버 | ✅ noVNC |
| **macOS** | ✅ | ✅ TCP 서버 | ✅ noVNC |

---

## 주요 명령어

| 명령어 | 설명 |
|--------|------|
| `dsr2 virtual` | 시뮬레이션 모드 시작 |
| `dsr2 virtual --gripper-port mock` | 시뮬레이션 + Mock 그리퍼 서버 |
| `dsr2 real --ip <IP>` | 실제 로봇 연결 |
| `dsr2 gripper` | 그리퍼 서버만 시작 |
| `dsr2 gripper --stop` | managed background gripper server 종료 |
| `dsr2 --help` | 도움말 |

---

## 추가 리소스

- [Doosan Robot Framework](https://github.com/doosan-robotics/doosan-robot2)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MoveIt 2 Documentation](https://moveit.picknik.ai/)
