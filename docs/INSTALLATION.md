# 설치 가이드

Docker 기반 두산 로봇 제어 환경 설치 방법입니다.

---

## 사전 요구사항

### 모든 플랫폼 공통

- **Docker Desktop** 또는 **Docker Engine**
- **Git**
- **Python 3.10+** (CLI 도구용)

### 시스템 요구사항

- RAM: 최소 8GB (16GB 권장)
- 디스크: 20GB 이상 여유 공간
- CPU: 쿼드코어 이상

---

## Windows 11 설치

### Step 1: WSL2 설치

PowerShell을 **관리자 권한**으로 실행:

```powershell
# WSL2 + Ubuntu 22.04 설치
wsl --install -d Ubuntu-22.04

# 설치 후 재부팅 필요
```

설치 완료 후 Ubuntu 터미널에서 사용자명/비밀번호를 설정합니다.

**확인:**
```powershell
wsl --list --verbose
# Ubuntu-22.04 VERSION 2 확인
```

### Step 2: Docker Desktop 설치

1. [Docker Desktop](https://www.docker.com/products/docker-desktop) 다운로드
2. 설치 시 **"Use WSL 2 based engine"** 체크
3. 재부팅

**Docker Desktop 설정:**
1. Settings → Resources → WSL Integration
2. **Ubuntu-22.04** 활성화
3. Apply & Restart

**확인 (WSL2 터미널에서):**
```bash
docker --version
docker compose version
docker run hello-world
```

> **참고:** GUI(RViz, Gazebo)는 noVNC를 통해 브라우저에서 접속합니다. X Server 설치가 필요 없습니다. 자세한 내용은 [XSERVER.md](./XSERVER.md)를 참조하세요.

### Step 3: 프로젝트 클론

**중요:** WSL2 파일시스템 내부에 클론하세요 (`/home/username/`). `/mnt/c/`에 클론하면 성능이 5-10배 느려집니다.

```bash
cd ~
git clone <repository-url> sdml-doosan-robot
cd sdml-doosan-robot
```

### Step 4: CLI 설치

```bash
pip install -e .
# 또는
uv pip install -e .
```

---

## Linux 설치

### Step 1: Docker 설치

```bash
# Docker Engine 설치 (Ubuntu)
sudo apt-get update
sudo apt-get install -y docker.io docker-compose-v2

# 현재 사용자를 docker 그룹에 추가
sudo usermod -aG docker $USER
newgrp docker

# 확인
docker --version
docker compose version
```

### Step 2: 프로젝트 클론 및 CLI 설치

```bash
git clone <repository-url> sdml-doosan-robot
cd sdml-doosan-robot
pip install -e .
```

---

## macOS 설치

### Step 1: Docker 환경 설치

두 가지 방법 중 선택:

#### 옵션 A: Docker Desktop (간편)

1. [Docker Desktop for Mac](https://www.docker.com/products/docker-desktop) 다운로드
2. 설치 및 실행

#### 옵션 B: Colima (무료, 경량)

Docker Desktop 대신 오픈소스 Colima를 사용할 수 있습니다. Apple Silicon(M1/M2/M3/M4)에서도 잘 작동합니다.

```bash
# Homebrew로 설치
brew install colima docker docker-compose

# Colima 시작 (Apple Silicon의 경우 Rosetta 포함)
colima start --arch aarch64 --vm-type vz --vz-rosetta

# 확인
docker --version
docker compose version
```

**편의를 위한 alias 설정 (~/.zshrc에 추가):**
```bash
alias dstart='colima start --arch aarch64 --vm-type vz --vz-rosetta'
alias dstop='colima stop'
```

> **참고:** Colima를 시작해야 `docker` 명령이 작동합니다. 재부팅 후에도 `dstart` (또는 `colima start ...`)를 실행해야 합니다.

### Step 2: 프로젝트 클론 및 CLI 설치

```bash
git clone <repository-url> sdml-doosan-robot
cd sdml-doosan-robot
pip install -e .
```

---

## USB 패스스루 설정 (Real 모드, WSL2)

실제 그리퍼 하드웨어 연결 시 필요합니다.

### usbipd-win 설치

PowerShell (관리자):
```powershell
winget install usbipd
```

### USB 장치 연결

```powershell
# 장치 목록 확인
usbipd list

# 장치 바인드 및 WSL에 연결 (BUSID를 실제 값으로 교체)
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

### WSL2에서 확인

```bash
ls /dev/ttyACM*
```

---

## 다음 단계

- [QUICKSTART.md](./QUICKSTART.md) - 빠른 시작
- [XSERVER.md](./XSERVER.md) - GUI 접속 가이드 (noVNC, X Server 설치 불필요)
