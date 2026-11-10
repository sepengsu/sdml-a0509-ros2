# 문제 해결 가이드

자주 발생하는 문제와 해결 방법입니다.

---

## Docker 관련

### Docker가 실행되지 않음

**증상:** `docker: command not found` 또는 `Cannot connect to Docker daemon`

**해결:**
```bash
# Docker 상태 확인
docker info

# Docker Desktop 재시작 (Windows/macOS)
# 또는 Docker 서비스 시작 (Linux)
sudo systemctl start docker
```

### 이미지 빌드 실패

**해결:**
```bash
# 캐시 삭제 후 재빌드
docker system prune -a
dsr2 virtual --no-build  # 기존 이미지 사용
# 또는
docker compose --profile virtual build --no-cache
```

### 디스크 공간 부족

```bash
# Docker 리소스 정리
docker system prune -a
docker volume prune
```

---

## ROS 2 관련

### 패키지를 찾을 수 없음

**증상:** `Package 'dsr_bringup2' not found`

**원인:** 워크스페이스가 빌드되지 않았거나 source되지 않음

**해결:**
```bash
docker exec -it ros-control bash
cd /ros2_ws
colcon build
source install/setup.bash
```

### colcon build 권한 오류

**증상:** `PermissionError: [Errno 13] Permission denied: 'log'`

**해결:**
```bash
sudo chown -R robot:robot /ros2_ws
colcon build
```

> Dockerfile을 업데이트했다면 이미지 재빌드 필요: `dsr2 virtual --build`

### 토픽이 보이지 않음

**해결:**
```bash
# ROS 2 데몬 재시작
ros2 daemon stop
ros2 daemon start
ros2 topic list

# DOMAIN_ID 확인
echo $ROS_DOMAIN_ID
```

---

## GUI / noVNC 관련

### noVNC 페이지에 접속이 안 됨

**증상:** `http://localhost:6080` 접속 불가

**해결:**
```bash
# 1. 컨테이너 실행 확인
docker ps | grep ros-control

# 2. 포트 확인
curl http://localhost:6080

# 3. VNC 프로세스 확인
docker exec ros-control ps aux | grep -E "Xvfb|x11vnc|websockify"

# 4. 컨테이너 로그 확인
docker logs ros-control
```

### 화면이 검은색/회색임

**원인:** 창 관리자(Fluxbox)가 시작되지 않음

**해결:**
```bash
# 로그에서 Fluxbox 오류 확인
docker logs ros-control | grep -i fluxbox

# 컨테이너 재시작
docker compose --profile virtual restart ros-control
```

### GUI 앱 실행 시 "cannot open display" 오류

**원인:** DISPLAY 환경 변수가 설정되지 않음

**해결:**
```bash
# docker exec 시 DISPLAY 명시
docker exec -it ros-control bash -c "export DISPLAY=:99 && rviz2"

# 또는 컨테이너 내부에서
export DISPLAY=:99
rviz2 &
```

### VNC 연결이 느림/끊김

**해결:**
- 해상도 낮추기: `.env`에 `VNC_RESOLUTION=1280x720`
- 유선 네트워크 사용
- 브라우저 캐시 삭제 후 재접속

자세한 내용: [XSERVER.md](./XSERVER.md)

---

## 에뮬레이터 관련

### 에뮬레이터 연결 실패

**증상:** `Cannot connect to DRCF`

**해결:**
```bash
# 에뮬레이터 상태 확인
docker ps | grep emulator

# 에뮬레이터 로그 확인
docker logs dsr01_emulator

# 포트 연결 테스트
docker exec ros-control bash -c "python3 -c \"import socket; s=socket.socket(); s.connect(('robot-emulator', 12345)); print('OK')\""
```

### 에뮬레이터 재시작

```bash
docker compose --profile virtual restart emulator
```

---

## 실제 로봇 연결

### 로봇에 연결할 수 없음

**해결:**
```bash
# 네트워크 연결 확인
ping 192.168.137.100

# 같은 서브넷인지 확인
ip addr  # Linux
ipconfig  # Windows
```

### 로봇 상태가 STANDBY가 아님

로봇이 비상 정지 상태이거나 다른 클라이언트가 제어권을 가지고 있을 수 있습니다.

1. 로봇 티치 펜던트에서 상태 확인
2. 다른 연결 해제
3. 비상 정지 해제

---

## 그리퍼 관련

### 그리퍼 포트를 찾을 수 없음

**Windows:**
1. 장치 관리자 → 포트(COM & LPT)
2. DH-Robotics 또는 USB Serial Device 확인

**Linux:**
```bash
ls /dev/ttyACM* /dev/ttyUSB*
dmesg | tail -20  # 최근 USB 연결 로그
```

**macOS:**
```bash
ls /dev/tty.usb*
```

### 그리퍼 서버 연결 실패

```bash
# 서버 실행 확인
dsr2 gripper --gripper-port mock  # Mock 서버만 실행
dsr2 virtual --gripper-port mock  # Virtual과 함께 실행

# 포트 확인
netstat -an | grep 9999  # Linux/macOS
netstat -an | findstr 9999  # Windows
```

컨테이너를 내린 뒤에도 `9999` 포트가 남아 있다면 `dsr2 gripper --stop`으로 managed background gripper server를 정리합니다.

### USB 패스스루 실패 (WSL2)

PowerShell (관리자):
```powershell
# 장치 목록
usbipd list

# 재연결
usbipd detach --busid <BUSID>
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

---

## 성능 문제

### 컨테이너 시작이 느림

1. **WSL2 파일시스템 사용:** `/home/user/`에 클론 (not `/mnt/c/`)
2. **Docker 리소스 할당:** Settings → Resources → Memory 8-16GB

### ROS 2 통신이 느림

```bash
# 토픽 발행 주기 확인
ros2 topic hz /joint_states
# 예상: 100Hz (real), 10Hz (virtual)
```

---

## 검증 체크리스트

### Virtual 모드

```bash
# 1. 컨테이너 상태
docker compose ps

# 2. 에뮬레이터 연결
docker logs dsr01_emulator | tail -5

# 3. ROS 토픽
docker exec ros-control ros2 topic list

# 4. Joint states 발행
docker exec ros-control ros2 topic hz /joint_states
```

### Real 모드

```bash
# 1. 로봇 연결
ping 192.168.137.100

# 2. 그리퍼 연결 (Linux)
ls /dev/ttyACM0

# 3. 그리퍼 서버 (Windows/macOS)
# 별도 터미널에서: dsr2 gripper --gripper-port mock
# 또는 한 번에: dsr2 virtual --gripper-port mock
netstat -an | grep 9999     # Linux/macOS
netstat -an | findstr 9999  # Windows
```

---

## 로그 수집

문제 보고 시 다음 정보를 포함해주세요:

```bash
# 시스템 정보
uname -a
docker --version
docker compose version

# 컨테이너 로그
docker compose logs > docker-logs.txt

# ROS 로그
ls ~/.ros/log/
```
