# GUI 접속 가이드 (noVNC)

Docker 컨테이너에서 RViz, Gazebo 등 GUI 애플리케이션을 사용하기 위한 가이드입니다.

이 프로젝트는 **VNC/noVNC**를 사용하여 모든 OS에서 동일한 방식으로 GUI에 접근합니다.
X Server 설치가 필요 없습니다.

---

## 접속 방법

### Step 1: 컨테이너 시작

```bash
# Virtual 모드
dsr2 virtual

# 또는 docker-compose 직접 사용
docker compose --profile virtual up -d
```

### Step 2: 브라우저에서 접속

컨테이너가 시작되면 브라우저에서 접속합니다:

| 프로파일 | noVNC URL |
|----------|-----------|
| virtual | http://localhost:6080 |
| real | http://localhost:6081 |
| real-external | http://localhost:6082 |

### Step 3: Connect 클릭

noVNC 페이지에서 **Connect** 버튼을 클릭하면 데스크톱이 표시됩니다.

---

## 데스크톱 환경

- **창 관리자**: Fluxbox
- **터미널**: xterm (자동 실행)
- **해상도**: 1920x1080 (기본값)

### 기본 사용법

- **우클릭**: Fluxbox 메뉴 표시
- **터미널에서 ROS 명령 실행**

### ROS GUI 앱 실행 예시

터미널에서:
```bash
# RViz2
rviz2 &

# RQt
rqt &

# Joint State Publisher GUI
ros2 run joint_state_publisher_gui joint_state_publisher_gui &
```

---

## 해상도 변경

`.env` 파일에서 해상도를 변경할 수 있습니다:

```bash
VNC_RESOLUTION=1280x720
```

또는 docker-compose 실행 시:
```bash
VNC_RESOLUTION=2560x1440 docker compose --profile virtual up -d
```

---

## VNC 클라이언트 접속 (선택사항)

noVNC 대신 전용 VNC 클라이언트를 사용할 수도 있습니다:

- **주소**: `localhost:5900`
- **비밀번호**: 없음

권장 VNC 클라이언트:
- Windows: [TightVNC Viewer](https://www.tightvnc.com/download.php)
- macOS: 기본 Screen Sharing 또는 [RealVNC](https://www.realvnc.com/)
- Linux: `vncviewer`, `remmina`

---

## 문제 해결

### 화면이 검은색/회색인 경우

창 관리자가 시작되지 않았을 수 있습니다:
```bash
docker logs ros-control
```

Fluxbox 관련 로그를 확인하세요.

### 연결이 안 되는 경우

1. 컨테이너가 실행 중인지 확인:
   ```bash
   docker ps
   ```

2. 포트가 열려 있는지 확인:
   ```bash
   curl http://localhost:6080
   ```

3. 컨테이너 로그 확인:
   ```bash
   docker logs ros-control
   ```

### 성능이 느린 경우

- 해상도를 낮추세요 (`VNC_RESOLUTION=1280x720`)
- 유선 네트워크 사용 권장

---

## 기존 X Server 방식과의 비교

| 항목 | X Server (이전) | noVNC (현재) |
|------|-----------------|--------------|
| Windows 설정 | VcXsrv/X410 설치 필요 | 불필요 |
| macOS 설정 | XQuartz 설치 필요 | 불필요 |
| Linux 설정 | xhost 설정 필요 | 불필요 |
| 접속 방법 | 호스트에 창 표시 | 브라우저 접속 |
| 크로스 플랫폼 | OS별 설정 다름 | 모두 동일 |
| 원격 접속 | 어려움 | URL 공유로 가능 |

---

## 아키텍처

```
┌─────────────────────────────────────────────────────┐
│  Docker Container                                   │
│  ┌─────────┐  ┌─────────┐  ┌──────────────────┐   │
│  │  Xvfb   │──│ x11vnc  │──│   websockify     │   │
│  │  :99    │  │  :5900  │  │   (noVNC) :6080  │   │
│  └─────────┘  └─────────┘  └──────────────────┘   │
│       │                            │               │
│  ┌─────────┐                       │               │
│  │ Fluxbox │                       │               │
│  │ + xterm │                       │               │
│  └─────────┘                       │               │
└────────────────────────────────────│───────────────┘
                                     │
                              ┌──────▼──────┐
                              │   Browser   │
                              │ :6080/vnc.html│
                              └─────────────┘
```
