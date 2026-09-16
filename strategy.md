# SDML A0509 패키지 정리 전략

## 목표
- 기존 기능을 깨지 않고 패키지 구조를 다음 4개 축으로 재편한다.
  - `sdml_a0509_description`: 로봇/그리퍼 모델(URDF/Xacro/mesh/RViz)
  - `sdml_a0509_control`: 노드/시나리오 실행 로직(rclpy, 서비스/토픽)
  - `sdml_a0509_moveit_config`: MoveIt 설정/런치 래퍼
  - `sdml_a0509_bringup`: 실행 진입점(virtual/real/pick_and_place 통합 런치)

## 현재 상태(요약)
- `sdml_a0509_robot`에 launch + python node + urdf 자산이 혼재되어 있음.
- 신규 4개 패키지 스켈레톤은 생성되어 있으나, 대부분은 기존 패키지 include 기반.
- `/do_pick_and_place`는 `std_msgs/String`로 전환됨(기존 custom msg 의존 제거 방향).

## 원칙
- 동작 우선: 한 번에 대규모 이동하지 말고 "호환 레이어"를 유지하며 단계 이관.
- 인터페이스 고정: 런치 인자(`host`, `rt_host`, `mode`, `model`)는 기존과 동일 유지.
- 검증 선행: 각 단계마다 `colcon build` + 핵심 런치 smoke test 수행.

## 단계별 이관 계획

### 1단계: Description 자산 분리
- 대상:
  - `sdml_a0509_robot/urdf/*`
  - (필요 시) description 관련 RViz/config/mesh
- 작업:
  - `sdml_a0509_description`로 파일 이동
  - path 참조를 `package://sdml_a0509_description/...`로 정리
  - placeholder URDF 제거 후 실제 URDF 반영
- 완료 기준:
  - `robot_state_publisher`가 `sdml_a0509_description` 자산만으로 정상 기동

### 2단계: Control 로직 분리
- 대상:
  - `sdml_a0509_robot/pick_and_place/*.py`
  - `sdml_a0509_robot/sdml_a0509_robot/*.py`(래퍼)
- 작업:
  - 실행 노드(`node.py`, `merge_node.py`, scenario/utils)를 `sdml_a0509_control`로 이동
  - `setup.py` console script를 `sdml_a0509_control` 기준으로 전환
  - 내부 import에서 구 패키지명 의존 제거
- 완료 기준:
  - `ros2 run sdml_a0509_control pick_and_place_node` 정상 동작

### 3단계: MoveIt 설정 정리
- 대상:
  - 현재 `dsr_moveit_config_a0509` 참조부
- 작업:
  - 단기: `sdml_a0509_moveit_config`는 래퍼 유지
  - 중기: 프로젝트 고유 설정(필요한 수정본)이 있으면 `sdml_a0509_moveit_config` 내부로 복제/관리
- 완료 기준:
  - `ros2 launch sdml_a0509_moveit_config start.launch.py` 동작

### 4단계: Bringup 단일 진입점 확정
- 대상:
  - `sdml_a0509_robot/launch/*.launch.py`
  - `sdml_a0509_bringup/launch/*.launch.py`
- 작업:
  - `sdml_a0509_bringup` 런치가 직접 `description + control + moveit_config`를 조합하도록 변경
  - `sdml_a0509_robot` include 의존 제거
- 완료 기준:
  - 운영 명령 표준화:
    - `ros2 launch sdml_a0509_bringup a0509_virtual.launch.py`
    - `ros2 launch sdml_a0509_bringup a0509_real.launch.py`

### 5단계: 레거시 정리
- 대상:
  - `sdml_a0509_robot` (임시 호환 패키지)
  - `pick_and_place_msgs` (미사용 시)
- 작업:
  - 참조가 완전히 사라졌는지 검색 후 제거
  - README/scripts/docs 명령어 전면 교체
- 완료 기준:
  - repo 전역에서 레거시 패키지명 참조 0건

## 파일/기능 매핑 가이드
- `launch/*.launch.py` → `sdml_a0509_bringup`
- `pick_and_place node/scenario/utils` → `sdml_a0509_control`
- `urdf/mesh/rviz` → `sdml_a0509_description`
- moveit 래퍼/설정 → `sdml_a0509_moveit_config`

## 검증 체크리스트
- 공통:
  - `colcon build --symlink-install`
  - `source install/setup.bash`
- 런치:
  - virtual: bringup 기동 + `host:=robot-emulator rt_host:=robot-emulator`
  - real: bringup 기동 + 실제 host
- 기능:
  - `/do_pick_and_place`에 `std_msgs/String(data='go')` publish 시 시나리오 실행
  - MoveIt plan/execution 요청 시 controller timeout/Disconnected 재발 여부 확인

## 리스크와 대응
- 리스크: import 경로/entrypoint 누락으로 런타임 에러
  - 대응: 단계마다 `rg`로 구 패키지명 탐지 후 즉시 수정
- 리스크: launch 인자 누락(`rt_host`)로 Windows Docker에서 재발
  - 대응: bringup 표준 런치 인자에 `rt_host` 고정 노출
- 리스크: docs/스크립트 불일치
  - 대응: 마지막 단계에서 README/scripts 일괄 정리

## 권장 운영 명령(정리 완료 후)
- `ros2 launch sdml_a0509_bringup a0509_virtual.launch.py host:=robot-emulator rt_host:=robot-emulator`
- `ros2 run sdml_a0509_control pick_and_place_node`

