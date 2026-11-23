# sim_real_gap_getting

A0509의 JSON 관절 경로를 MuJoCo에서 연습하고, 별도 ROS domain에서 실로봇 RT 상태를 확인하는 ROS 2 패키지입니다.

## 빌드와 실행

T1–T16에 15개 마찰 시나리오를 적용하는 별도 v3 실행기는
`ros2 run sim_real_gap_getting validate_friction`입니다.
모델 분석 → Offline → 실제 Dynamic pilot → 전체 수집 순서로 실행하며,
기존 F1–F4 수집과 결과를 분리합니다.
[감쇠 분석·계수·Ubuntu 실행 안내](../../docs/FRICTION_ROS2_V3.md)를 참고하세요.

ROS 2 Humble, MuJoCo, `a0509_mujoco_sim`을 준비한 워크스페이스에서:

```bash
colcon build --packages-up-to sim_real_gap_getting
source install/setup.bash
ros2 launch sim_real_gap_getting practice.launch.py
```

이 launch는 `ROS_DOMAIN_ID=173`에서 MuJoCo와 Tk GUI를 함께 시작하고 예제 JSON을 불러옵니다. 다른 경로는 `trajectory_file:=/absolute/path/trajectory.json`으로 지정하거나 GUI에서 선택합니다. 연습용 Home 관절각은 **[0, -32.01, 84.97995, -3.99, 115.02, 45.00]°**입니다. 기존 작업 시나리오의 Home에서 J1만 0°로 바꾼 값입니다. GUI의 **Go Home in simulation**을 누르면 현재 측정된 시뮬레이터 자세에서 이 Home까지 8초간 100 Hz의 완만한 경로로 이동합니다. 이어서 **Run in simulation**을 누릅니다. 시작 관절 자세가 JSON 첫 점에서 0.05 rad 이상 다르면 실행하지 않습니다. `sim_domain_id:=...`, `real_domain_id:=...`, `gripper_type:=none|ag145|mdpg_1s1` 등의 launch 인수를 사용할 수 있습니다. 두 domain ID는 달라야 합니다.

GUI의 **Save folder**에서 **Choose folder**로 저장할 기본 폴더를 지정합니다. **Gripper option**에서 `None`, `AG145`, `mDPG`를 고른 뒤 **Save gripper JSON**을 누르면 선택한 그리퍼가 `gripper_type`으로 기록된 JSON을 `<선택 폴더>/<gripper>/`에 저장합니다. 파일명이 이미 있으면 `_001`, `_002`를 붙입니다. 이 선택은 파일에 붙는 도구 정보이며 실물 장착 상태를 확인하지 않습니다. MuJoCo 모델은 launch 때 선택되므로 다른 그리퍼로 시뮬레이션하려면 `gripper_type:=...`으로 다시 실행해야 합니다.

## JSON 형식

### J1 고정 전진 및 뒤쪽 벽 연습

```bash
ros2 launch sim_real_gap_getting forward_wall.launch.py
```

이 전용 launch는 그리퍼 `None`의 A0509, 로봇 기준 X=-350 mm의 뒤쪽 벽, `examples/forward_150mm_wall.json`을 함께 띄웁니다. GUI에서 **Go Home in simulation**으로 시작 자세를 만든 뒤 **Run in simulation**을 누르세요. JSON은 J1=0°를 전 구간 유지하고, J2·J3으로 플랜지(`link_6`)를 로봇 기준 +X로 150 mm 움직입니다. 100 Hz 경로를 MuJoCo 모델에서 접촉·관절 한계·전진 단조성을 검사했습니다. 이 모델 기반 검사는 실제 벽 위치나 실로봇 충돌 안전성을 확인한 것은 아닙니다.

두 번의 직각 계단 경로는 아래 전용 launch로 엽니다. J1은 0°로 유지하고, 플랜지가 +X로 200 mm 전진한 뒤 -Z로 200 mm 하강하는 구간을 두 번 반복합니다. 각 구간은 8초, 시작과 끝 정지는 각각 1초로 전체 34초입니다. 모서리마다 정지하며 뒤쪽 벽 방향으로 돌아가지 않습니다. GUI에서 **Go Home in simulation** 다음 **Run in simulation**을 누릅니다.

```bash
ros2 launch sim_real_gap_getting right_angle_wall.launch.py
```


`examples/small_joint_1.json`을 형식 예시로 사용합니다. 예제는 위 Home에서 시작해 J1만 0.03 rad 움직였다가 Home으로 돌아옵니다. 관절 순서는 `joint_1`부터 `joint_6`, 단위는 rad, rad/s, rad/s²입니다. 각 `points` 항목에는 `time_s`, `q_rad`, `dq_rad_s`, `ddq_rad_s2`의 6개 값이 필요합니다. 시간은 0에서 시작해 증가해야 하고 시작과 끝의 속도와 가속도는 0이어야 합니다. 점 사이에는 5차 다항식으로 보간하고 100 Hz 표본의 관절 위치·속도·가속도를 시뮬레이션 한계와 대조합니다. 기존 `a0509_dynamics_experiment`가 내보낸 관절 경로 `trajectory.json`의 `points`도 읽습니다. 관절값이 없는 Cartesian JSON은 실행할 수 없습니다.

```bash
ros2 run sim_real_gap_getting validate_trajectory /absolute/path/trajectory.json
```

## 좁은 공간 좌우 반복 궤적 (MuJoCo 사전 검증)

`generate_lr_trajectories`는 현재 디렉터리의 `a0509_lr_outputs/`에
**현재 설정 자세에서 DR_BASE +X로 100 mm 이동하는 1회 준비 JSON**과
P1~P5 100 Hz **1 Cycle** JSON,
`a0509_lr_run_config.json`, `a0509_lr_validation.json`, 경로별 미리보기 PNG를 만듭니다.
기본 좌우 범위는 모델의 `DR_BASE Y`에서 중심 ±100 mm이고, 반복 설정은 10회입니다.
설정 파일의 `cycle_repeats`, `half_width_mm`(최대 120), `center_offset_mm`를
바꾼 뒤 `--config`로 다시 생성할 수 있습니다. 전체 좌우 사용 범위는 기준 중심
±150 mm 안에 있어야 합니다.

```bash
ros2 run sim_real_gap_getting generate_lr_trajectories
ros2 run sim_real_gap_getting generate_lr_trajectories \
  --config /path/to/a0509_lr_run_config.json --expand-repeats
```

기존 GUI/JSON 실행기는 반복 설정을 직접 읽지 않으므로 `--expand-repeats`는
선택적으로 준비 이동 1회와 Cycle 10회를 연결한 별도 JSON도 생성합니다.
`start_joint_rad`는 설정 파일에서 변경할 수 있습니다. 기본값은 패키지에서 만든
시뮬레이션 후보 자세이며 실로봇의 현재 관절값을 읽은 결과가 아닙니다.
모든 출력은 `simulation_only`이고
실기 스트리밍에서 차단됩니다. 검증은 MuJoCo의 bare A0509, 바닥, X=-0.35 m
후방 벽 모델에 한정됩니다. 실제 장착 도구, 주변 장애물, 현재 로봇 자세와
100 Hz 실기 인터페이스는 별도로 확인해야 합니다. `validation.json`에는 각
경로의 관절 최대 속도·가속도·jerk, TCP 범위, 모델 충돌·특이점 검사와
MuJoCo 사전 추종 결과가 기록됩니다.

## 전방 공간을 활용한 4개 반복 궤적

`generate_front_trajectories`는 좁은 실험실에서 후방 벽을 피하고 전방 공간을
활용하는 새 캠페인입니다. 기본 시작 자세는 위 좌우 캠페인의 설정 자세와 같고,
먼저 DR_BASE +X 100 mm를 이동하고 Z를 60 mm 올리는 준비 이동을 **1회** 실행한 뒤 네 종류의 닫힌 Cycle을
각각 기본 10회 반복합니다.

- F1: 전방 약 200 mm 저속 왕복(20초/Cycle)
- F2: 같은 공간 경로를 중속으로 왕복(14초/Cycle)
- F3: 전방의 네 자세에서 X/Z를 바꾸며 J2/J3을 가진하고 두 자세에서 정지 유지(44초/Cycle)
- F4: 한 전방 위치에서 팔을 멈춘 채 J4~J6만 두 자세로 움직이고 정지 유지(50초/Cycle)

F3은 전방 여러 자세에서 TCP를 약 20 mm만 옆으로 이동하고 약 23 mm 높이를
바꿉니다. 보수적 CAD 외형을 포함한 준비 이동과 네 Cycle의 **팔 전체·그리퍼**
모델 점유폭은 AG145 약 **402.5 mm**, mDPG 약 **343.5 mm**입니다. AG145는
총 400 mm 조건을 모델상 초과하므로 실기 후보에서 제외합니다. mDPG도 실제
공간의 좌우 경계 위치를 모르면 통과로 표시하지 않습니다.
전방 이동폭은 경로별 약 200~241 mm이며, 준비 자세보다 뒤로 이동하지 않습니다.
기존 후방 벽 `X=-0.35 m`, bare A0509 모델의 링크 충돌, 관절 한계,
특이점 근접, 100 Hz 시간·도함수와 MuJoCo 추종을 확인합니다. 실제 그리퍼,
페이로드 및 실험실 장애물은 검증되지 않아 모두 시뮬레이션 전용입니다.

`audit_front_grippers`는 각 100 Hz 샘플의 AG145 근사 충돌 박스·CAD 외형과
mDPG CAD·80×80×100 mm 계획용 박스를 검사합니다. 30 mm는 계획 여유값이며
실로봇 인증값이 아닙니다. 준비 이동의 최저 모델 높이(DR_BASE Z)는 AG145
약 406 mm, mDPG 약 506 mm입니다. 네 Cycle에서는 각각 약 476 mm, 564 mm
이상입니다. mDPG 장착 변환은 현재 0 위치의 **시각화용 가정**입니다.
작업대 상면 높이와 실제 좌우 경계, 장착 어댑터와 케이블 치수를 측정하기
전에는 `model_clearance_passed`와 `measured_corridor_passed`가 `null`이고
실로봇 실행은 차단됩니다. 전체 구간을 가로지르는 수평 장애물을 가정한
보수적 상면 검사는 다음처럼 다시 실행할 수 있습니다.

네 경로는 동일한 명령을 반복해 관절 추종 오차와 그리퍼 조건 차이를 비교하는
첫 실험에 적합합니다. 좌우 폭 때문에 J1 가진이 작으므로 여섯 관절의
동역학 Residual을 일반화하려면 별도의 안전한 시작 자세·높이 또는 더 넓은
작업공간에서 J1 검증 경로가 추가로 필요합니다.
`audit_front_excitation`은 목표 관절의 움직임 범위와 MuJoCo 질량행렬의
관성 토크, 관절 감쇠 토크를 분리해 `a0509_front_quality_audit.json`에 기록합니다.
F1/F2는 같은 위치 경로를 다른 속도로 지나가며, F3에서 J1의 전체 범위는
약 0.049 rad입니다. 현재 MuJoCo 모델은 J1~J3에 800 N·m·s/rad,
J4~J6에 600 N·m·s/rad의 감쇠가 설정되어 있습니다. 이 수치는 실측 보정값이
아니므로 모델 토크를 실기 토크의 정답으로 취급하지 않습니다.

```bash
ros2 run sim_real_gap_getting audit_front_excitation \
  --trajectory-dir /path/to/examples/front
```

## 전방 궤적의 시뮬레이션 데이터 받기

GUI의 **Run in simulation**은 목표 관절각을 재생하지만 측정값 파일을 저장하지
않습니다. `collect_front_sim`은 검증된 그리퍼 모델에서 준비 이동 1회와 선택한
Cycle 반복을 MuJoCo로 직접 실행하고, 100 Hz 목표 관절값·시뮬레이션 관절값·
속도·컨트롤러 명령 토크를 `*.jsonl.gz`로 압축 저장합니다. 옆의
`*.summary.json`에는 모델·입력 JSON의 해시, 표본 수, 충돌, 포화 및 추종 오차가
기록됩니다. 명령 토크는 실측 토크가 아닙니다. 첫 실험은 `--repeats 1`로
확인한 뒤 설정 기본값인 10회로 늘립니다. 두 그리퍼의 결과는 각각 저장합니다.

### F1~F4 일괄 수집 GUI

기존 Tk GUI의 **Collect F1–F4 data (MuJoCo)** 버튼을 누르거나 아래 명령으로
별도 창을 엽니다. **그리퍼 모델**에서 AG145, mDPG, 두 모델 모두 중 하나를
고르고 **경로별 반복**과 **저장 폴더**를 지정한 뒤 **F1~F4 모두 수집**을 누릅니다.
기본 반복은 10회이며, 먼저 1회로 파일과 그래프를 확인하는 것이 좋습니다.
네 경로는 각각 독립적으로 준비 이동을 한 번 수행하고 선택한 Cycle을 반복합니다.
**수집 중지**는 현재 압축 파일을 닫고 해당 실행을 중단합니다. 이 창은 ROS 및
실로봇에 연결하지 않는 MuJoCo 수집 전용입니다. **MuJoCo 장면** 칸에서
다른 XML을 선택하면 그 장면으로 사전 검증과 수집을 진행합니다.

```bash
ros2 run sim_real_gap_getting front_batch_gui
# GUI 없이 같은 배치를 실행할 때
ros2 run sim_real_gap_getting collect_front_batch \
  --trajectory-dir /path/to/examples/front \
  --tool both --repeats 1 --output-dir /path/to/results
```

저장 폴더 아래에 실행마다 `front_batch_<UTC 시간>/`이 생성됩니다. 그 안의
`manifest.json`에 경로별 성공 여부, 행 수, 압축 크기, 추종 오차가 있고,
`inputs/`에는 사용한 JSON과 사전 검증 보고서, `models/`에는 모델 파일,
`data/<gripper>/`에는 각 경로의 `*.jsonl.gz`와 `*.summary.json`이 있습니다.
두 모델 모두 선택하면 8개 데이터 파일을 만듭니다. 실제 공간과 장착 치수가
검증되지 않았으므로 이 배치 결과는 `simulation_only`로 기록됩니다.

CLI의 `--tool`은 `mdpg_1s1`, `ag145`, `none`, `both`를 지원합니다.
`none`은 그리퍼를 붙이지 않은 원본 A0509 장면을 사용합니다. 단일 모델을
선택하면 해당 모델만 사전 검사·수집합니다. `both`는 기존처럼 AG145와
mDPG 두 모델을 의미합니다. `--repeats 3`은 경로마다 준비 이동 1회 후
3사이클을 연속 기록하므로 모델당 4개 파일·12사이클입니다.

각 Ubuntu에 최신 코드를 반영하고 워크스페이스 루트에서 다시 빌드합니다.

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select sim_real_gap_getting
source install/setup.bash
```

아래는 Windows의 `F:\연구\grad`가 각 Ubuntu에서 `/mnt/f/연구/grad`로
보이는 경우입니다. WSL이 아닌 Ubuntu에서는 실제 드라이브 또는 공유 폴더의
마운트 경로로 바꿉니다. `/mnt/f`가 마운트되지 않으면 수집하지 않습니다.

```bash
# Ubuntu 1 — mDPG
mountpoint /mnt/f && ros2 run sim_real_gap_getting collect_front_batch --tool mdpg_1s1 --repeats 3 --output-dir "/mnt/f/연구/grad/mdpg"
```

```bash
# Ubuntu 2 — AG145
mountpoint /mnt/f && ros2 run sim_real_gap_getting collect_front_batch --tool ag145 --repeats 3 --output-dir "/mnt/f/연구/grad/ag145"
```

```bash
# Ubuntu 3 — 그리퍼 없음
mountpoint /mnt/f && ros2 run sim_real_gap_getting collect_front_batch --tool none --repeats 3 --output-dir "/mnt/f/연구/grad/none"
```

세 명령의 합계는 원자료 12개 파일·36사이클입니다. 출력 하위의 실행별
폴더와 모델 폴더를 분리하므로 같은 공유 드라이브에서도 파일이 겹치지 않습니다.
기본 후방 벽 장면이 설치되지 않은 환경에서는 `--arm-scene`으로 사용할
MuJoCo XML을 명시해야 하며, 선택한 XML이 검사와 수집에 함께 쓰입니다.

### 60th 모델 장면 대조

저장소 루트의 `doosan_changes_export.tar.gz`와 `scripts/apply_doosan_changes.sh`는
`D:\Jaewon\60th\sdml-doosan-robot`에서 복사했습니다. 압축 파일에는 A0509 장면의
time capsule 장애물, URDF·MoveIt 설정, 제어기 코드가 함께 들어 있습니다.
복원 스크립트는 `src/doosan-robot2/`의 기존 파일을 덮어쓰므로, 현재 RT 수정본과
차이를 검토한 뒤 사용해야 합니다. 이번 F1~F4 검증은 압축 파일 전체를 적용하지
않고 장애물 장면만 후방 벽 장면에 합쳐 수행했습니다.

```bash
python scripts/inspect_doosan_changes.py
# 차이를 검토한 뒤 전체 덮어쓰기가 필요할 때만:
bash scripts/apply_doosan_changes.sh doosan_changes_export.tar.gz
```

```bash
ros2 run sim_real_gap_getting build_front_60th_scene \
  --archive /path/to/doosan_changes_export.tar.gz \
  --output-dir /path/to/results/front_60th_scene
ros2 run sim_real_gap_getting audit_front_grippers \
  --trajectory-dir /path/to/examples/front \
  --scene-dir /path/to/results/front_60th_scene/models \
  --arm-scene /path/to/results/front_60th_scene/scene_forward_wall_60th_capsule.xml \
  --report-file /path/to/results/front_60th_scene/clearance.json
ros2 run sim_real_gap_getting collect_front_batch \
  --trajectory-dir /path/to/examples/front --tool both --repeats 1 \
  --arm-scene /path/to/results/front_60th_scene/scene_forward_wall_60th_capsule.xml
ros2 run sim_real_gap_getting review_front_real \
  --clearance-report /path/to/results/front_60th_scene/clearance.json \
  --validation-report /path/to/examples/front/a0509_front_validation.json \
  --output /path/to/results/front_60th_scene/real_readiness.json
```

이 장면에서 준비 이동과 F1~F4 모두 로봇 접촉은 모델상 0회였습니다. 원본
time capsule과 바닥 사이의 접촉은 로봇 접촉과 분리해 기록합니다. 충돌이 없다는
결과는 실물 물체의 현재 위치를 검증한 것이 아닙니다. `review_front_real`은
준비 이동과 각 Cycle을 함께 분류하며 실기 플래그를 자동으로 풀지 않습니다.
AG145의 F3는 보수적 모델 폭 약 402.5 mm로 400 mm 조건을 넘습니다. 나머지
AG145 경로도 여유가 전체 5~6 mm에 불과해 실제 경계·장착 치수 확인이
필요합니다. mDPG 역시 실제 경계 Y와 아래 장애물 Z가 미측정이라 차단됩니다.

```bash
ros2 run sim_real_gap_getting audit_front_grippers \
  --trajectory-dir /path/to/examples/front
ros2 run sim_real_gap_getting collect_front_sim \
  --trajectory-dir /path/to/examples/front \
  --tool ag145 --profile f1_slow_forward --repeats 1 \
  --output-dir /path/to/results/front_sim_data
```

현재 mDPG 장착 변환은 시각화용 가정입니다. 이 수집기는 실로봇에 연결하지
않습니다. 실로봇 수집 시에는 아래의 RT 점검과 물리적 작업공간 검증을 먼저
완료해야 하며, 실기 실행기가 별도의 `_samples.jsonl`과 실행 요약을 만듭니다.

```bash
ros2 run sim_real_gap_getting audit_front_grippers \
  --trajectory-dir /path/to/examples/front \
  --lower-obstacle-top-z-mm <BASE_Z_MM> \
  --corridor-y-min-mm <LEFT_BASE_Y_MM> --corridor-y-max-mm <RIGHT_BASE_Y_MM>
```

```bash
ros2 run sim_real_gap_getting generate_front_trajectories
ros2 run sim_real_gap_getting generate_front_trajectories \
  --config /path/to/a0509_front_run_config.json --expand-repeats
```

기본 출력 폴더는 현재 디렉터리의 `a0509_front_outputs/`입니다. 저장소의
`examples/front/`에는 생성된 JSON, 검증 보고서와 그래프가 있습니다.
`--expand-repeats`는 기존 실행기가 설정 파일을 읽지 못할 때 준비 이동 1회와
Cycle 반복을 이어 붙인 별도 JSON을 만듭니다. 반복 횟수와 구간 시간은
`a0509_front_run_config.json`에서 조정할 수 있습니다.

## Real RT 확인

`a0509_real.launch.py`에 `experiment_only:=true gui:=true`를 지정하면
**실제 A0509용 F1~F4 수집 GUI**와 RT 상태 브리지를 함께 시작합니다.
이 모드에서는 pick-and-place와 조깅 노드를 시작하지 않습니다. GUI만 띄워서는
로봇이 움직이지 않으며, **1. 실기 경로 검증 → 2. 실제 로봇 실행·저장**을
차례로 눌러야 합니다. 실기 GUI의 저장 폴더·장면·그리퍼·경로·반복 횟수를
직접 지정할 수 있습니다. gripper_type은 실제 장착 도구와 일치해야 하며,
mDPG는 launch에 실측 장착 변환값을 넣어야 합니다.

실기 GUI는 로봇 베이스 기준 좌우 경계 Y 최소·최대, 작업대 상면 Z, 장착
변환과 주변 장애물 확인을 요구합니다. 각 모델 경로는 좌우 경계로부터
20 mm, 아래 장애물로부터 30 mm의 계획 여유를 요구합니다. AG145는 400 mm
총폭에서 이 측면 여유를 확보하지 못하고, F3는 모델 외형 자체가 약 402.5 mm로
초과합니다. 총폭만 알고 경계의 실제 Y 좌표가 없으면 실기 실행 버튼이
활성화되지 않습니다. 기본은 경로별 1회이며 첫 실기 시험도 1회로 시작합니다.

검증을 통과하면 GUI가 `supervised_real_trial` 입력 사본을 만들고, 실제
로봇의 현재 관절값과 정지 상태·RT 피드백·추종 오차를 다시 확인합니다.
준비 이동은 전체 배치 시작 때 1회만 수행하며, 이어서 선택한 F 경로를
반복합니다. `<저장 폴더>/front_real_trial_<UTC>/`에는 `preflight.json`,
`model_audit.json`, 사용한 입력 JSON, `manifest.json`, 구간별 RT 목표·관측
`*_samples.jsonl`, 실행 요약과 로그가 저장됩니다. **실기 중지**는 현재
실행기에 정지 신호를 전달하고 남은 경로를 실행하지 않습니다.

현재 Doosan 하드웨어 드라이버는 RT 출력만 켜고 시작합니다. 실기 실행기는
기존 RT 세션을 정지한 뒤 입력·출력 주기를 모두 10 ms로 설정하고 재시작하여
첫 유지 명령을 바로 보냅니다. 마지막 자세는 100 Hz로 유지하며 정지 상태를
확인하고, 종료 시 입력 감시를 해제한 피드백 세션으로 복구합니다. RT 설정
서비스의 성공 응답은 설정 패킷 전송만 뜻하므로, 첫 실기 시험에서는
실제 제어기의 수신·동작을 현장에서 확인해야 합니다. 피드백 중단·마감 시간
초과·추종 오차가 발생하면 현재 구간을 중지하고 결과에 실패 원인을 남깁니다.

```bash
colcon build --symlink-install --packages-up-to sdml_a0509_bringup sim_real_gap_getting
source install/setup.bash
ros2 launch sdml_a0509_bringup a0509_real.launch.py \
  gripper_type:=mdpg_1s1 experiment_only:=true gui:=true \
  mdpg_mount_xyz:='X Y Z' mdpg_mount_rpy:='R P Y'
```

`X Y Z`는 실측 장착 위치(m), `R P Y`는 실측 장착 각도(rad)로 바꿉니다.
AG145는 `gripper_type:=ag145`로 GUI를 열 수 있지만, 현재 F1~F4 모델은
400 mm 공간과 양쪽 20 mm 여유를 동시에 만족하지 못해 실기 실행이 차단됩니다.

실로봇 RT 드라이버가 아직 없다면 **별도 터미널**에서 아래 launch를 실행합니다. 기존 `a0509_real.launch.py`나 다른 실로봇 컨트롤러가 실행 중이면 중복 시작하지 않습니다.

```bash
ros2 launch sim_real_gap_getting real_rt.launch.py \
  real_domain_id:=0 host:=192.168.137.100 rt_host:=192.168.137.50
```

GUI의 **Check real RT topics**는 real domain에 별도 프로세스로 연결해 다음 값을 읽습니다. JSON을 불러온 상태라면 현재 관절 위치를 deg에서 rad로 변환해 경로 시작점과 비교하고, 관절 속도로 정지 상태도 확인합니다. CLI로도 확인할 수 있습니다.

GUI에서 선택한 그리퍼로 JSON을 저장한 뒤 **Check real RT topics**를 누르세요. 이어서 **Save RT report**를 누르면 선택 그리퍼와 RT 필드별 상태·수신값·경로 시작점 비교가 담긴 JSON을 같은 그리퍼 폴더에 저장합니다. 보고서의 `physical_tool_verified`는 `false`이며, 실제 장착된 그리퍼를 RT 토픽만으로 판단하지 않습니다.

```bash
ROS_DOMAIN_ID=0 ros2 run sim_real_gap_getting check_real_rt --timeout 5 --trajectory /absolute/path/trajectory.json
```

| 값 | 우선순위 | 원본 단위/의미 |
| --- | --- | --- |
| `actual_joint_position` | 필수 | deg |
| `actual_joint_velocity` | 필수 | deg/s |
| `actual_joint_torque` | 필수 검토 | Nm, 컨트롤러 추정값 |
| `actual_motor_torque` | 중요 | Nm, 전류/기어비 변환값 |
| `gravity_torque` | 중요 | Nm |
| `time_stamp` | 중요 | 컨트롤러 데이터 취득 시각 |

이 필드는 `dsr_msgs2/msg/RobotStateRt`에 정의되어 있습니다. `dsr_msgs2/msg/RobotState`에는 같은 필드가 없습니다. 기존 드라이버는 `RobotStateRt` 전체 토픽 대신 `/rt_topic/<필드명>`을 `std_msgs/msg/Float32MultiArray`로 발행합니다. 체커는 전체 `RobotStateRt` 토픽이 있으면 그것도 읽습니다. 각 토픽의 존재뿐 아니라 실제 수신, 개수(관절값 6개/시간 1개), 유한값 여부를 확인합니다. 0 토크나 0 속도는 정지 상태에서 가능한 값이므로 결측으로 처리하지 않습니다.

## 실로봇 100 Hz 실행

**Run on real (100 Hz)**는 실로봇을 실제로 움직입니다. 선택 그리퍼가 기록된 JSON을 저장하고, 15초 이내에 성공한 **Check real RT topics** 결과가 있어야 버튼이 진행됩니다. 확인 대화상자에서 실제 그리퍼·페이로드·작업공간과 비상정지 접근성을 작업자가 확인한 뒤 별도 real ROS domain 프로세스가 실행됩니다. **Stop real** 또는 GUI 종료 요청은 실행 프로세스에 중지 신호를 보내고 `motion/move_stop` 빠른 정지를 요청합니다.

기본 예제 JSON의 시작 자세도 위 연습용 Home입니다. 실로봇은 별도의 Go Home 절차로 이 자세에 도달해야 하며, Real 실행 프로세스는 RT 측정 관절각이 JSON 첫 점과 0.02 rad 이내로 일치하지 않으면 명령을 시작하지 않습니다.

실행 프로세스는 기존 드라이버의 `/rt_topic/<필드명>` 분리 토픽과 `/rt_topic/robot_state`를 필요로 합니다. `RobotStateRt` 전체 토픽만 제공하는 드라이버에서는 Real 실행을 시작하지 않습니다.

실행 프로세스는 JSON의 5차 보간 경로를 100 Hz로 재표본화하고, 각 10 ms 목표 시각에 위치·속도·가속도 명령을 발행합니다. 실로봇 보고값, `time_stamp`, STANDBY/AUTONOMOUS/REAL 상태, 시작 자세, 다른 RT 명령 발행자, 100 Hz 발행 마감 시각, 관절 추종 오차를 확인합니다. 제한은 관절별 최대 30 deg/s 및 30 deg/s², 시작 오차 0.02 rad, 추종 오차 0.08 rad입니다. 마감 시각을 3 ms 넘게 놓치거나 피드백이 0.1초 이상 끊기면 중지합니다. 실행 시도 요약은 `<선택 폴더>/<gripper>/*_real_execution_*.json`, 100 Hz 목표·관측 RT 값은 같은 이름의 `_samples.jsonl`에 기록됩니다.

이 Python/ROS 경로는 하드 실시간을 보장하지 않습니다. [Doosan 공식 RT 예제](https://doosanrobotics.github.io/doosan-robotics-api-manual/GL013302/tutorials/scenarios/6_realtime_control_sample.html)는 1 ms 주기와 실시간 스레드를 사용합니다. 따라서 실로봇에서 100 Hz 유지와 실제 궤적 추종을 검증하기 전에는 이 기능을 연구용 연습으로 취급해야 합니다. RT 토픽과 JSON 검증은 작업공간 충돌이나 실제 장착 도구를 자동으로 검증하지 않습니다.
