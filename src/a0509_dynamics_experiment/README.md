# A0509 dynamics experiment

Doosan A0509의 동역학 데이터 수집용 ROS 2 패키지입니다. T1~T14는 초기 연구 제안값이며 **실로봇 안전 경로가 아닙니다**. `--preview`는 ROS 연결 없이 출력만 만듭니다. `--simulation`은 별도 ROS domain에서 MuJoCo를 시작하고 위치 명령을 시뮬레이터에만 보냅니다. 기본 Real Mode는 이미 실행 중인 Doosan RT 토픽을 읽고 기록하며 이동 명령을 만들지 않습니다.

## 준비

ROS 2 Humble 워크스페이스의 루트에서 실행합니다.

```bash
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
# mujoco_ros2_control 빌드에 MuJoCo C++ 배포판과 MUJOCO_DIR 설정 필요
# Python 실행 환경에 mujoco, matplotlib, numpy, PyYAML 설치 필요
# --record-video 사용 시 mss, imageio-ffmpeg도 필요
# render_mdpg_replay 사용 시 imageio-ffmpeg, Pillow도 필요
colcon build --packages-up-to a0509_dynamics_experiment
source install/setup.bash
```

`src/a0509_mujoco_sim`, `src/sdml_a0509`는 이 저장소의 `sim2real` 브랜치에서 가져왔습니다. `src/mujoco_ros2_control`은 [상위 프로젝트](https://github.com/moveit/mujoco_ros2_control)를 고정된 서브모듈로 연결했습니다. 그 프로젝트의 [MuJoCo 설치 안내](https://github.com/moveit/mujoco_ros2_control#installation-guide)에 따라 C++ 헤더와 라이브러리를 설치하고 `MUJOCO_DIR`을 지정해야 합니다.

Docker를 사용하는 경우 `docker/Dockerfile.ros`는 MuJoCo 3.2.7 C++/Python 환경을 포함합니다.

```bash
docker compose --profile virtual build ros-control
docker compose --profile virtual run --rm --no-deps ros-control bash
# 컨테이너 안에서: colcon build --packages-up-to a0509_dynamics_experiment
```

## 실행

```bash
# T1~T4를 생성하고 PNG, HTML, CSV, JSON 저장. ROS/로봇에 연결하지 않음.
ros2 run a0509_dynamics_experiment run_experiment --preview --trajectory all

# MuJoCo를 자동 시작. T1~T3는 모델에서 IK가 풀린 시작 자세로 먼저 이동.
ros2 run a0509_dynamics_experiment run_experiment --simulation --trajectory T1
ros2 run a0509_dynamics_experiment run_experiment --simulation --trajectory all
ros2 run a0509_dynamics_experiment run_experiment --simulation --trajectory T1 --record-video

# AG145를 장착한 A0509로 Preview, MuJoCo 실행 및 영상 기록.
ros2 run a0509_dynamics_experiment run_experiment --preview --trajectory all --gripper-type ag145 --results-dir results/ag145
ros2 run a0509_dynamics_experiment run_experiment --simulation --trajectory all --gripper-type ag145 --results-dir results/ag145
ros2 run a0509_dynamics_experiment run_experiment --simulation --trajectory T1 --gripper-type ag145 --record-video --results-dir results/ag145

# 수집된 T1 관절 데이터를 가까운 사선 시점으로 다시 렌더링.
ros2 run a0509_dynamics_experiment render_ag145_replay results/ag145/simulation/T1/trial_001

# 별도 터미널에서 RT 수집용 실로봇 환경을 명시적으로 시작한 뒤에만 실행.
# 기존 a0509_real.launch.py와 동시에 실행하면 컨트롤러가 중복되므로 하나만 사용.
ros2 launch a0509_dynamics_experiment real_rt.launch.py host:=192.168.137.100 rt_host:=192.168.137.50
ros2 run a0509_dynamics_experiment run_experiment --trajectory T1 --duration 10

# 실로봇 이동 요청은 현재 전체 경로 검증/제어 권한/작업자 승인 인터페이스가
# 충족되지 않으므로 오류 코드 3으로 차단하며, 읽기 전용 로그만 저장.
ros2 run a0509_dynamics_experiment run_experiment --trajectory T1 --execute
```

`--simulation --execute`를 함께 주면 시뮬레이션만 실행합니다. `--preview --execute` 및 `--preview --simulation`은 오류입니다. `--trajectory all`은 Preview와 Simulation에서만 허용합니다.

주요 옵션: `--reference-tcp-mm X Y Z`, `--start-mm X Y Z`, `--end-mm X Y Z`, `--speed low|medium`, `--segment-seconds`, `--repeats`, `--pre-hold`, `--post-hold`, `--sample-hz`, `--joints 1 3`, `--joint-mode sequential|simultaneous`, `--amplitude-deg`, `--gripper-type`, `--payload-mass`, `--results-dir`. `--record-video`는 Simulation에서만 사용할 수 있으며 MuJoCo X11 화면을 MP4로 기록합니다. T1~T3의 TCP 기준점이 생략되면 Simulation/Real에서 현재 측정 위치를 쓰고 Preview에서는 원점을 씁니다. T4는 현재 관절 자세를 시작점으로 씁니다.

Simulation의 기본 `ROS_DOMAIN_ID`는 173이며 `--sim-domain-id`로 변경할 수 있습니다. 실로봇 제어 노드가 그 domain에서 발견되면 명령을 차단합니다. `--connect-existing`은 동일한 격리 domain의 기존 MuJoCo에 연결할 때만 사용합니다. MuJoCo 위치 컨트롤러가 구독 중이지 않으면 명령을 보내지 않습니다.

### AG145 모델

`--gripper-type ag145`는 저장소의 `dh_ag145_description` URDF/STL을 기존 A0509 MuJoCo 모델의 `link_6`에 결합합니다. 장착 회전은 기존 `a0509_ag145.urdf.xacro`의 45°를 따릅니다. 페이로드가 없으면 AG145는 벌린 자세, 0.5/1.0 kg 박스 페이로드가 있으면 URDF 조인트 한계 안의 **0.60 rad 고정 파지 자세**를 사용합니다. 원본 손가락 메시를 계산해 이 자세에서 양쪽 안쪽 면이 60 mm 박스의 ±30 mm 옆면과 거의 일치하도록 했습니다. 파지 동작이나 접촉력은 시뮬레이션하지 않으며 박스는 이상적으로 고정됩니다. [제조사 AG-105-145 사양](https://en.dh-robotics.com/product/ag)의 1.3 kg을 모델에 더합니다. 원본 URDF에 관성 정보가 없어서 관성·충돌 형상은 대략적인 박스로 구성했습니다. `ag145_tcp`는 플랜지에서 그리퍼 로컬 Z 방향으로 200 mm 떨어진 **임시 기준점**이며 실물 TCP 측정/보정값이 아닙니다. 이 가정은 각 trial의 `metadata.json`에 기록됩니다.

URDF의 고정축 `rpy`와 MuJoCo XML의 기본 Euler 회전 순서는 다릅니다. AG145 링크/시각 메시의 `rpy`를 쿼터니언으로 변환해 손가락과 링크가 분리돼 보이지 않도록 했습니다. 이전에 생성한 잘못된 화면 녹화본은 결과 목록에서 제외하고 수정된 T1 trial을 다시 기록했습니다.

AG145 Preview는 MuJoCo Python 패키지가 설치된 환경에서 로컬 모델만 읽습니다. T1~T3의 기준 TCP를 생략하면 모델의 도달 가능한 초기 자세에서 AG145 TCP 좌표를 사용합니다. 시뮬레이션은 같은 TCP site에 대해 IK와 경로 충돌 검사를 수행하고 MuJoCo 데이터에 그리퍼 질량을 반영합니다. `--connect-existing`과 AG145는 함께 사용할 수 없습니다. 대형 STL을 MuJoCo 화면에서 처리하기 위해 메시를 축소합니다. Docker 이미지에는 `fast-simplification`이 포함되고, 일반 Python 환경에서는 NumPy 기반 축소가 사용됩니다.

`--record-video`는 MuJoCo 화면을 `simulation.mp4`에 저장합니다. `render_ag145_replay`는 AG145 trial의 `dynamics.csv`에 저장된 MuJoCo 관절 위치를 재생하여 `simulation_replay.mp4`를 만듭니다. 두 영상 모두 시뮬레이션이며, 재생 영상은 실제 촬영물이 아닙니다. 재생 영상의 카메라 설정과 데이터 출처는 metadata에 기록됩니다. Docker 안에서는 `DISPLAY`와 X 서버가 필요합니다.

이 모델과 예시 궤적은 실로봇 AG145 설치, 손가락 구동, Payload/TCP 설정 또는 안전 경로를 검증한 결과가 아닙니다. Real Mode에서 AG145를 지정해도 기존 읽기 전용 로깅 및 이동 차단 정책이 적용됩니다.

### mDPG_1S1 공압 흡착 그리퍼

첨부된 `mdpg_1s1_description`의 STL/Xacro를 A0509 `link_6`에 고정 연결했습니다. 활성 URDF는 `sdml_a0509_description/urdf/a0509_with_gripper.urdf.xacro`이며, `gripper_type:=ag145|mdpg_1s1|none`으로 하나만 선택합니다. MuJoCo 기본 launch도 같은 값을 받아 선택된 시각 모델을 장착합니다.

공압 그리퍼의 장착 위치·회전은 제공된 STEP으로 확인할 수 없으므로 `mdpg_mount_xyz`(m)와 `mdpg_mount_rpy`(rad)를 명시해야 합니다. 누락하면 launch가 거부합니다. `mdpg_tcp`의 장착 프레임 기준 +90 mm는 CAD 외곽 기준 잠정 값이며 보정된 실제 TCP가 아닙니다. 기본 `visual` 모드는 질량 없이 메시만 장착합니다. `--mdpg-dynamics-model box_approx`를 명시하면 사용자가 제공한 제품 이미지의 그리퍼 질량 0.62 kg과 65 × 70 × 81 mm 균일 박스 가정에서 계산한 관성을 MuJoCo/URDF에 추가합니다. 중심질량과 관성은 **실측값이 아닙니다**. 이미지의 1.5 kg은 최대 허용 페이로드이며 현재 장착된 하중이 아닙니다. CAD 빨판 끝단은 약 90 mm로 이미지의 전체 높이 81 mm와 다르며, 두 자료가 동일한 조립체인지 독립 확인하지 못했습니다. 접촉·흡착력·공압 IO·어댑터 질량은 구현되지 않았고 TODO로 남습니다.

```bash
# 측정된 link_6 장착 변환으로 치환할 것. 숫자 0을 실제 장착값으로 가정하지 말 것.
ros2 launch sdml_a0509_bringup rviz_only.launch.py gripper_type:=mdpg_1s1 mdpg_mount_xyz:="<X Y Z>" mdpg_mount_rpy:="<R P Y>"
ros2 launch a0509_mujoco_sim mujoco_sim.launch.py gripper_type:=mdpg_1s1 mdpg_mount_xyz:="<X Y Z>" mdpg_mount_rpy:="<R P Y>"
ros2 run a0509_dynamics_experiment run_experiment --preview --trajectory T1 --gripper-type mdpg_1s1 --mdpg-mount-xyz X Y Z --mdpg-mount-rpy R P Y

# 명시적 근사 동역학 T1. 아래 0 0 0은 미측정 시험 장착값이며 실물 설정값이 아닙니다.
ros2 run a0509_dynamics_experiment run_experiment --simulation --trajectory T1 \
  --gripper-type mdpg_1s1 --mdpg-dynamics-model box_approx \
  --mdpg-mount-xyz 0 0 0 --mdpg-mount-rpy 0 0 0 \
  --results-dir results/mdpg_box_approx
```

실로봇의 `a0509_real.launch.py gripper_type:=mdpg_1s1`은 AG145 구동/피킹 노드 없이 RT 상태 발행과 RViz를 사용합니다. 실제 접속은 검증하지 않았습니다. mDPG 시뮬레이션 동역학 기록은 `box_approx` 선택 시에만 가능하고 결과 메타데이터에 근사 모델을 표시합니다. 이 합성 데이터는 실측 PKAN 정답 토크로 사용하면 안 됩니다. 실로봇 `--execute`는 장착/안전 검증과 공압 IO가 없어 차단합니다.

2026-11-10 Docker MuJoCo T1 시험에서 `results/mdpg_box_approx/simulation/T1/trial_001/`에 궤적·동역학 CSV, PNG/HTML, `mdpg_scene.xml`, ROS Bag과 메타데이터를 저장했습니다. 1001개 동역학 행을 기록했고 `/joint_states` 관찰 주기는 시뮬레이션 시간 기준 약 99.99 Hz, 추정 누락률은 0%였습니다. 전체 모델 질량은 기본 팔 대비 0.62 kg 증가했습니다. 이 시험의 `0 0 0` 장착값은 **미측정 테스트 픽스처**이며 실물 장착이나 토크 정확도 검증 결과가 아닙니다.

장착 위치가 미측정인 상태에서 **영상만** 필요하면, T1을 그리퍼 없이 MuJoCo에서 실행한 관절 기록에 mDPG 시각 메시를 덧씌워 재생할 수 있습니다. A0509 `A0509_6_0.stl`은 `link_6` 로컬 Z=-66~0 mm이고 끝 장착면이 Z=0입니다. mDPG CAD는 빨간 덮개의 반대편 장착면이 Z=0, 빨판이 -90 mm이므로, **CAD 메시 자체를 X축으로 180° 회전**해 빨간 면을 J6 끝에 두고 빨판을 바깥쪽(+Z)으로 향하게 했습니다. 다음의 `0 0 0`은 실물 장착값이 아닌 **시각화 테스트 기준**이며 실제 어댑터, TCP, 질량, 흡착 동역학을 검증하지 않습니다. 재생 영상은 별도 `mode=visual_replay` 메타데이터와 함께 저장되며 새 그리퍼 동역학 데이터로 사용하면 안 됩니다.

```bash
ros2 run a0509_dynamics_experiment run_experiment --simulation --trajectory T1 \
  --gripper-type none --results-dir results/mdpg_visual_source
ros2 run a0509_dynamics_experiment render_mdpg_replay \
  results/mdpg_visual_source/simulation/T1/trial_001 results/mdpg_visual/T1/trial_001 \
  --mount-xyz 0 0 0 --mount-rpy 0 0 0 --visual-fixture
```

영상 출력은 `mdpg_t1_visual_replay.mp4`, 대표 화면은 `thumbnail.png`입니다. `metadata.json`에는 원본 T1 CSV 해시, 그리퍼를 제외한 원본 시뮬레이션 조건, 미검증 장착 상태가 기록됩니다. 실측 장착 변환이 확보되면 `--mount-xyz`와 `--mount-rpy`를 해당 측정값으로 바꿔 재렌더링할 수 있습니다.
이번 T1 실행의 [영상](../../docs/media/mdpg_t1_visual/mdpg_t1_visual_replay.mp4), [대표 화면](../../docs/media/mdpg_t1_visual/thumbnail.png), [출처 메타데이터](../../docs/media/mdpg_t1_visual/metadata.json)도 저장소에 보존했습니다.

이 실행의 [궤적 추종·q–dq–ddq·토크·품질 분석](../../docs/media/mdpg_t1_visual/analysis/README.md)에는 그래프 7개, 관절별 CSV와 기계 판독용 JSON이 있습니다. 원본은 그리퍼 질량이 없는 팔 MuJoCo 실행이므로, 토크 그래프는 mDPG 부착 후 토크 측정으로 해석하지 않습니다.

## 결과와 데이터 의미

### 7조건 × T1~T14 × 3회 재수집

`scripts/play_7_condition_dataset.ps1`은 bare A0509 무부하 1조건, AG145 0/0.5/1.0 kg 3조건, mDPG 0/0.5/1.0 kg 3조건을 T1~T14에서 각각 3회 실행하도록 294개 trial을 계획합니다. 기본 실행은 **계획 CSV만 생성**합니다. `-Play`를 명시해야 격리된 Docker MuJoCo에서 순차 수집을 시작합니다. 기존 결과를 덮어쓰지 않고 `results/recollection_7conditions/`에 조건·반복별로 저장하며, 성공한 trial은 재실행 시 건너뜁니다. 실로봇 명령은 사용하지 않습니다.

7개 조건의 모델 사진은 `python3 src/a0509_dynamics_experiment/scripts/render_7_condition_gallery.py`를 Docker의 `/ros2_ws`에서 실행해 `results/condition_gallery/`에 만듭니다. MuJoCo가 실제로 로드하는 로봇·그리퍼·페이로드 장면을 같은 관절 자세와 카메라로 렌더링합니다. 사진은 모델 비교용 고정 자세이며 궤적 실행이나 수집된 trial의 특정 시각을 뜻하지 않습니다. AG145 페이로드 사진은 실행 모델과 같은 고정 파지 자세를 사용합니다. `manifest.json`에 장면 해시, 자세, 페이로드 설정 해시와 mDPG 장착 가정을 기록합니다.

0.5/1.0 kg 조건은 사용자가 허용한 **시뮬레이션 전용 균일 직사각형 박스 근사**로 설정했습니다. 두 질량 모두 박스 크기 60 × 40 × 40 mm를 쓰고, 중심질량은 박스 중심, 관성은 `Ixx=m(y²+z²)/12`, `Iyy=m(x²+z²)/12`, `Izz=m(x²+y²)/12`로 계산합니다. AG145 박스 중심은 그리퍼 기준 `z=195 mm`로 두어 0.60 rad 고정 손가락 자세가 박스 양옆을 감싸도록 가정합니다. 1.0 kg은 [AG-105-145 제조사 권장 작업물 질량 2 kg](https://en.dh-robotics.com/wp-content/uploads/2023/05/Servo-Electric-Gripper_DH-Robotics_EN2303.pdf)보다 작습니다. mDPG 박스는 명목 흡착 TCP `z=90 mm` 앞에 놓아 중심을 `z=110 mm`로 가정했습니다. 1.0 kg은 사용자 제공 사양 이미지의 최대 페이로드 1.5 kg보다 작습니다. 이 치수·부착 위치·COM은 **실측값이 아니며 실제 파지/흡착 또는 동적 안전성을 보증하지 않습니다.**

이 가정은 `config/payload_conditions.json`의 네 항목과 각 trial metadata에 기록됩니다. `mount_xyz_m`/`mount_rpy_rad`는 해당 그리퍼 MuJoCo 본체(`ag145_base_link` 또는 `mdpg_base_link`)에서 박스 중심까지의 변환이고, `com_xyz_m`은 그 중심에서 질량중심까지입니다. `inertia_model: uniform_box`와 `box_size_xyz_m`에서 관성텐서를 계산합니다. 직접 측정한 텐서를 사용하려면 `inertia_model`/`box_size_xyz_m`을 제거하고 `inertia_kg_m2`를 `[ixx, iyy, izz, ixy, ixz, iyz]` 순서로 넣습니다. 단위는 m, rad, kg·m²입니다. 설정이 비어 있거나 관성텐서가 물리적으로 유효하지 않으면 첫 trial 전에 차단합니다. **mDPG의 `link_6` 장착 변환 실측값은 여전히 없습니다.**

MuJoCo에서는 박스의 질량·관성이 토크 계산에 반영되고 반투명 박스로 보입니다. 박스의 충돌, 실제 파지력·진공 유지·이탈은 모델링하지 않았습니다. 따라서 이 수집은 **이상적인 고정 부착 시뮬레이션 데이터**이며 실제 작업물을 집어 올리는 실험이나 안전한 실로봇 경로 검증으로 해석할 수 없습니다. mDPG 자체의 0.62 kg 박스 관성과 장착/TCP 역시 아직 검증되지 않았습니다.

```powershell
# 저장소 루트에서 계획 294건 생성. 데이터 수집은 시작하지 않음.
./scripts/play_7_condition_dataset.ps1

# 기존 영상과 같은 CAD 빨간 장착면 중심을 link_6 원점에 둔 시뮬레이션 가정으로 실행.
./scripts/play_7_condition_dataset.ps1 -Play -UseCadFaceCenterFixture

# 실측 장착 변환이 생긴 경우에는 별도로 입력해 실행.
./scripts/play_7_condition_dataset.ps1 -Play -MdpgMountXyz @($xM,$yM,$zM) -MdpgMountRpy @($roll,$pitch,$yaw)
```

`-UseCadFaceCenterFixture`는 빨간 장착면 중심을 J6 끝에 일치시키는 영상/CAD 기준 `(0,0,0)` 변환을 **실측이 아닌 시뮬레이션 가정으로** 선택합니다. CAD 메시 자체의 X축 180° 회전은 모델 내부에 이미 적용돼 있으므로 장착 `rpy`에 다시 더하지 않습니다. 선택 근거와 변환값은 결과 루트의 `mount_declaration.json`에 보존되며, 각 trial metadata의 `mount_verified`는 false입니다. 실측 변환을 넣어 재실행하면 기존 결과의 변환과 비교해 달라진 조건을 완료된 trial로 건너뛰지 않습니다. 실로봇 장착·TCP·공압 IO 검증에는 사용할 수 없습니다.

#### 병렬 MuJoCo 수집

`scripts/play_7_condition_dataset_parallel.ps1`은 `-RobotCount`만큼 독립적인 **시뮬레이션 로봇 컨테이너**를 실행합니다. 각 컨테이너에 별도 `ROS_DOMAIN_ID`를 주고, `(조건, 반복)` 단위로 14개 궤적을 한 작업자에게 배정합니다. 동일 trial을 두 작업자가 동시에 쓰지 않으며, 완료 메타데이터·CSV·Bag·모델 조건을 검증한 뒤 이어서 수집합니다. 기존 `results/recollection_7conditions_grasp_v2/`의 완료 trial도 재사용합니다. 컨테이너는 분리되어 있지만 설치된 ROS 패키지는 읽기 전용으로 공유합니다. 스크립트는 첫 실행 전 기존 `a0509-dynamics`에서 한 번 빌드합니다.

```powershell
# 계획만 출력
.\scripts\play_7_condition_dataset_parallel.ps1 -RobotCount 4

# 이미 수집한 결과를 건너뛰고, 독립 MuJoCo 로봇 4대로 재개
.\scripts\play_7_condition_dataset_parallel.ps1 -Play -RobotCount 4 -UseCadFaceCenterFixture -ResultsName recollection_7conditions_grasp_v2

# 진행 상태와 컨테이너 상태 확인
.\scripts\play_7_condition_dataset_parallel.ps1 -Status -ResultsName recollection_7conditions_grasp_v2
```

작업자는 Docker에서 백그라운드로 실행되며 로그는 `results/<ResultsName>/parallel_runs/<run_id>/worker_XX.log`에 저장합니다. 중단 후 같은 `-Play` 명령을 다시 실행하면 성공한 trial을 건너뛰고 새 작업자를 시작합니다. 같은 결과 폴더에서 기존 순차 수집과 병렬 수집을 동시에 실행하면 안 됩니다. 기본값은 영상 기록을 생략합니다. `-RecordVideo`를 명시하면 모든 trial의 영상을 기록하므로 수집이 느려지고, 기존 영상 없는 trial도 재실행합니다. `-RobotCount`는 1~8을 지원하지만 16 GB Docker 메모리 환경에서는 2~4부터 시작해 CPU·메모리 사용률을 확인하세요.

순차·병렬 시작 스크립트는 공통 잠금을 사용하고, 이미 실행 중인 병렬 컨테이너가 있으면 새 수집을 차단합니다. 작업자별 ROS 도메인과 Docker 네트워크 공간은 분리되며, 빌드·설치 볼륨과 소스는 작업자에서 읽기 전용입니다. 결과는 `(조건, 반복)`별 고유 폴더에 씁니다. 이것은 파일·ROS 충돌 방지이며 Docker 메모리 부족이나 호스트 재시작까지 막지는 못합니다. 현재 Xvfb/GLFW 경로는 `LIBGL_ALWAYS_SOFTWARE=1`로 CPU의 Mesa llvmpipe를 사용합니다. NVIDIA 장치가 Docker에 보이더라도 `--gpus all`만으로 이 화면 렌더러가 GPU로 바뀌지 않습니다.

병렬 수집은 각 trial의 100 Hz 원자료, ROS Bag, 최종 궤적·동역학 그래프를 유지합니다. 실행 전 임시 궤적 그래프 중복 생성과 T5~T14 매 trial의 전체 요약을 제거하고, 요약·품질 분석은 `(조건, 반복)`의 14개 궤적을 모두 기록한 뒤 한 번 실행합니다. 모든 작업은 `--simulation`만 사용하며 실로봇 연결이나 이동은 하지 않습니다.

### Dynamics10: 신규 관절 여기 T5~T14

T5~T14는 AG145 고정 개방 자세의 MuJoCo 시뮬레이션 데이터 확장을 위한 관절 궤적입니다. T5는 J2/J3 교대, T6은 J1/J4 결합, T7은 J5/J6 손목, T8은 J1/J3/J5, T9는 J2/J4/J6, T10은 6축 동시 multi-sine입니다. T11은 J2/J3/J5 chirp, T12는 다른 관절 J1/J4/J6 chirp입니다. T13과 T14는 각각 다른 시작 자세에서 다관절 또는 순차·동시 운동을 결합합니다. 각 축의 진폭·주파수·위상·운동 구간은 `trajectory_generator.EXCITATIONS`와 개별 `metadata.json`에 기록합니다. 시작/종료에는 C2 quintic envelope를 쓰고 위치·속도·가속도를 같은 해석식에서 계산합니다.

분할은 궤적과 trial 단위입니다: **TRAIN T5~T11**, **VALIDATION T12**, **TEST T13~T14**. 기존 T4는 진단용이며 최종 독립 시험으로 간주하지 않습니다. T1~T3의 기존 학습 데이터와 신규 데이터는 자동 병합하거나 PKAN을 재학습하지 않습니다. 새 데이터는 AG145의 기존 1.3 kg·근사 관성·임시 TCP를 유지합니다. mDPG-1S1 데이터는 명시적 `box_approx` 선택 시에만 별도 실험 폴더로 수집하며, AG145 PKAN 데이터셋에 자동 병합하지 않습니다.

```bash
# 저장소 루트, ROS 2 Humble과 install/setup.bash를 source한 후
ros2 run a0509_dynamics_experiment run_experiment --preview --trajectory-set dynamics10 --gripper-type ag145 --results-dir results/dynamics10
ros2 run a0509_dynamics_experiment run_experiment --simulation --trajectory-set dynamics10 --gripper-type ag145 --sample-hz 100 --results-dir results/dynamics10
ros2 run a0509_dynamics_experiment run_experiment --simulation --trajectory T10 --gripper-type ag145 --results-dir results/dynamics10
```

Preview는 관절 위치·속도·가속도와 AG145 MuJoCo 접촉을 모든 100 Hz 계획 샘플에서 검사하지만 ROS 명령을 보내지 않습니다. Simulation은 별도 ROS domain에서 계획을 재검사하고, 다른 시작 자세로 이동할 때 6초 quintic 시뮬레이션 전용 준비 이동 후 도달 여부를 확인합니다. T5~T14의 Real Mode는 CLI에서 거부합니다. 각 simulation trial은 ROS Bag, 목표/실제 관절 상태, 별도 MuJoCo 모델에서 재구성한 역동역학, 6×6 질량행렬, bias, damping을 기록합니다. Bridge가 직접 제공하지 않는 `qfrc_actuator`는 비워 둡니다. 가속도는 최근 5개의 서로 다른 ROS 시뮬레이션 시각에 기록된 속도에 대한 최소제곱 기울기이며, 3개 미만이면 비웁니다. 역동역학 토크는 물리 센서 실측값이 아닙니다.

결과는 `results/dynamics10/preview/T5..T14/`, `results/dynamics10/simulation/T5..T14/trial_NNN/`에 저장됩니다. 각 trial에 `trajectory.csv`, `trajectory.json`, `dynamics.csv`, `metadata.json`, `trajectory_3d.png`, `dynamics_plot.png`, `rosbag/`를 둡니다. 루트의 `dataset_manifest.json`과 `dataset_summary.csv`는 성공·실패, 유효 샘플, 누락·비유한 값, 시뮬레이션 시각·샘플 주기, 목표 추종, 관절 범위 및 기존 T1~T3 대비 **관절별 1차원 범위**를 기록합니다. `figures/`에는 계획 여기 10개, 위치·속도·가속도 범위, 속도·가속도 RMS, 토크 프로파일을 저장합니다. 1차원 범위 확대만으로 전체 18차원 상태공간의 충분한 탐색을 주장하지 않습니다. 품질 검사를 통과한 시뮬레이션 trial만 수집 완료로 계산합니다.

배치 실행은 각 궤적을 독립적으로 시도하고, 실패 원인을 출력한 뒤 다음 궤적으로 넘어갑니다. 새 trial 폴더를 만들므로 이전 T1~T4 결과와 체크포인트를 덮어쓰지 않습니다. 동일 궤적을 다시 실행하면 `trial_002`처럼 번호가 증가하며, 매니페스트는 최신 metadata가 있는 trial을 선택합니다.

2026-11-10 AG145 MuJoCo 실행에서 Preview 10개, Simulation 10개와 ROS Bag 10개를 만들었고 최신 simulation trial은 모두 품질 검사를 통과했습니다. 전체 유효 행은 **15,710개**이며 TRAIN 10,807, VALIDATION 1,601, TEST 3,302행입니다. 행들은 시계열 샘플이며 독립 trial은 궤적당 1회입니다.

| ID | 분할 | 최신 trial | 유효 행 | 속도 최대 (rad/s) | 가속도 최대 (rad/s²) | 역동역학 토크 최대 (Nm) |
|---|---|---|---:|---:|---:|---:|
| T5 | Train | trial_003 | 1,401 | 0.114 | 0.422 | 110.96 |
| T6 | Train | trial_001 | 1,401 | 0.135 | 0.531 | 83.05 |
| T7 | Train | trial_001 | 1,401 | 0.156 | 0.663 | 95.62 |
| T8 | Train | trial_001 | 1,601 | 0.124 | 0.501 | 97.84 |
| T9 | Train | trial_001 | 1,601 | 0.133 | 0.513 | 79.60 |
| T10 | Train | trial_001 | 1,801 | 0.113 | 0.395 | 81.74 |
| T11 | Train | trial_001 | 1,601 | 0.106 | 0.314 | 106.69 |
| T12 | Validation | trial_001 | 1,601 | 0.126 | 0.511 | 84.73 |
| T13 | Test | trial_001 | 1,601 | 0.115 | 0.433 | 73.34 |
| T14 | Test | trial_001 | 1,701 | 0.122 | 0.485 | 87.96 |

표의 최대값은 6축 중 최대 절대값입니다. 관절별 위치 최소/최대, 속도·가속도·토크 RMS 및 최대, 최대 목표 추종 오차는 `dataset_summary.csv`와 각 trial `metadata.json`의 `data_quality`에 있습니다. `/joint_states`의 시뮬레이션 시각 기준 관찰 주기는 전 trial에서 약 99.98 Hz, 추정 누락률은 0%였습니다. 최대 목표 추종 오차는 0.0461 rad, 기록 행의 시각은 모두 증가했고 측정 관절 제한 위반은 없었습니다. 처음 실행한 T5 `trial_001`은 기록 시각 중복으로 품질 검사에 실패해 보존했고, 시각 단조 증가를 보장한 뒤 재실행한 `trial_002`와 배치의 `trial_003`은 통과했습니다.

기존 AG145 T1~T3와 비교하면 신규 데이터의 위치 범위는 J1 아래쪽 0.206 rad, J2 아래쪽 0.171 rad, J4 아래쪽 0.241 rad 등으로 확장됐습니다. J3/J5의 속도 범위도 일부 확장됐습니다. 5행 간격 `q,dq` 12차원 표준화 최근접 거리의 중앙값은 1.439, 95백분위는 2.914입니다. 계획 관절각의 가장 가까운 두 궤적은 T5/T8이며 6축 RMS 거리는 0.033 rad로, 0.01 rad 미만 근접 중복은 없었습니다. 이 수치는 **전체 18차원 Coverage의 비율이나 독립 표본 증가율이 아닙니다.** 기존 T1~T3의 가속도는 벽시계 차분, 신규 데이터는 시뮬레이션 시각 필터에서 얻었으므로 가속도 범위는 직접 비교하지 않습니다.

재현 그래프는 `results/dynamics10/figures/trajectory_overview.png`, `joint_excitation_coverage.png`, `velocity_acceleration_distribution.png`, `torque_profiles.png`입니다. `qfrc_inverse`와 토크 그래프는 **별도 MuJoCo 모델의 재구성 요구 토크**이며 실측 토크나 bridge의 `qfrc_actuator`가 아닙니다. 이 데이터로 PKAN을 학습하지 않았습니다.

`results/<preview|simulation|real>/<T1..T4>/trial_001/`에 `trajectory.csv`, `trajectory.json`, `waypoints.csv`, `trajectory_3d.png`, `trajectory_3d.html`, `metadata.json`을 기록합니다. Simulation과 Real은 `dynamics.csv`, `dynamics_plot.png`, `rosbag/`도 기록합니다. `--record-video`를 지정하면 `simulation.mp4`가 추가됩니다. `trial_id`와 Bag 경로는 metadata에 연결됩니다. 결과 폴더는 Git에서 제외합니다.

공통 CSV 열은 위치·속도·가속도와 목표 관절값을 rad 단위로 저장합니다. Real RT 원본 배열은 Doosan 메시지의 deg, deg/s, mm, Nm, °C 단위를 그대로 보존하고 공통 열에서만 rad로 변환합니다. 원본 `dsr_controller2`는 RT 토픽을 기본으로 발행하지 않습니다. 이 패키지의 `config/dsr_controller2_rt.yaml`은 `use_rt_topic_pub: true`, `rt_timer_ms: 10`을 설정하고 필요한 토픽을 선택합니다. 토픽은 헤더 없는 `Float32MultiArray`이므로 수신 시각 기준 30 ms 이내 데이터를 결합합니다. metadata에 관찰 주기, 추정 누락률, 빈 필드 개수, 단위와 토크 의미를 기록합니다.

`rt_time_stamp`는 컨트롤러가 제공한 값을 별도로 보존합니다. 원본 double 값이 RT 토픽에서 float32로 변환되어 정밀도가 떨어질 수 있으므로, 토픽 결합과 누락률 계산은 수신 시각을 사용합니다.

MuJoCo의 `qpos`, `qvel`, `qacc`, `qfrc_inverse`, `qfrc_bias`는 측정 관절 상태를 독립 MuJoCo 모델에 적용해 관찰·재구성합니다. `qfrc_actuator`는 현재 상위 ROS bridge가 직접 발행하지 않아 빈 값입니다. `/joint_states.effort`는 상위 bridge에서 `qfrc_applied`를 사용하므로 이를 `qfrc_actuator`로 잘못 표시하지 않습니다. 로봇의 모터 토크 환산값과 컨트롤러 추정 관절 토크도 독립 토크 센서 측정으로 표시하지 않습니다. 서로 다른 토크 필드는 의미와 단위를 검증하기 전까지 직접 비교하지 않습니다.

Simulation의 샘플링 품질은 `JointState.header.stamp`의 시뮬레이션 시간으로 계산합니다. MuJoCo가 실제 시간보다 느리게 재생될 수 있으므로 수신 벽시계 주기는 `wall_arrival_hz`로 별도 보존하고 데이터 누락률로 사용하지 않습니다.

## 한 MuJoCo 장면에서 4×4 수집

Windows PowerShell에서 아래처럼 실행합니다. `-Pilot`는 2×2에 bare, AG145+0.5 kg,
mDPG, mDPG+1.0 kg을 배치해 T5를 검증합니다. 추종 오차가 0.05 rad를 넘거나,
토크 제한을 넘거나, 충돌이 있으면 `-Play`는 시작하지 않습니다. `-Play`는
7조건 × T1~T14 × 3회 = 294 trial을 16대, 이어서 5대의 두 batch로 수집합니다.

```powershell
.\scripts\play_grid_dataset.ps1 -Pilot -UseCadFaceCenterFixture
docker logs -f a0509-grid-grid_4x4_7conditions
.\scripts\play_grid_dataset.ps1 -Status
.\scripts\play_grid_dataset.ps1 -Play -UseCadFaceCenterFixture
docker logs -f a0509-grid-grid_4x4_7conditions
.\scripts\play_grid_dataset.ps1 -Status
```

`-UseCadFaceCenterFixture`는 mDPG 빨간 CAD 면 중심을 `link_6` 원점에 놓는
시뮬레이션 가정입니다. 실측 장착 변환이 있으면 `-MdpgMountXyz x,y,z`
`-MdpgMountRpy r,p,y`를 대신 제공합니다. 조건과 payload 명세의 해시가
기존 결과와 다르면 같은 결과 폴더를 재사용하지 않습니다. 한 궤적의 4×4
smoke test는 `-Play -Trajectory T1 -MaxBatches 1`로 제한할 수 있습니다.

장면의 2.5 m 간격은 시뮬레이션 배치값이며 접촉은 실행 중 검사합니다.
로봇마다 관절·액추에이터·툴·페이로드를 복제하고 한 `/clock`을 씁니다.
ROS Bag은 `/clock`, 접두사가 붙은 전체 `/joint_states`, `/target_joint_states`를
오프라인으로 기록합니다. 각 trial의 `metadata.json`에서 공통 Bag 경로와
lane을 연결합니다. 궤적별 시작 자세는 직접 초기화합니다. AG145 자세와
mDPG 흡착 상태는 고정이며 실제 집기/진공/공압 IO는 모델링하지 않습니다.

이 경로는 기존 ROS bridge 경로와 **다른 직접 MuJoCo 제어기**를 사용합니다.
역동역학 feedforward와 관절 PD를 합쳐 MJCF 토크 제한으로 포화시키며,
`qfrc_actuator`와 `qfrc_applied`를 구분해 기록합니다. 두 경로의 데이터는
제어 및 물리량 정의를 검증하기 전 자동으로 합치지 않습니다. 헤드리스
실행이어서 GPU 렌더링을 사용하지 않습니다. 한 장면도 16대의 물리 연산은
수행하므로 16배 속도 향상을 보장하지 않습니다. 대신 ROS·컨테이너 시작,
소프트웨어 렌더링 및 프로세스 간 통신 비용을 줄입니다.

원래 MJCF의 관절 감쇠와 토크 제한에서는 일부 궤적의 요구 토크가 한계를
넘습니다. 4×4 경로는 동일한 기하 경로를 느리게 재생합니다. `time_scale`은
T1 1.0, T2 3.0, T3 1.5, T4/T5 1.25, T6 2.25, T7 2.75, T8 2.5,
T9/T10 2.0, T11 1.75, T12 2.25, T13 2.0, T14 2.5입니다. 실제 100 Hz
궤적의 속도·가속도와 `metadata.json`에 이를 기록합니다. 따라서 한 batch의
계획 시뮬레이션 시간만 약 610초이고 두 batch 외에 모델 준비, 그래프,
Bag 기록 시간이 추가됩니다. 원본 ROS bridge 데이터와 시간 이력이 달라
자동 병합할 수 없습니다.

결과는 `results/grid_4x4_7conditions/simulation/T1/<condition>/trial_001/`와
`bags/batch_00/T1/...` 등에 저장합니다. 실패한 궤적은 완료 metadata가
없어 재실행 시 다시 시도합니다. `-Status`는 294개 중 완료 수를 표시합니다.

## 안전과 검증 상태

`--execute`는 요청 시에도 실제 이동 서비스를 호출하지 않습니다. 제어 권한, TCP/Tool/Payload 설정, 전체 경로의 충돌·특이점·작업 공간, 현재 자세와 시작점, 작업자 최종 승인에 관한 검증 체인이 구현·검증되기 전에는 이동을 열지 않습니다. 현재 구현은 이 항목을 미충족으로 보고 차단합니다.

호스트 측 시험:

```bash
PYTHONPATH=src/a0509_dynamics_experiment python3 -m pytest -q src/a0509_dynamics_experiment/tests
PYTHONPATH=src/a0509_dynamics_experiment python3 -m a0509_dynamics_experiment.run_experiment --preview --trajectory all
```

Docker의 ROS 2 Humble 환경에서 MuJoCo 실행과 Bag 기록을 수행했습니다. 이 결과는 시뮬레이션에 한정되며 실로봇 연결 시험은 수행하지 않았습니다. 상위 `mujoco_ros2_control`은 종료 신호를 받으면 컨트롤러 매니저 executor 관련 예외를 출력하지만, Bag과 실험 결과 저장은 그 전에 완료됩니다.
