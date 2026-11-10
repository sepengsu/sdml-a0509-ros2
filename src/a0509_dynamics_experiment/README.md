# A0509 dynamics experiment

Doosan A0509의 동역학 데이터 수집용 ROS 2 패키지입니다. T1~T4는 초기 연구 제안값이며 **실로봇 안전 경로가 아닙니다**. `--preview`는 ROS 연결 없이 출력만 만듭니다. `--simulation`은 별도 ROS domain에서 MuJoCo를 시작하고 위치 명령을 시뮬레이터에만 보냅니다. 기본 Real Mode는 이미 실행 중인 Doosan RT 토픽을 읽고 기록하며 이동 명령을 만들지 않습니다.

## 준비

ROS 2 Humble 워크스페이스의 루트에서 실행합니다.

```bash
git submodule update --init --recursive
source /opt/ros/humble/setup.bash
# mujoco_ros2_control 빌드에 MuJoCo C++ 배포판과 MUJOCO_DIR 설정 필요
# Python 실행 환경에 mujoco, matplotlib, numpy, PyYAML 설치 필요
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

# 별도 터미널에서 RT 수집용 실로봇 환경을 명시적으로 시작한 뒤에만 실행.
# 기존 a0509_real.launch.py와 동시에 실행하면 컨트롤러가 중복되므로 하나만 사용.
ros2 launch a0509_dynamics_experiment real_rt.launch.py host:=192.168.137.100 rt_host:=192.168.137.50
ros2 run a0509_dynamics_experiment run_experiment --trajectory T1 --duration 10

# 실로봇 이동 요청은 현재 전체 경로 검증/제어 권한/작업자 승인 인터페이스가
# 충족되지 않으므로 오류 코드 3으로 차단하며, 읽기 전용 로그만 저장.
ros2 run a0509_dynamics_experiment run_experiment --trajectory T1 --execute
```

`--simulation --execute`를 함께 주면 시뮬레이션만 실행합니다. `--preview --execute` 및 `--preview --simulation`은 오류입니다. `--trajectory all`은 Preview와 Simulation에서만 허용합니다.

주요 옵션: `--reference-tcp-mm X Y Z`, `--start-mm X Y Z`, `--end-mm X Y Z`, `--speed low|medium`, `--segment-seconds`, `--repeats`, `--pre-hold`, `--post-hold`, `--sample-hz`, `--joints 1 3`, `--joint-mode sequential|simultaneous`, `--amplitude-deg`, `--gripper-type`, `--payload-mass`, `--results-dir`. T1~T3의 TCP 기준점이 생략되면 Simulation/Real에서 현재 측정 위치를 쓰고 Preview에서는 원점을 씁니다. T4는 현재 관절 자세를 시작점으로 씁니다.

Simulation의 기본 `ROS_DOMAIN_ID`는 173이며 `--sim-domain-id`로 변경할 수 있습니다. 실로봇 제어 노드가 그 domain에서 발견되면 명령을 차단합니다. `--connect-existing`은 동일한 격리 domain의 기존 MuJoCo에 연결할 때만 사용합니다. MuJoCo 위치 컨트롤러가 구독 중이지 않으면 명령을 보내지 않습니다.

## 결과와 데이터 의미

`results/<preview|simulation|real>/<T1..T4>/trial_001/`에 `trajectory.csv`, `trajectory.json`, `waypoints.csv`, `trajectory_3d.png`, `trajectory_3d.html`, `metadata.json`을 기록합니다. Simulation과 Real은 `dynamics.csv`, `dynamics_plot.png`, `rosbag/`도 기록합니다. `trial_id`와 Bag 경로는 metadata에 연결됩니다. 결과 폴더는 Git에서 제외합니다.

공통 CSV 열은 위치·속도·가속도와 목표 관절값을 rad 단위로 저장합니다. Real RT 원본 배열은 Doosan 메시지의 deg, deg/s, mm, Nm, °C 단위를 그대로 보존하고 공통 열에서만 rad로 변환합니다. 원본 `dsr_controller2`는 RT 토픽을 기본으로 발행하지 않습니다. 이 패키지의 `config/dsr_controller2_rt.yaml`은 `use_rt_topic_pub: true`, `rt_timer_ms: 10`을 설정하고 필요한 토픽을 선택합니다. 토픽은 헤더 없는 `Float32MultiArray`이므로 수신 시각 기준 30 ms 이내 데이터를 결합합니다. metadata에 관찰 주기, 추정 누락률, 빈 필드 개수, 단위와 토크 의미를 기록합니다.

`rt_time_stamp`는 컨트롤러가 제공한 값을 별도로 보존합니다. 원본 double 값이 RT 토픽에서 float32로 변환되어 정밀도가 떨어질 수 있으므로, 토픽 결합과 누락률 계산은 수신 시각을 사용합니다.

MuJoCo의 `qpos`, `qvel`, `qacc`, `qfrc_inverse`, `qfrc_bias`는 측정 관절 상태를 독립 MuJoCo 모델에 적용해 관찰·재구성합니다. `qfrc_actuator`는 현재 상위 ROS bridge가 직접 발행하지 않아 빈 값입니다. `/joint_states.effort`는 상위 bridge에서 `qfrc_applied`를 사용하므로 이를 `qfrc_actuator`로 잘못 표시하지 않습니다. 로봇의 모터 토크 환산값과 컨트롤러 추정 관절 토크도 독립 토크 센서 측정으로 표시하지 않습니다. 서로 다른 토크 필드는 의미와 단위를 검증하기 전까지 직접 비교하지 않습니다.

## 안전과 검증 상태

`--execute`는 요청 시에도 실제 이동 서비스를 호출하지 않습니다. 제어 권한, TCP/Tool/Payload 설정, 전체 경로의 충돌·특이점·작업 공간, 현재 자세와 시작점, 작업자 최종 승인에 관한 검증 체인이 구현·검증되기 전에는 이동을 열지 않습니다. 현재 구현은 이 항목을 미충족으로 보고 차단합니다.

호스트 측 시험:

```bash
PYTHONPATH=src/a0509_dynamics_experiment python3 -m pytest -q src/a0509_dynamics_experiment/tests
PYTHONPATH=src/a0509_dynamics_experiment python3 -m a0509_dynamics_experiment.run_experiment --preview --trajectory all
```

현재 개발 PC에서는 ROS 2 실행 명령과 Docker 데몬이 없어 MuJoCo ROS 2 실제 실행, Bag 기록 및 실로봇 연결 시험은 수행하지 못했습니다. 고정된 Doosan A0509 MuJoCo 모델을 별도로 열어 T1~T3의 전체 100 Hz 목표점 역기구학과 역동역학 계산은 검증했습니다.
