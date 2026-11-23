# A0509 15개 마찰 조건 — 독립 MuJoCo 시뮬레이션 v4

독립 패키지 `a0509_friction_sim`이 MuJoCo 물리 엔진만 실행한다. 기본 실행기는 `scripts/run_friction_simulation.ps1`이며 ROS2, Docker, 실로봇 연결이 필요하지 않다. 잘못 넣었던 sim-to-real 패키지의 마찰 연결은 제거했다. 기존 모델, H:의 원본 데이터, 기존 결과는 보존하고 `friction_sim_v4`라는 새 출력에만 저장한다.

## 모델 분석과 구현 순서

1. 원본 모델·감쇠·토크 통계와 출처를 확인하고 계수를 고정한다.
2. 기존 336청크의 관절 상태·seed·원본 마찰 이력을 재현한다.
3. 같은 q/dq/ddq에서 15조건 inverse를 비교하는 Offline 단계를 통과한다.
4. 마찰을 물리 엔진에 적용해 실제 `mj_step`으로 pilot을 수행한다.
5. 같은 목표와 제어기를 유지한 T1–T16 전체 길이 Dynamic 수집을 수행한다.

원본은 Doosan `dsr_description2/mujoco_models/a0509/a0509.xml`이다. 고정 서브모듈은 `c3c8c6122eddb26b69b190bb8de00e2a14f48940`이며, damping은 2025-07-04의 모델 추가 커밋 `7e6d640ea32d87b2f0f8f679b705762338662b40`에서 도입되었다. 실측 보정 자료는 확인하지 못했다.

|축|D₀ (Nm·s/rad)|모델 최대 모터 토크 (Nm)|기존 감쇠 RMS (Nm)|기존 RBD RMS (Nm)|기존 모터 RMS (Nm)|
|---|---:|---:|---:|---:|---:|
|J1|800|194|41.86980|0.01961|41.86975|
|J2|800|194|37.75301|32.55980|49.88308|
|J3|800|163|27.14861|13.17964|30.14731|
|J4|600|50|14.43624|1.29026|14.46137|
|J5|600|50|13.92366|2.00176|14.06575|
|J6|600|50|13.09172|0.00274|13.09172|

표는 기존 채택된 111회, 3,742,051상태의 스냅샷이다. RBD는 기존 저장 inverse에서 D₀·dq를 빼서 복원하므로 float32 정밀도의 영향을 받는다. 기존 채택 파일은 포화가 발생한 시도를 제외했으므로 관측 포화 비율 0을 전체 시도의 결과로 일반화하지 않는다. `--mode analyze`가 축별 최대값·RMS·모델 토크 제한·제한 근접 비율을 CSV와 JSON으로 다시 기록한다.

J4–J6은 감쇠 보상만으로 약 4.775 deg/s에서 모델의 50 Nm 제한에 도달한다. 현재 모델의 속도를 강하게 제한한다는 근거는 있지만, 실제 A0509 대비 과대하다고 판단할 실측 근거는 부족하다. **800/600을 유지하며 임의로 축소하지 않는다.**

출처: [고정 원본 모델](https://github.com/DoosanRobotics/doosan-robot2/blob/c3c8c6122eddb26b69b190bb8de00e2a14f48940/dsr_description2/mujoco_models/a0509/a0509.xml), [MuJoCo 3.2.7 계산 문서](https://mujoco.readthedocs.io/en/3.2.7/computation/index.html).

## 기존 데이터와 15개 계수

입력 manifest는 `H:/a0509-data/training_datasets_2026-11-22/data/friction_diverse_v1/manifest.json`이다. 7조건×16궤적×3변형 =336청크, 1,317,456상태다. 기존 15개는 동일한 상태에 붙인 추가 마찰 토크 정답이다.

원본 `pkan-robot-modeling/src/model/collect_friction.py`를 확보했고, `a0509_friction_sim/reference_original.py`에 원문을 보관했다. 복원 검사는 이 파일에서 필요한 4개 함수만 AST로 추출하며 원본 전체 모듈의 실행 진입점은 호출하지 않는다. 원문 SHA-256은 `ba62c3b72c1efbc6a3b3450e83eb6f88bbc641ec11475ff00d6640ec8878ae61`, 정렬·축약 JSON config SHA-256은 `22c19b09b4d1c490eb979a3e6fa8177aa0ff62e91ad89edcc144216ccffd6aad`다. Git에는 원문 바이트를 보존하도록 지정했다.

|계수|J1|J2|J3|J4|J5|J6|
|---|---:|---:|---:|---:|---:|---:|
|Fc (Nm)|0.4|0.35|0.25|0.15|0.12|0.08|
|B (Nm·s/rad)|1|0.875|0.625|0.375|0.3|0.2|

Coulomb, Viscous, Stribeck, Asymmetric, LuGre형의 각 계수는 low=0.5, nominal=1, high=1.5를 Fc와 B에 곱한다. 원본 검증 프로필은 이 값과 수식을 해시로 고정한다. 설정은 패키지의 `a0509_friction_sim/config.json`이며, 출력 `friction_parameters.json`에 축별 실제 계수를 모두 펼쳐 기록한다.

v는 관절 속도, s는 강도다. F는 추가 모터 보상 토크이고 실제 마찰력은 −F다.

|계열|보상 토크|물리 엔진 적용 위치|
|---|---|---|
|Coulomb|native frictionloss=Fcₛ의 inverse − baseline inverse|`dof_frictionloss=Fcₛ`; 추가 sign 토크 없음|
|Viscous|Bₛv|`dof_damping=D₀+Bₛ`; 추가 external 항 없음|
|Stribeck|`[Fcₛ+0.5Fcₛ exp(−(|v|/0.08)²)]tanh(v/0.005)+Bₛv`|D₀ 유지, frictionloss=0, `qfrc_applied=−F`|
|Asymmetric|`Fcₛr(v)tanh(v/0.005)+0.5Bₛv`; r=1.25(v≥0), 0.75(v<0)|D₀ 유지, frictionloss=0, `qfrc_applied=−F`|
|LuGre형|`120z+0.1ż+0.5Bₛv`|D₀ 유지, frictionloss=0, 상태 갱신 후 `qfrc_applied=−F`|

LuGre형은 `g=[Fcₛ+0.5Fcₛ exp(−(|v|/0.08)²)]/120`, `ż=v−|v|z/g`다. 원본의 지수 감쇠 갱신식과 z(0)=0을 재현한다. σ₀=120, σ₁=0.1은 강도에 따라 바꾸지 않는다. 원본은 0..452초, dt=0.1초 전체 이력을 갱신한 뒤 앞뒤 30초를 잘랐다. 잘린 이력부터 z=0으로 재시작하지 않는다. Dynamic에서는 실제 속도와 0.002초 물리 step에서 한 번씩 갱신하므로 원본 0.1초 합성 정답과 같아야 하는 것은 아니다.

원본 궤적 변형 seed는 `20261119+1000×T번호+trial번호`다. 원본 정답은 `tau_reference_nm+delta_friction_nm`이고 reference에 D₀ 감쇠가 이미 포함되어 있다. 기존 Coulomb 정답은 native inverse 차이를 한 번만 더했으며 Fc·sign(v)를 중복 가산하지 않았다.

## 힘 정의와 검증

접촉 없는 6축 팔에서 다음 식을 검사한다. 접촉이 발견되면 실행을 중단한다.

```text
tau_RBD = M(q)ddq + bias
tau_RBD = actuator + passive + constraint + external
passive = −D·dq
qfrc_inverse = actuator + external
required_motor_inverse = qfrc_inverse − external
```

native `frictionloss`는 건마찰 constraint이고 `qfrc_passive`의 damping과 다르다. raw inverse는 순수 RBD가 아니다. nonlinear external 마찰의 음수를 raw inverse에서 한 번 빼 모터 요구 토크를 구한다. 원본 감쇠와 시나리오 마찰을 별도 채널로 저장한다. LuGre 상태는 forward/inverse 호출 횟수와 무관하게 물리 시각마다 한 번만 갱신한다.

Offline은 청크당 32상태, 총 10,752상태×15조건을 같은 q/dq/ddq에서 비교한다. 설정 `offline.samples_per_chunk=0`은 전체 상태 검사이며 별도 config/출력 버전을 사용해야 한다. 원본 전체 336청크의 궤적·이력 재현은 표본 검사 전에 수행한다. 원본 비교 검증에서 원본 정답 대비 최대 차이는 약 2.98e-8 Nm, 마찰항 중복·부호 잔차는 약 3.42e-14 Nm였다.

Dynamic은 nominal T1–T16을 재사용한다. seed 변형 3회는 Offline에서 복원한다. 신규 Dynamic의 같은 결정론적 실행을 3회 복제하지 않는다. 한 도구·무하중 조건당 16궤적×(baseline+15조건)=256회다. 3종 무하중만 수집하면 768회, 기존 7개 도구·하중 조건 전체는 1,792회다.

고정 제어기는 baseline inverse feedforward +150/20 PD, 속도 1.0배, integrator implicitfast, timestep 0.002초, 저장 100 Hz다. 100 Hz feedforward를 500 Hz 제어로 선형 보간한다. 모든 시나리오가 같은 목표·제어기를 사용하고 추가 마찰을 feedforward에서 보상하지 않는다. 자동 감속하지 않는다.

품질 기준은 최대 추종 오차 0.05 rad, 축별 저장 sample RMSE 0.02 rad, 전체 500 Hz 제어 step의 제한 전 명령 포화 비율 0이다. 포화나 추종 실패가 있어도 정상 적분을 마치면 진단 데이터와 `quality_pass=false`를 저장하고 실패 코드로 종료한다. 실제 qacc는 그대로 기록하고 원본 planning 가속도 가이드라인 초과 여부를 별도 표시한다. 실제 위치·속도 제한, 비정상 값, 접촉, 수치 경고는 실행을 중단한다.

## Windows PowerShell 실행

저장소 루트에서 실행한다. 입력 데이터는 H:에서 읽고, 새 시뮬레이션 결과만 F:에 저장한다.

```powershell
Set-Location 'C:\Users\na062\Desktop\graduate\sdml-a0509-ros2'
$ErrorActionPreference = 'Stop'
.\scripts\run_friction_simulation.ps1 -Mode setup
.\scripts\run_friction_simulation.ps1 -Mode analyze -DataRoot 'F:\연구\grad'
.\scripts\run_friction_simulation.ps1 -Mode offline -DataRoot 'F:\연구\grad'
.\scripts\run_friction_simulation.ps1 -Mode pilot -DataRoot 'F:\연구\grad'
.\scripts\run_friction_simulation.ps1 -Mode collect -DataRoot 'F:\연구\grad' -Tool none -MaxJobs 16
.\scripts\run_friction_simulation.ps1 -Mode status -DataRoot 'F:\연구\grad' -Tool none
```

환경은 `.venv-friction-sim`이다. Python 3.10–3.12와 고정 버전 라이브러리를 사용한다. 환경 검사에 `python -c`를 사용하지 않는다. 기본 결과는 `F:\연구\grad\friction_sim_v4`다. 입력 위치를 바꾸려면 `-SourceRoot`, `-LegacySpeedRoot`를 별도로 지정한다. 검증 결과를 보존하면서 다른 실험을 시작하려면 `-ResultsName`과 JSON config를 바꾼다.

`-MaxJobs 16`은 선택한 도구의 T1 baseline+15조건을 실행한다. 같은 명령을 반복하면 완료 파일을 체크섬 검증 후 재사용한다. 해당 도구의 전체 T1–T16을 수집하려면 MaxJobs를 제거한다. `-Tool none|ag145|mdpg_1s1`은 무하중 1조건을 선택한다. 하중은 `-Tool` 대신 `-Conditions ag145_p05,ag145_p10` 등으로 선택한다. Analyze/Offline/Pilot은 Tool 필터 없이 전체 검증을 먼저 수행한다. 기본 pilot은 bare_p0/T15 첫 30초×16조건이며 모든 도구·궤적 검증을 대신하지 않는다.

## Ubuntu에서도 ROS2 없이 실행

저장소 루트에서 Python 가상환경만 준비한다. 시뮬레이터는 수학적 궤적과 모델 자산만 사용하고 ROS 노드·토픽·실로봇 드라이버를 불러오지 않는다.

```bash
python3 -m venv .venv-friction-sim
source .venv-friction-sim/bin/activate
python3 -m pip install -r src/a0509_friction_sim/a0509_friction_sim/requirements.txt
SOURCE="/mnt/h/a0509-data/training_datasets_2026-11-22/data/friction_diverse_v1"
SPEED="/mnt/h/a0509-data/wide16_speed_sim_v1"
OUT="/mnt/f/연구/grad/friction_sim_v4/none"
friction_sim() {
  python3 scripts/collect_friction_simulation.py --source "$SOURCE" --legacy-speed-root "$SPEED" --output "$OUT" "$@"
}
mountpoint /mnt/f && mountpoint /mnt/h &&
friction_sim --mode analyze &&
friction_sim --mode offline &&
friction_sim --mode pilot &&
friction_sim --mode collect --tool none --max-jobs 16 &&
friction_sim --mode status --tool none
```

F:와 H:가 각각 `/mnt/f`, `/mnt/h`로 보이는 경우의 예다. 일반 Ubuntu에서는 실제 마운트 경로로 바꾼다. 세 Ubuntu로 분리할 때는 각 머신의 OUT을 `.../none`, `.../ag145`, `.../mdpg`로 나누고 collect의 tool을 각각 `none`, `ag145`, `mdpg_1s1`로 지정한다. 입력은 읽기 전용으로 공유하고 출력 루트는 공유하지 않는다.

이미 ROS2 워크스페이스를 사용하는 경우에도 이 패키지는 독립적으로 빌드한다. 필요하면 `ros2 run a0509_friction_sim collect_friction --help`로 같은 MuJoCo 실행기를 호출할 수 있다. 기본 수집에는 이 빌드나 ROS2가 필요하지 않다.

## 결과와 실행 보호

`experiment.json`에 config·코드·실제 모델 자산 SHA-256·입력 manifest SHA-256·MuJoCo/NumPy 버전을 저장한다. 각 실행 metadata에는 모델 XML과 그 SHA-256, 계수, timestep, 제어기, seed, 초기 상태, 추종 RMSE, 전체 step 포화율, 최대 실제 가속도, 데이터 SHA-256을 저장한다. 코드·config·모델·입력이 다르면 기존 버전에 이어 쓰지 않는다.

NPZ/CSV에는 실제 q/dq/ddq, 목표 q, 실제 actuator torque, 순수 RBD, raw inverse, 보정 motor inverse, passive force, constraint/dry force, external 및 전체 추가 마찰력, 배경·추가 감쇠, 제한 전 명령, 포화 여부, LuGre z/ż, 힘 평형 잔차를 저장한다. 목표 dq/ddq도 NPZ에 저장한다. 이 값은 시뮬레이션 출력이며 실로봇 센서 토크가 아니다.

`audit`, `offline`, `pilot`, `dynamic`, `models`, `reference`, `logs`, `figures` 폴더로 저장한다. 속도–마찰 곡선은 고정 q의 Offline 특성 검사다. pilot과 전체 길이의 토크 변화·추종 그래프를 각각 보존한다. Dynamic 차이는 실제 상태 변화도 포함하므로 동일 상태의 순수 마찰 효과는 Offline 결과로 확인한다.

프로세스는 한 개씩 실행한다. 시작 시 여유 RAM 2 GiB·저장 공간 10 GiB를 확인하고, 실행 중 worker 2 GiB·호스트 여유 RAM 1 GiB·900초 제한을 감시한다. 중단 시 자식 프로세스도 종료하며 Linux/Windows 모두 파일 잠금으로 같은 출력에 동시 기록하지 못하게 한다. 진단 실패와 partial 파일을 완료 데이터로 취급하지 않는다. 원본 모델과 데이터에는 쓰지 않는다.

전체 summary의 `training_ready=true`는 전체 1,792회가 모두 품질을 통과한 경우에만 설정된다. `--tool`로 선택한 부분 캠페인의 현황은 `--mode status --tool ...`에서 별도 개수로 확인한다. 전체 T1–T16·모든 조건 완료를 pilot 통과만으로 판단하지 않는다.

전체 캠페인은 CSV+NPZ 기준 약 130 GB 규모가 될 수 있다. 10 GiB는 worker 시작을 위한 여유 공간 하한이며 전체 캠페인 용량이 아니다. 최초 검증의 실제 완료 범위와 수치는 [실행 기록](FRICTION_SIMULATION_V4_VALIDATION.md)에 남겼다.
