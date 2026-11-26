# A0509 TCC1 Tool Changer

TCC1을 A0509의 `link_6` 뒤에 추가하는 시뮬레이션 모델이다. `tool_changer_type:=tcc1`에서 그리퍼가 없으면 Master만 장착한다. 그리퍼를 선택하면 고정 연결 사슬은 `link_6 → Master → Tool → Gripper → Payload`이다. Rack은 선택 시 `world`에 고정한다.

## 검증 실행

전체 A0509 ROS 2 작업 공간을 빌드하고 소싱한 다음 실행한다. 각 검증용 Launch는 설치된 A0509 RViz 또는 MuJoCo Launch에 같은 파라미터를 전달한다.

```bash
ros2 launch tcc1_tool_changer_description verify.launch.py viewer:=rviz gripper_type:=ag145
ros2 launch tcc1_tool_changer_description verify.launch.py viewer:=mujoco gripper_type:=ag145 tool_changer_dynamics_model:=dynamics
ros2 launch tcc1_tool_changer_description verify.launch.py viewer:=rviz gripper_type:=none tool_rack_enabled:=true
```

`a0509_mujoco_sim mujoco_sim.launch.py` 및 `sdml_a0509_bringup`의 `rviz_only.launch.py`, `mujoco.launch.py`에서도 `tool_changer_type`, `tool_changer_dynamics_model`, `tool_rack_enabled`, `tcc1_config`를 받는다. 기본값은 `none`, `visual`, `false`, 패키지 내 설정 파일이다. TCC1이 활성화된 mDPG의 장착 위치는 공유 YAML에서 읽는다.

기존 캠페인과 별도의 `a0509_dynamics_experiment.tcc1_conditions`에 신규 7개 조건을 정의했다. `build_tcc1_condition_scene`은 요청한 한 조건의 MJCF만 만든다. 페이로드가 있는 조건에는 기존 `payload_conditions.json`을 넘긴다. T5 비교는 기존 `grid_16_trajectories`의 `ag145_p0/trial_001` 17초 기록을 사용한다. AG145와 mDPG에서 장착 전후에 동일한 q, dq, ddq를 넣고 역동역학을 계산한다. 결과는 `output/tcc1_nominal`에 보고서, 토크 비교 그래프, AG145 비교 영상으로 저장된다.

```powershell
$env:PYTHONPATH = 'src/tcc1_tool_changer_description;src/a0509_dynamics_experiment;src/mdpg_1s1_description'
.\.venv\Scripts\python.exe -m scripts.render_tcc1_comparison
.\.venv\Scripts\python.exe -m scripts.compare_tcc1_dynamics
.\.venv\Scripts\python.exe -m pytest -q src/tcc1_tool_changer_description/test
```

## CAD 정합과 확인 범위

원본 Master와 Tool STEP을 OpenCascade로 읽어 플랜지 평면, 중심 Ø42 mm 체결 형상, 공압 포트 중심 `(-37, ±8)` mm, Pogo 방향의 상·하부 부품 좌표를 비교했다. X/Y 등록을 유지하고 Tool의 원본 CAD Z에 +55 mm를 적용했다. 이때 공압과 Pogo 하우징의 후보 대향면 간격은 각각 약 0.1 mm, 0.2 mm이다.

Master 플랜지 Z는 +59.51547 mm이고, Tool Plate 본체 출력면 Z는 −37.48453 mm이다. +55 mm를 적용한 Tool 출력면은 Master CAD 기준 +17.51547 mm이므로 두 면 사이 거리는 42.0 mm이다. 원본 조립체에 포함된 Tool 출력 연장 부품은 본체 출력면에서 Z −57.48453 mm까지 20 mm 더 길다. 이전 62 mm 영상은 이 연장 부품을 포함한 형상이다. 공칭 프로파일은 본체 출력면에서 그리퍼를 장착한다. STEP의 Master 조립체에는 Ø77 mm 플랜지판과 측면 구조도 있어 카탈로그 외형 Ø63 mm와 형상 범위가 다르다. 이 플랜지판의 BOM 귀속, 실제 잠금 위치 및 올바른 어댑터 조합은 실측과 도면 확인이 필요하다.

| 항목 | 좌표 및 상태 |
| --- | --- |
| 플랜지 → Master | Master의 로봇 쪽 평면은 원본 CAD 기준 Z +59.51547 mm. A0509 플랜지 원점에 동심 배치하고 CAD를 X축으로 π rad 회전했다. 실제 볼트 패턴과 회전각은 미확인이다. |
| Master → Tool | 공유 CAD 기준면에 따라 로컬 +Z 27.6 mm. 공압과 Pogo 위치로 같은 방향을 선택했다. CAD 기반 추정이다. |
| Tool → AG145 / mDPG | 본체 출력면 Z −37.48453 mm를 기준으로 Tool 체결 프레임부터 로컬 +Z 14.4 mm. AG145는 기존 45° yaw, mDPG는 동심 방향을 사용한다. 실제 어댑터 형상과 회전각은 미확인이다. |
| Gripper → TCP | 기존 명목 좌표인 AG145 +200 mm, mDPG +90 mm를 사용한다. T5에서 두 경우 모두 장착 전후 TCP 위치 차이는 42.0 mm다. |
| World → Rack | 시각 검사 배치이며 설치 위치는 미확인이다. Rack에는 접촉 및 관성이 없다. |

원본 STLs와 매니페스트를 보관한다. 공칭용 STL은 Master 본체 그룹 32개, Tool Plate 본체 그룹의 5개 솔리드로 다시 생성했다. 별도 이름이 붙은 Air/Pogo 그룹과 20 mm Tool 연장 솔리드는 공칭 메시 및 물성 계산 범위에서 제외했다. URDF와 MJCF는 같은 0.001 배율, 공칭 메시 및 장착 좌표를 사용한다. `scripts/preprocess_tcc1_nominal_cad.py`와 `nominal_cad_manifest.json`에 추출 조건 및 원본 STEP 해시를 기록했다.

## 질량과 관성

[Magbot 제조사 카탈로그](https://pub-mediabox-storage.rxweb-prd.com/exhibitor/document/exh-2afd04a1-42c8-4ecb-ab25-b61a71cfe688/bb005229-0889-4d80-900e-d54a5439b9d0.pdf)의 TCC1 공칭 질량을 사용한다. 제조사 치수, 정격 5 kgf, 반복정밀도 ±0.05 mm는 YAML에 스펙으로 기록했다. 정격 하중과 반복정밀도는 시뮬레이션의 페이로드나 관절 오차에 입력하지 않는다.

| 부품 | 질량 (kg) | 장착 프레임 CoM (mm) | CoM 기준 관성텐서 (kg·m²) |
| --- | ---: | --- | --- |
| Master | 0.448 | −0.002673, 5.475862, 14.057852 | 2.489835e−4, 1.517190e−4, 3.220366e−4, 2.302316e−6, −2.766e−9, 2.02917e−7 |
| Tool Plate | 0.145 | 0.245257, 1.195779, 8.536286 | 5.447382e−5, 5.043582e−5, 9.836337e−5, 6.40503e−7, 9.4607e−8, 6.78827e−7 |

CoM과 관성은 선택된 솔리드의 체적을 합산한 균일 밀도 추정값이다. 서로 겹치는 CAD 솔리드나 다른 재질은 개별 보정하지 않았다. CAD CoM에는 CAD→장착 프레임의 회전과 이동을 적용했다. CoM 기준 단위 질량당 관성텐서는 같은 회전으로 좌표축을 바꾼 뒤 공칭 질량을 곱했다. 텐서 순서는 `[Ixx, Iyy, Izz, Ixy, Ixz, Iyz]`이다.

추후 실측값은 별도 YAML 복사본의 `mass_kg`, `com_m`, `parameter_source`, `verified`로 관리한다. `inertia_kg_m2`가 비어 있으면 바뀐 질량에 맞춰 CAD 단위 텐서를 다시 스케일한다. 실측 CoM이 CAD CoM과 다를 때 질량 스케일만으로 실제 관성이 보정되지는 않으므로, 확보한 CoM 기준 전체 텐서를 `inertia_kg_m2`에 명시할 수 있다. 외부 기준점의 텐서는 장착 프레임 축으로 회전하고 평행축 정리로 CoM 기준으로 변환한 뒤 입력한다. 누락된 값이나 물리적으로 유효하지 않은 관성은 모델 생성 시 오류를 낸다.

Air Coupler와 Pogo Coupler는 제조사 카탈로그에서 액세서리로 분류한다. 해당 CAD 리비전의 공급 BOM과 각 부품이 실제 0.593 kg에 포함되는지는 확인되지 않았다. `accessories.master`와 `accessories.tool`에는 각 Coupler 절반과 로봇/그리퍼 측 어댑터의 질량을 따로 둘 수 있다. 질량이 `null`이면 미확보 상태로 역동역학에서 제외한다. 입력 시 해당 부모 프레임의 CoM·관성 및 출처도 채운다. 별도 어댑터가 장착 거리를 바꾸는 경우에는 해당 `mounts`와 메시도 실물에 맞춰 수정해야 한다.

이번 결과는 CAD와 제조사 공칭값을 조합한 추정 모델이다. T5 0 kg에서 AG145 모델 총질량은 20.124→20.717 kg, mDPG는 19.444→20.037 kg이다. 같은 기록에서 장착 전후 역동역학 토크 차이가 가장 큰 관절은 두 경우 모두 J3이며, 최대 절댓값은 각각 2.757 및 2.598 N·m다. 실장 질량, CoM, 관성, 어댑터 형상·정렬 및 실제 TCP가 후속 검증 항목이다.

## 변경 위치

- 이 패키지의 `config/`, `meshes/`, `urdf/`, `launch/`, `tcc1_tool_changer_description/model.py`, `test/`
- `sdml_a0509_description/urdf/a0509_with_gripper.urdf.xacro` 및 `mdpg_1s1_description/urdf/mdpg_1s1_macro.xacro`
- `a0509_mujoco_sim/launch/mujoco_sim.launch.py`, `sdml_a0509_bringup/launch/{mujoco,rviz_only}.launch.py` 및 패키지 의존성
- `a0509_dynamics_experiment/tcc1_conditions.py` 및 `scripts/{preprocess_tcc1_nominal_cad,render_tcc1_comparison,compare_tcc1_dynamics}.py`
