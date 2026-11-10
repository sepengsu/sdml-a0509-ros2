# A0509 + AG145 MuJoCo dataset

AG145를 벌린 고정 자세로 장착한 A0509의 시뮬레이션 결과입니다. 실제 로봇 측정값은 없습니다.

| 궤적 | 유효 trial | dynamics 행 | 시뮬레이션 시각 기준 Hz | 추정 누락률 | 계획–측정 오차 중앙값 / 95백분위 |
|---|---|---:|---:|---:|---:|
| T1 | `simulation/T1/trial_004` | 1,001 | 99.99 | 0% | TCP 10.10 / 25.26 mm |
| T2 | `simulation/T2/trial_001` | 1,001 | 99.99 | 0% | TCP 8.79 / 20.54 mm |
| T3 | `simulation/T3/trial_001` | 3,401 | 99.99 | 0% | TCP 9.58 / 22.90 mm |
| T4 | `simulation/T4/trial_001` | 7,401 | 99.86 | 0.134% | 관절 벡터 0.008 / 0.022 rad |

오차는 `trajectory.csv`와 `dynamics.csv`의 같은 행을 비교한 값입니다. 각 행의 측정 상태는 새 목표를 발행한 직후 확보한 최신 관절 상태이므로 제어 지연을 포함합니다. T4 값은 6개 관절의 L2 norm이며 TCP 오차와 단위가 다릅니다.

각 유효 trial에는 기준 궤적 CSV/JSON, waypoint CSV, 동역학 CSV, metadata, PNG/HTML 그래프, ROS 2 Bag, AG145 MuJoCo scene 및 축소 메시가 있습니다. T1에는 `simulation.mp4`(MuJoCo 화면 녹화)와 `simulation_replay.mp4`(수집된 qpos를 근접 시점으로 렌더링)가 추가로 있습니다. `preview/T1`~`T4`는 로봇 연결 없이 만든 경로 그림입니다.

모델의 AG145 질량은 1.3 kg이며 관성과 충돌 형상은 근사값입니다. AG145 TCP는 플랜지에서 200 mm 떨어진 임시 기준점입니다. 손가락 구동, 실물 TCP 보정, 실물 충돌 안전성은 검증하지 않았습니다. `qfrc_actuator`는 상위 MuJoCo ROS bridge에서 노출하지 않아 빈 값입니다. 원본 모델 및 재생 방법은 `src/a0509_dynamics_experiment/README.md`를 참조하세요.
