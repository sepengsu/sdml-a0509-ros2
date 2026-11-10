# mDPG 영상 원본 T1 데이터 분석

분석 대상은 [mDPG 시각 재생 영상](../mdpg_t1_visual_replay.mp4)을 만든 **A0509 팔만의 MuJoCo T1 실행**입니다. 원본은 `results/mdpg_visual_source/simulation/T1/trial_001/dynamics.csv`의 1,001행(시뮬레이션 시간 10.00초)입니다. mDPG는 영상에서만 보이며, 원본 실행의 질량·관성·접촉·공압 물리에는 포함되지 않습니다. [summary.json](summary.json)에 원본 SHA-256, 계산식, 수치와 제한을 기록했습니다.

| 분석 | 확인한 내용 | 그래프 |
|---|---|---|
| 궤적 추종 | 6축 목표·관측 관절각 및 오차, 계획·관측 팔 TCP | [Joint Tracking](joint_tracking.png), [TCP Tracking](tcp_tracking.png) |
| 운동 다양성 | 관절별 q·dq·ddq 분포와 상태 공간 | [q–dq–ddq Distribution](q_dq_ddq_distribution.png), [Phase Space](joint_phase_space.png) |
| 동역학 특성 | 역동역학 토크, 관성·중력·속도 bias·감쇠 성분 | [Torque Profile](torque_profile.png), [Component RMS](torque_component_rms.png) |
| 데이터 품질 | 시각 간격, 누락, 큰 이상치 후보, 팔 모델 접촉, 물리식 재구성 | [Quality Summary](quality_summary.png) |

## 주요 결과

계획된 팔 TCP의 X 이동 범위는 **120.0 mm**, 관측 범위는 **121.83 mm**입니다. 시간별 X 추종 RMSE는 **11.47 mm**, 최대 절대오차는 **20.70 mm**로, 왕복 중 추종 지연이 보입니다. 이 TCP는 원본 팔 모델 기준이며 보정되지 않은 mDPG 빨판 TCP의 실측값이 아닙니다.

| 관절 | 각도 추종 RMSE | q 범위 | 최대 속도 | 최대 가속도 | 역동역학 토크 최대 절댓값 |
|---|---:|---:|---:|---:|---:|
| J1 | 1.22° | 13.64° | 6.62°/s | 17.86°/s² | 92.36 Nm |
| J2 | 1.29° | 14.61° | 6.75°/s | 15.26°/s² | 95.22 Nm |
| J3 | 0.78° | 7.93° | 3.79°/s | 27.95°/s² | 69.41 Nm |
| J4 | 0.69° | 9.83° | 5.27°/s | 18.39°/s² | 57.53 Nm |
| J5 | 0.52° | 7.52° | 3.71°/s | 9.93°/s² | 38.95 Nm |
| J6 | 1.35° | 19.35° | 10.07°/s | 34.93°/s² | 105.50 Nm |

6축 모두 위치·속도·가속도 분포가 형성됐지만, **한 개의 저속 왕복 trial**입니다. 높은 속도·하중·다양한 작업 공간의 동역학 식별 범위를 대표하지 않습니다. 가속도는 MuJoCo 내부 구동 가속도가 아니라 ROS 시뮬레이션 시각 기준 최근 5개 관절 속도의 기울기입니다.

기록된 물리 항은 다음처럼 분해했습니다.

```text
qfrc_inverse = M(q)·ddq + gravity(q) + velocity_bias(q,dq) + damping·dq
velocity_bias = qfrc_bias - gravity(q)
```

중력 항은 동일한 팔 MuJoCo 모델에서 기록된 `q`와 `dq=0`으로 재계산했습니다. MuJoCo 장면의 관절 감쇠 계수는 J1~J3 **800**, J4~J6 **600 N·m·s/rad**입니다. 그 결과 감쇠 RMS가 J1 **49.38**, J2 **52.71**, J3 **28.82**, J4 **27.36**, J5 **20.67**, J6 **53.50 Nm**로 대부분의 토크 파형을 지배합니다. J3의 중력 RMS는 **16.43 Nm**이며 관성·속도 bias 항은 상대적으로 작습니다. 이 `qfrc_inverse`는 별도 MuJoCo 모델의 **재구성값**입니다. `qfrc_actuator`와 `qfrc_applied`는 전 행에서 이용할 수 없으므로 실제 모터 요구 토크나 물리 토크 센서 값으로 표시하지 않습니다. 이 데이터를 바로 PKAN의 실기 토크 정답으로 사용하면 모델에 설정된 큰 감쇠를 학습할 위험이 있습니다.

## 품질 판정

필수 관절 상태·목표·TCP·역동역학 필드는 빈 셀이 **0**입니다. JointState 관측 주기는 **99.99 Hz**, 상위 로거의 추정 누락률은 **0%**입니다. 다만 저장된 행 간격에는 1.5배 중앙값(약 15 ms)을 넘는 구간이 **3개** 있고 최대 **50 ms**입니다. 이는 기록 행의 초기 시각 지연을 보여 주며, JointState 메시 누락률 0%와는 다른 지표입니다.

명시한 큰 점프·절댓값 임계값의 이상치 후보는 0개, 관측 자세에서 팔 모델 접촉은 0개였습니다. 이 검사는 **mDPG 접촉을 검사하지 않습니다**. 물리식 재구성 최대 절대오차는 **1.42×10⁻¹⁴ Nm**입니다. 이는 같은 MuJoCo 질량행렬·bias·감쇠 항의 산술 일치이며 독립 실측 검증이 아닙니다. [관절별 CSV](per_joint_summary.csv)에 수치를 보존했습니다.

저장소 루트에서 `model/.venv/Scripts/python.exe -m model.analyze_mdpg_t1`로 그래프와 보고용 수치를 다시 생성할 수 있습니다. 다른 환경에서는 NumPy, Matplotlib, MuJoCo가 설치된 Python으로 같은 모듈을 실행합니다.
