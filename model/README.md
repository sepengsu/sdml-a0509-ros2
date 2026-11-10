# A0509 + AG145 Physics-Informed KAN

`model/`은 기존 `results/ag145/simulation/`의 **MuJoCo 데이터만** 읽는 독립적인 오프라인 연구 코드입니다. ROS 2 노드를 시작하거나 실제 로봇에 명령을 보내지 않습니다. 이 모델은 실로봇 동역학을 식별한 결과가 아니라, 현재 A0509 + 고정 AG145 MuJoCo 모델의 **역동역학 대리 모델**입니다.

## 모델

[원본 KAN](https://arxiv.org/abs/2404.19756)의 학습 가능한 단변수 스플라인 edge 함수를 2층 KAN에 구현했습니다. [Physics-Informed KAN 연구](https://arxiv.org/abs/2407.18373)를 참고하되, 이 데이터에 맞는 다음의 기계 구조를 직접 적용합니다.

```text
tau(q, dq, ddq) = M_KAN(q) @ ddq + b_KAN(q, dq) + D @ dq
```

- `M_KAN(q)`는 Cholesky 인수로 구성해 항상 대칭 양의 정부호입니다.
- `b_KAN(q,dq) = g_KAN(q) + c_KAN(q,dq)`이며 `c_KAN(q,0)=0`입니다.
- `D=[800,800,800,600,600,600]` Nm·s/rad는 원본 A0509 MuJoCo 장면의 관절 damping을 읽어 고정합니다.
- 손실은 MuJoCo `M(q)` 교사 행렬, 기록된 `qfrc_bias`, 기록된 `qfrc_inverse`의 정규화 MSE 합입니다.
- T1→T2→T3→T4 순서로 학습하며 이전 task별 64개 rehearsal 샘플과 대각 Fisher 기반 EWC를 사용합니다. 둘 다 끈 결과도 저장했습니다.

이는 가속도에 대한 토크의 선형성과 관성행렬의 양의 정부호를 구조로 보장합니다. `b_KAN`의 속도 의존성은 학습하므로 완전한 Euler–Lagrange/Christoffel 제약 모델은 아닙니다.

## 학습한 동역학 꺼내기

최종 체크포인트 `model/runs/ag145_pkan/final.pt`는 수치적 **역동역학 함수**입니다. 입력 `q`(rad), `dq`(rad/s), `ddq`(rad/s²)에 대해 다음 항과 합산 토크를 계산합니다.

```text
tau = M(q) @ ddq + g0(q) + v(q,dq) + D @ dq
g0(q) = b(q,0),  v(q,0) = 0
```

`M(q)`는 6×6 일반화 관성행렬(kg·m²)이고, 나머지 네 토크 항은 관절별 Nm입니다. `g0`는 정지 상태의 바이어스로, 학습 시 중력 단독 라벨로 감독하지 않았습니다. `v`는 속도에 따른 바이어스 **벡터**이며 유일한 코리올리 행렬 `C(q,dq)`를 식별한 결과가 아닙니다. `D`는 MuJoCo 장면에서 읽은 고정 감쇠입니다. 링크별 질량, 질량중심, 관성텐서를 PKAN 가중치에서 직접 역산할 수는 없습니다.

저장소 루트의 PowerShell에서 아래 명령으로 한 관절 상태를 JSON으로 조회하거나, 기록된 네 궤적 전체의 수치 동역학을 CSV와 PNG로 추출할 수 있습니다. 모든 명령은 오프라인으로 동작합니다.

```powershell
# 한 자세: M(q), 네 토크 항, 합산 토크, 운동에너지
model/.venv/Scripts/python.exe -m model.predict --q -0.5 -0.5 1.6 0 -0.6 0 --dq 0 0 0 0 0 0 --ddq 0 0 0 0 0 0

# T1~T4, 기본 5행 간격(원본 100 Hz 중 20 Hz); --stride 1은 모든 유효 행
model/.venv/Scripts/python.exe -m model.export_dynamics --trajectory all
model/.venv/Scripts/python.exe -m model.plot_dynamics

# MuJoCo 정지/속도 바이어스와 같은 trial의 마지막 20%에서 따로 비교
model/.venv/Scripts/python.exe -m model.validate_decomposition
```

출력은 `model/exports/ag145_pkan/`에 저장됩니다. `{T1..T4}_dynamics.csv`에는 입력 관절 상태, `M_11`~`M_66`, `zero_velocity_bias`, `velocity_bias`, `inertia_torque`, `damping_torque`, 합산 예측 토크, MuJoCo 토크와 오차가 있습니다. `manifest.json`에는 단위, 원본 trial, 체크포인트 SHA-256, 샘플 간격, 전체 기록 행에 대한 오차를 저장합니다. 이 전체 기록 오차에는 학습 행도 포함되므로 아래 시간 구간 검증 수치와 구별해야 합니다. `figures/`에는 각 궤적의 관성행렬과 토크 분해 그래프가 있습니다.

정지 바이어스 `g0(q)`를 MuJoCo에서 **해당 자세의 속도를 0으로 놓고 새로 계산한** `qfrc_bias(q,0)`와 비교한 시간 구간 검증 RMSE는 T1~T4 순서로 0.102, 0.336, 0.297, 0.255 Nm입니다. 속도 바이어스도 `qfrc_bias(q,dq)-qfrc_bias(q,0)`와 비교했으며 상세 결과는 `decomposition_validation.json`에 있습니다. 이는 같은 시뮬레이터 내부 분해와의 일치도이지 실로봇 검증이 아닙니다.

## 데이터와 검증 분할

| Task | 원본 trial | 100 Hz 원본 행 | 5행 간격 유효 샘플 | 학습 / 시간 구간 검증 |
|---|---|---:|---:|---:|
| T1 | `trial_005` | 1,001 | 200 | 160 / 40 |
| T2 | `trial_001` | 1,001 | 200 | 160 / 40 |
| T3 | `trial_001` | 3,401 | 680 | 544 / 136 |
| T4 | `trial_001` | 7,401 | 1,480 | 1,184 / 296 |

각 task의 앞 80%를 학습하고 마지막 20%를 검증합니다. CSV의 빈/비유한 관측값은 제외합니다. `model/data/ag145_mass_matrices.npz`는 동일한 MuJoCo 3.2.7 장면에서 각 기록 자세의 질량행렬을 추출한 교사 데이터입니다. `physics_check.py`는 `qfrc_inverse = M(q)ddq + qfrc_bias + D dq`가 이 데이터에서 수치 오차 범위로 성립하는지 확인합니다. 학습 시 CSV SHA-256과 자세 배열을 대조해 교사 데이터의 불일치를 거부합니다.

## 현재 학습 결과

기본 설정 40 epoch/task, seed 42의 **시간 구간 검증** 토크 RMSE입니다. 단위는 Nm이며, 6개 관절과 해당 task의 검증 행을 함께 계산했습니다.

| 최종 T4 학습 뒤 검증 | Replay + EWC | Replay/EWC 없음 | T1 평균 질량행렬·bias 고정 기준 |
|---|---:|---:|---:|
| T1 | 0.061 | 13.197 | 2.358 |
| T2 | 0.179 | 13.006 | 2.671 |
| T3 | 0.332 | 12.918 | 2.140 |
| T4 | 0.264 | 0.388 | 9.235 |

세부 task별 관절 오차·bias·관성행렬 오차는 `runs/ag145_pkan/metrics.json`, 비교값은 `comparison.json`에 있습니다. 두 학습은 동일한 seed와 epoch를 사용했습니다. 이 비교는 하나의 시뮬레이션 trial/작업 조건에서 얻은 결과이며 독립 반복 실험은 아닙니다.

## 실행 (Windows PowerShell, 저장소 루트)

```powershell
uv venv model/.venv --python 3.12
uv pip install --python model/.venv/Scripts/python.exe --index-url https://download.pytorch.org/whl/cpu torch numpy
uv pip install --python model/.venv/Scripts/python.exe -r model/requirements.txt

# 이미 포함된 교사 데이터 재생성 및 물리식 확인
model/.venv/Scripts/python.exe -m model.teacher
model/.venv/Scripts/python.exe -m model.physics_check

# T1→T4 Continual Learning
model/.venv/Scripts/python.exe -m model.train

# 비교 학습과 그림
model/.venv/Scripts/python.exe -m model.train --replay-per-task 0 --ewc-lambda 0 --output model/runs/ablation_no_continual
model/.venv/Scripts/python.exe -m model.plot

# 오프라인 토크 예측 예시: q(rad), dq(rad/s), ddq(rad/s²)
model/.venv/Scripts/python.exe -m model.predict --q -0.5 -0.5 1.6 0 -0.6 0 --dq 0 0 0 0 0 0 --ddq 0 0 0 0 0 0

model/.venv/Scripts/python.exe -m unittest discover -s model/tests -v
```

학습된 가중치는 `runs/ag145_pkan/final.pt`, 각 task 뒤 가중치는 `after_T1.pt`~`after_T4.pt`입니다. 검증 예측 CSV와 그래프도 모두 `model/runs/`에 있습니다.

## 적용 범위

- 라벨 `qfrc_inverse`와 질량행렬은 **같은 MuJoCo 모델**에서 생성됐습니다. 작은 검증 오차가 실제 A0509의 독립 토크 측정 정확도를 뜻하지 않습니다.
- AG145는 1.3 kg 고정 강체 근사이며 손가락 운동·물체 파지·변경된 payload를 학습하지 않았습니다.
- 기록된 `ddq`는 관측 속도의 벽시계 차분입니다. 새 실험에서는 시뮬레이션 시각 기준 가속도 또는 MuJoCo 내부 `qacc`를 직접 기록하는 편이 좋습니다.
- 검증 구간은 같은 trial의 마지막 부분입니다. 새 trial, 속도, payload, 실로봇 RT 데이터에 대한 일반화는 아직 시험하지 않았습니다. 이 체크포인트를 실로봇 제어에 사용하지 마세요.
