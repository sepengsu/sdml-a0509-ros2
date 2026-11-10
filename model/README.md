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
