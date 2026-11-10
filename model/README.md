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
- 이 절의 기존 PhysicsKAN 검증 구간은 같은 trial의 마지막 부분입니다. 새 trial, 속도, payload, 실로봇 RT 데이터에 대한 일반화는 이 기존 체크포인트에서 시험하지 않았습니다. 이 체크포인트를 실로봇 제어에 사용하지 마세요.

## RBD + Residual PKAN 연구 (`rbd_v1`)

이 절은 위의 **기존 PhysicsKAN 및 Replay/EWC 실험과 별개**입니다. 기존 모델과 결과를 변경하지 않고 `model/residual_*`, `model/rbd.py`에 새 연구를 추가했습니다. 모든 계산은 오프라인 MuJoCo 역동역학이며 실로봇 토크 측정이나 제어가 아닙니다.

### 모델과 데이터

```text
tau_reference = MuJoCo reference model의 mj_inverse(q,dq,ddq)
tau_nominal   = 불확실한 Tool 파라미터를 적용한 MuJoCo RBD
delta_target  = tau_reference - tau_nominal
tau_pred      = tau_nominal + KAN(sin(q),cos(q),dq,ddq,nominal_tool_10)
```

`RBDModel`은 `M(q)ddq + bias(q,dq) + damping`을 계산하고 `mj_inverse`와 비교합니다. 감쇠는 한 번만 더합니다. `tool_aware_rbd`는 정확한 기준 Tool 파라미터로 같은 RBD 식을 재구성하는 상한 비교군입니다. 동일한 MuJoCo 물리식과 파라미터를 사용하므로 거의 0인 오차는 **구성상 예상된 값**입니다. 입력 34개, 출력 6개인 `ResidualPKAN`은 기존 `model/pkan.py`의 스플라인 `KAN(34,16,6,grid_intervals=8)`을 재사용합니다. 정규화 통계는 학습 분할에서만 계산해 체크포인트에 저장합니다. MLP 비교군도 같은 입력과 타깃을 사용합니다.

Tool 조건은 `bare`(엔드이펙터 없음), 기존 근사 AG145(1.3 kg, 로컬 주관성 0.0073/0.0050/0.0033 kg·m², COM z=0.10 m), 그리고 합성 `virtual_light`(0.9 kg), `virtual_heavy`(1.7 kg), `virtual_unseen`(1.5 kg)입니다. 가상 Tool의 COM과 관성, 플랜지 좌표계 기준 10개 파라미터는 [manifest.json](residual_data/rbd_v1/manifest.json)에 기록했습니다. AG145 형상과 장착은 저장된 MuJoCo 장면을 사용하고 가상 Tool은 그 장면의 관성만 바꿉니다. 공압 그리퍼의 검증된 물리 파라미터로 해석하면 안 됩니다. 학습 입력에는 **명목 추정치만** 넣으며 기준 파라미터는 라벨 계산과 감사 기록에만 사용합니다. 일반 조건의 명목 질량은 기준값의 0.8배, 관성은 0.75배이고 로컬 COM을 (+0.005, -0.005, -0.01) m 옮긴 합성 오차입니다. `bare`에는 Tool 불일치가 없습니다. 실제 운용에서 명목값을 추정한 실험이 아니므로 Tool 일반화 결과는 이 합성 오차 규칙에 한정됩니다.

기존 T1~T4 AG145 기록의 `q,dq,ddq`를 5행 간격으로 읽고, 모든 Tool 조건에 **같은 상태**를 입력해 짝지은 기준/명목 토크를 만들었습니다. AG145 기준 토크와 원본 `qfrc_inverse`의 최대 차이는 T1~T4에서 2.9e-14 Nm 이하였습니다. 이는 원본 재현 확인이며 독립 실측이 아닙니다. `--payload-mass`는 현재 `simulation_executor.py`에서 결과 메타데이터와 로거에만 전달되며 MuJoCo 질량·관성을 바꾸지 않습니다. 이 연구에서는 해당 옵션에 의존하지 않고 별도 MuJoCo 모델의 `<inertial>`을 명시적으로 변경합니다.

| 분할 | 샘플 | 조건 |
|---|---:|---|
| 학습 | 3,456 | bare/AG145/가상 light/heavy, T1~T3의 앞 80% |
| 검증 | 864 | 같은 네 Tool, T1~T3의 뒤 20%; early stopping 전용 |
| Test A: 새 trial | 960 | 학습하지 않은 8초 해석적 6축 관절 상태열, 네 Tool |
| Test B: 새 궤적 | 5,920 | 학습에서 완전히 제외한 T4, 네 Tool |
| Test C: 새 Tool | 1,080 | 학습하지 않은 `virtual_unseen`, T1~T3 상태 |
| Test D: 불일치 강도 | 1,480 | AG145 T4 부분집합, 명목 질량 배율 0.6~1.0 |

Test A는 **새로 생성한 상태열의 오프라인 역동역학 계산**으로, ROS/MuJoCo에서 새 궤적을 실행해 측정한 trial이 아닙니다. Test C는 Tool 파라미터만 새로운 조합이며 T1~T3 관절 상태를 재사용합니다. Test D는 T4 부분집합이라 Test B와 독립적이지 않습니다. 데이터셋 총 13,760행이며 분할 행과 원본 SHA-256은 manifest에 있습니다.

### 실행

저장소 루트에서 실행합니다. 기존 데이터와 MuJoCo 장면 및 `src/doosan-robot2` 서브모듈이 필요합니다. 의존성은 [requirements.txt](requirements.txt)의 NumPy, PyTorch, MuJoCo, Matplotlib입니다. Linux에서는 환경을 활성화한 `python3`을 사용합니다.

```powershell
# Windows PowerShell
model/.venv/Scripts/python.exe -m model.residual_data
model/.venv/Scripts/python.exe -m model.train_residual
model/.venv/Scripts/python.exe -m model.evaluate_residual
model/.venv/Scripts/python.exe -m model.plot_residual
model/.venv/Scripts/python.exe -m model.predict_residual --tool-condition ag145 --q -0.5 -0.5 1.6 0 -0.6 0 --dq 0 0 0 0 0 0 --ddq 0 0 0 0 0 0
model/.venv/Scripts/python.exe -m unittest discover -s model/tests -v
```

```bash
# Linux, 저장소 루트와 활성화된 Python 3.12 환경
python3 -m model.residual_data
python3 -m model.train_residual
python3 -m model.evaluate_residual
python3 -m model.plot_residual
python3 -m model.predict_residual --tool-condition ag145 --q -0.5 -0.5 1.6 0 -0.6 0 --dq 0 0 0 0 0 0 --ddq 0 0 0 0 0 0
python3 -m unittest discover -s model/tests -v
```

학습 기본값은 AdamW, 학습률 0.002, batch 128, 최대 100 epoch, 검증 손실 patience 15, seed 42, 잔차 정규화 손실 가중치 0.001입니다. 이번 실행에서 PKAN은 최적 4 epoch/종료 19 epoch, MLP는 최적 22 epoch/종료 37 epoch였습니다. `predict_residual`은 선택한 사전 정의 Tool의 **명목** 토크와 보정 토크를 출력하며 실로봇 명령은 보내지 않습니다.

### 실제 실행 결과

각 값은 해당 분할의 모든 행·6축을 합친 토크 RMSE [Nm]입니다. 결과는 [metrics.json](residual_runs/rbd_v1/metrics.json)에 관절별 RMSE, MAE, 최대 절대오차, 개선율과 함께 저장했습니다.

| 평가 | Nominal RBD | RBD + Residual PKAN | Tool-aware RBD | RBD + Residual MLP | PKAN 개선율 |
|---|---:|---:|---:|---:|---:|
| 시간 구간 검증 | 0.4607 | 0.0312 | ≈0 | 0.0285 | 93.2% |
| Test A: 새 해석적 trial | 0.4618 | 0.0362 | ≈0 | 0.0403 | 92.2% |
| Test B: 미학습 T4 | 0.0779 | **0.4389** | ≈0 | 0.4413 | **-463.1%** |
| Test C: 미학습 Tool | 0.6578 | 0.0705 | ≈0 | 0.0331 | 89.3% |
| Test D: T4 불일치 강도 | 0.0855 | **0.4056** | ≈0 | 0.4295 | **-374.6%** |

AG145만 보면 새 trial은 0.5228→0.0385 Nm로 개선됐지만, T4는 0.0737→0.4755 Nm로 악화됐습니다. 현재 KAN은 T1~T3 부근의 Tool 오차를 잘 근사하나, T4처럼 명목 RBD 오차 자체가 작은 새 상태 분포에서는 과보정합니다. 따라서 이 체크포인트를 미학습 궤적이나 실제 로봇 제어에 적용하면 안 됩니다. 정확한 Tool 파라미터를 가진 RBD가 거의 0 오차인 점도 AI 보정의 필요 범위를 제한합니다. 기존 PhysicsKAN은 학습 목적과 조건이 달라 이 표에 직접 섞지 않았습니다.

[dataset.csv](residual_data/rbd_v1/dataset.csv)와 `dataset.npz`에 입력·라벨·분할을, [predictions.csv](residual_runs/rbd_v1/predictions.csv)에 기준/명목/잔차/보정 토크를 저장했습니다. [training.json](residual_runs/rbd_v1/training.json)과 체크포인트 `best_pkan.pt`, `best_mlp.pt`가 학습 이력을 보존합니다. `figures/`에는 `torque_comparison.png`, `joint_rmse.png`, `residual_torque.png`, `generalization.png`, `loss_curve.png`가 있습니다. 단위는 rad, rad/s, rad/s², Nm, Tool 질량 kg, COM m, 관성 kg·m²입니다.

이 결과는 같은 MuJoCo 엔진 안에서 파라미터 불일치를 합성한 평가입니다. 실제 A0509 RT 토크와 비교하지 않았고 Sim-to-Real Gap 보정을 검증하지 않았습니다. 다음 단계에서는 독립적인 시뮬레이션 rollout과 실측 Tool 관성을 확보하고, T4 및 새 작업 공간을 포함해 재학습/재평가해야 합니다. 그 후에도 실로봇에는 별도 안전 검증이 필요합니다. Tool 순차 변경에 대한 Replay/EWC 비교는 후속 연구 범위입니다.
