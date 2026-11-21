# M0–M4: 7조건 공동 동역학 비교

구현: [`residual_models.py`](residual_models.py), [`prepare_joint_dataset.py`](prepare_joint_dataset.py), [`train_joint_models.py`](train_joint_models.py), [`evaluate_joint_models.py`](evaluate_joint_models.py). 완료된 이전 궤적 데이터의 실제 실행 결과는 [`LEGACY_M0_M4_RESULTS.md`](LEGACY_M0_M4_RESULTS.md)에 있다. 현재 구축 중인 `data/grid_16_wide_v3`의 결과는 전체 수집이 끝난 뒤 별도로 실행해야 한다.

| 번호 | 모델 | 학습 | 출력 토크 |
|---|---|---|---|
| M0 | Nominal RBD | 없음 | 도구·페이로드 관성을 0.8배로 둔 명목 RBD |
| M1 | Direct MLP | 7조건 공동 학습 | MLP 직접 예측 |
| M2 | Direct KAN | 7조건 공동 학습 | KAN 직접 예측 |
| M3 | RBD + MLP | 7조건 공동 학습 | M0 + MLP 잔차 |
| M4 | RBD + KAN | 7조건 공동 학습 | M0 + KAN 잔차 |

## 공통 입력과 출력

한 표본의 입력 22개는 순서대로 `q1..q6` [rad], `dq1..dq6` [rad/s], `ddq1..ddq6` [rad/s²], 그리퍼 종류 `[bare, AG145, mDPG]` 원핫 3개, 페이로드 질량 [kg] 1개다. 동일한 입력을 M1–M4에 제공한다. 출력은 모두 6축 토크 [Nm]다. M1·M2의 학습 표적은 `τ_reference`, M3·M4의 학습 표적은 `Δτ = τ_reference − τ_nominal_RBD`다. 명목 RBD 토크는 네트워크 입력에 포함하지 않고 M3·M4의 출력에 더한다.

| 모델 | 네트워크 | 비선형 함수 |
|---|---|---|
| Direct / Residual MLP | `22 → 96 → 96 → 6` | 두 은닉층 SiLU, 마지막 Linear |
| Direct / Residual KAN | `22 → 32 → 16 → 6` | 세 KAN 층 모두 3차 B-spline edge 함수, grid 간격 5 |

현재 구현의 학습 파라미터는 MLP 12,102개, KAN 10,550개다. 별도의 폭 탐색이나 파라미터 수 일치 조정은 하지 않았다.

KAN은 각 edge의 spline 출력을 합산하고 층별 bias를 더한다. 입력과 각 은닉 출력은 `tanh`로 spline grid `[-1, 1]` 범위에 매핑된다. 네 모델의 입력 및 표적 평균·스케일은 **학습 분할만** 사용해 산출한다. 기본 생성자는 평균 0, 스케일 1이며, 이 상태는 학습 완료를 뜻하지 않는다.

## 데이터 분할과 실행

`wide16_v3`의 T1–T12를 학습, T13–T14를 검증, T15–T16을 시험에 사용한다. 7조건 각각의 첫 완료 trial만 선택하며, 나머지 결정론적 반복을 독립 표본으로 세지 않는다. 기본 `--stride 25`는 100 Hz 기록에서 4 Hz 부분집합을 택한다. 준비 단계는 모든 7조건 × 16궤적이 완료됐는지, 품질 검사와 저장 XML이 토크 라벨을 재현하는지 확인한다. 비교 지표는 모든 시험 표본·6축의 RMSE [Nm]이며 조건별 결과도 저장한다. 라벨은 MuJoCo 역동역학으로, 실측 모터 토크가 아니다.

저장소 루트 PowerShell에서 전체 수집 완료 후 실행한다. 기존 출력 폴더가 있으면 덮어쓰지 않고 종료한다.

기본 학습 설정은 AdamW(학습률 0.002, weight decay 1e-5), 최대 120 epoch,
batch 2048, seed 42다. 검증 정규화 MSE가 `1e-6`보다 크게 개선되지 않은
상태가 10 epoch 이어지면 Early Stopping하며, 최적 검증 체크포인트를 저장한다.

```powershell
.\model\run_joint_models.ps1
```

출력은 `data/wide16_v3_joint_dataset/`, `data/wide16_v3_joint_models/M1..M4/`, `data/wide16_v3_joint_evaluation/`에 저장된다. `comparison.csv`와 `metrics.json`에 M0–M4의 검증·시험 결과를 기록한다.

## 호출 예시

```python
import torch
from model.residual_models import ResidualKAN, ResidualMLP, build_features, condition_features

q = torch.zeros(2, 6)
dq = torch.zeros(2, 6)
ddq = torch.zeros(2, 6)
condition = condition_features(["bare", "ag145"], torch.tensor([0.0, 0.5]))
x = build_features(q, dq, ddq, condition)
tau_nominal = torch.zeros(2, 6)  # 실제 사용 시 동일 상태의 명목 RBD 계산값

for network in (ResidualMLP(), ResidualKAN()):
    residual = network(x)
    tau_pred = network.corrected_torque(x, tau_nominal)
```

위 예시는 입출력 계약 확인용이다. 무작위 초기 가중치의 예측 토크를 물리적 결과나 학습 성능으로 해석하지 않는다. 기존 실험의 CSV·그림·지표는 보존하지만, 제거한 예전 체크포인트는 새 구조와 호환되지 않는다.
