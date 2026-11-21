# 7조건 Fourier T16 캠페인: RBD와 KAN

> 과거 실험 결과 기록. 당시 모델 코드와 체크포인트는 새 22입력 잔차 모델 구조로 전환하며 제거했다. 아래의 이전 모델 학습·예측 재현 명령은 현재 버전에서 실행할 수 없다. 기존 CSV·지표는 보존했다.

## 데이터와 분할

- 원본: `data/grid_16_fourier_t16_20261111/`. A0509 단독 0 kg, AG145 0/0.5/1.0 kg, mDPG 0/0.5/1.0 kg의 7조건 × T1~T16 × 3회 = **336 trial, 1,343,286행**이다. T16은 이전 chirp이 아닌 새 Fourier 궤적이다.
- 학습용 추출: `data/grid16_fourier_pkan_csv/`. 원본 100 Hz의 매 5번째 행과 각 궤적의 첫 완료 trial만 선택해 **89,642행**을 사용한다. 세 반복은 동일 설정의 결정론적 시뮬레이션이므로 독립적인 상태공간 표본으로 취급하지 않는다.
- 분할: T5~T11/T15 학습 31,766행, T12 검증 4,697행, T13/T14/T16 시험 11,431행, T1~T4 진단 41,748행. 시험과 진단 궤적은 학습과 체크포인트 선택에 사용하지 않는다.
- 입력은 `q`, `dq`, `ddq`; 토크 라벨은 MuJoCo `qfrc_inverse`다. 질량행렬 `M`, bias, damping도 내보낸다. `tau = M(q) ddq + bias(q,dq) + D dq`를 추출 시 검증한다. 실제 모터 토크 측정값은 없다.

## 모델

1. **장면 RBD:** 수집 때 저장된 조건별 MuJoCo XML에서 직접 역동역학을 계산한다. 같은 엔진과 같은 파라미터로 생성된 라벨이므로 오차가 수치 정밀도 수준인 것은 내부 재현 확인이지 독립적인 예측 성능이 아니다.
2. **명목 RBD:** 장면 XML의 그리퍼와 페이로드 질량·관성을 80%로 낮춘 합성 불일치 모델이다. 팔과 질량중심 위치는 유지한다. 단독 팔은 변경할 도구가 없어 장면 RBD와 같다.
3. **직접 PhysicsKAN:** 기존 Cholesky 양의 정부호 질량행렬 구조를 사용해 7조건을 **각각 따로** 학습했다. 토크, `M`, bias를 학습 분할에서 감독하고 T12로 체크포인트를 선택했다.
4. **RBD + 잔차 KAN:** 명목 RBD 토크에 하나의 조건 인식 KAN 잔차를 더한다. 입력은 `sin(q)`, `cos(q)`, `dq`, `ddq`, 7조건 원핫 코드다. 단독 팔의 잔차는 정의상 0으로 고정한다. 이 모델은 기존 7조건을 구별하며 새로운 그리퍼나 페이로드로의 외삽을 검증한 모델은 아니다.

## 시험 결과

T13/T14/Fourier T16의 모든 행과 6축을 합친 토크 RMSE [Nm]. 수치는 `data/grid16_fourier_model_comparison/comparison.csv`에 보존한다.

| 조건 | 명목 RBD | 직접 PhysicsKAN | RBD + 잔차 KAN |
|---|---:|---:|---:|
| bare, 0 kg | ≈0 | 0.530 | ≈0 |
| AG145, 0 kg | 0.467 | 1.244 | 0.138 |
| AG145, 0.5 kg | 0.672 | 1.745 | 0.177 |
| AG145, 1.0 kg | 0.877 | 1.922 | 0.255 |
| mDPG, 0 kg | 0.203 | 0.709 | 0.093 |
| mDPG, 0.5 kg | 0.385 | 1.640 | 0.116 |
| mDPG, 1.0 kg | 0.567 | 1.607 | 0.147 |
| **전체** | **0.528** | **1.431** | **0.151** |

직접 PhysicsKAN은 모든 조건의 시험 구간에서 명목 RBD보다 오차가 크다. 잔차 KAN은 전체 시험 오차를 줄이지만, **Fourier T16만 분리하면 모든 그리퍼·페이로드 조건에서 명목 RBD보다 악화**된다. 예를 들어 AG145 0 kg의 T16은 0.223 → 0.305 Nm다. 새로운 궤적에 대한 과보정 문제가 남아 있으므로 전체 시험 평균만으로 일반화를 주장하면 안 된다. `by_trajectory.csv`에 모든 궤적별 오차를 기록했다.

## 재현

저장소 루트의 PowerShell에서 실행한다. 기존 출력 경로에 다시 쓰기 전에 필요한 결과를 별도 경로로 보존한다. `export_grid16_csv`는 출력 폴더가 이미 있으면 덮어쓰지 않는다.

```powershell
$py = '.\model\.venv\Scripts\python.exe'
& $py -m model.export_grid16_csv --source data/grid_16_fourier_t16_20261111 --output data/grid16_fourier_pkan_csv_new
& $py -m model.campaign_rbd --data data/grid16_fourier_pkan_csv_new --output data/grid16_fourier_rbd_new
foreach ($condition in @('bare_p0','ag145_p0','ag145_p05','ag145_p10','mdpg_p0','mdpg_p05','mdpg_p10')) {
    & $py -m model.train_grid16 --data data/grid16_fourier_pkan_csv_new --condition $condition --output "data/grid16_fourier_pkan_models_new/$condition" --epochs 40
}
& $py -m model.train_campaign_residual --data data/grid16_fourier_pkan_csv_new --output data/grid16_fourier_residual_kan_new
& $py -m model.summarize_campaign_models --data data/grid16_fourier_pkan_csv_new --rbd data/grid16_fourier_rbd_new/metrics.json --pkan-root data/grid16_fourier_pkan_models_new --residual data/grid16_fourier_residual_kan_new/metrics.json --output data/grid16_fourier_model_comparison_new
```

임의 관절 상태의 오프라인 예측 예시:

```powershell
& .\model\.venv\Scripts\python.exe -m model.predict_campaign_models --condition ag145_p05 --q -0.5 -0.5 1.6 0 -0.6 0 --dq 0 0 0 0 0 0 --ddq 0 0 0 0 0 0
```

이 모델들은 MuJoCo 내부 역동역학을 근사하거나 재구성한다. 페이로드와 장착 변환은 시뮬레이션 가정이며, 실로봇 동역학 식별 또는 제어 안전성 검증 결과가 아니다.
