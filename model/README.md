# 동역학 잔차 모델

현재 모델 정의는 [M0–M4 공동 비교](RESIDUAL_MODELS.md)에 있다. M0는 Nominal RBD이며, M1·M2는 직접 토크 예측, M3·M4는 RBD 잔차 예측이다. 네 신경망 모두 7조건을 함께 학습한다. 완료된 `data/grid_16_trajectories`에 대한 M0–M4 학습·평가는 [이전 궤적 결과](LEGACY_M0_M4_RESULTS.md)에 정리했다. 현재 구축 중인 `data/grid_16_wide_v3`는 별도 프로필이며 전체 수집 완료 후 [PowerShell 실행 스크립트](run_joint_models.ps1)로 새로 학습·평가해야 한다.

과거 PhysicsKAN 및 34입력 잔차 모델의 코드와 체크포인트는 제거했다. 기존 CSV, 지표, 그림은 연구 기록으로 남겼다. 이전 연구 설명은 [과거 결과 기록](LEGACY_RESULTS.md), 7조건 Fourier 결과는 [캠페인 기록](CAMPAIGN_7CONDITIONS.md)에 있다. 해당 문서의 예전 모델 학습·예측 명령은 현재 버전에서 실행할 수 없다.

모델 구조 테스트:

```powershell
model/.venv/Scripts/python.exe -m unittest model.tests.test_residual_models -v
```
