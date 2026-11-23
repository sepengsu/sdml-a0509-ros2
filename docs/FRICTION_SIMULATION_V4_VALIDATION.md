# 독립 MuJoCo 시뮬레이션 v4 검증 — 2026-11-23

검증 identity는 `e98eb7f11599c4a3b574042f6c4456d8cf85853ad1b94c988157548337aca158`다.

마찰 시뮬레이터는 독립 패키지 `a0509_friction_sim`으로 이동했다. 잘못 추가했던 sim-to-real 패키지의 실행 등록·의존성·문서는 원상 복구했고, 새 기본 실행은 `scripts/run_friction_simulation.ps1`이다. 수집에 ROS2와 Docker가 필요하지 않다.

|검증|Windows PowerShell 5.1 / Python 3.12|Ubuntu Python 3.10|
|---|---|---|
|수식·힘 평형·LuGre 이력·잠금·도구 선택 등|14개 테스트 통과|14개 테스트 통과|
|실로봇·ROS 모듈 격리|독립 프로세스에서 import 없음 확인|동일 확인|
|원본 궤적 및 이력|336청크 / 1,317,456상태 재현|동일 범위 재현|
|Offline|10,752상태×15조건 통과|동일 범위 통과|
|30초 실제 적분 pilot|baseline+15조건, 16/16 통과|baseline+15조건, 16/16 통과|
|452초 전체 T1 baseline|1/1 통과|1/1 통과|
|PowerShell 5.1 한글 경로|UTF-8 BOM·구문·기본 F: 경로 검사 통과|해당 없음|

MuJoCo=3.2.7, NumPy=1.26.4다. Offline 원본 추가 토크 대비 최대 차이는 **2.979892066434786e-8 Nm**, 중복·부호 항등식 잔차는 **3.419486915845482e-14 Nm**였다. 전체 길이 실행의 최대 힘 평형/inverse 모터 토크 잔차는 **2.84e-14 Nm**, 포화 비율은 0이었다. 전체 실행당 226,000 물리 step, 45,201 저장 행을 기록했다.

14개 테스트 중 하나는 별도 프로세스에서 시뮬레이터를 불러온 뒤 sim-to-real 패키지, rclpy, 실로봇 메시지 모듈이 import되지 않았음을 확인한다. 시뮬레이터는 수학적 T1–T16 정의와 모델 자산만 재사용한다. 실제 ROS 노드나 실로봇 제어기를 실행하지 않는다.

Windows는 새 `.venv-friction-sim`에서 직접 실행했다. Ubuntu는 검증 환경을 격리하기 위해 메모리 3 GiB·2 CPU·네트워크 차단 임시 컨테이너를 사용했으나, 실행 스크립트는 ROS2 환경을 source하지 않고 일반 Python으로 모든 단계를 수행했다. worker 최대 관측 메모리는 Windows 약 217 MiB, Ubuntu 약 237 MiB였다. 수집 자체에는 컨테이너가 필요하지 않다.

결과는 `output/friction_sim_v4_verify`와 `output/friction_sim_v4_linux/results`에 저장했다. 원본 모델·H: 데이터·기존 v2/v3 결과는 보존했다. 두 OS에서 실행한 동일한 결정론적 검증은 독립 학습 반복으로 중복 집계하지 않는다.

전체 1,792회 캠페인은 아직 수행하지 않았으며 `training_ready=false`다. 각 무하중 도구별 범위는 256회, 3종 합계 768회다. 모델 감쇠 800/600과 원본 모터 제한, 제어기 150/20은 유지했다.

[분석·PowerShell·Ubuntu 실행 안내](FRICTION_SIMULATION_V4.md)
