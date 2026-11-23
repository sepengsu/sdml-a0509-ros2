# ROS2 마찰 검증 v3 실행 기록 — 2026-11-23

검증 identity는 `9056edddd7aae916b205ae98115d9f9eecfe62f1918fe81dabb63e31a0661aab`다. 원본 모델·데이터를 보존한 별도 출력에서 실행했다.

|검증|Windows Python 3.12|Ubuntu/ROS2 Humble Python 3.10|
|---|---|---|
|수식·힘 평형·LuGre 이력·중복 적용·잠금·도구 선택 테스트|13개 통과|13개 통과|
|원본 궤적 및 이력|336청크 /1,317,456상태 재현|동일 범위 재현|
|Offline|10,752상태×15조건 통과|동일 범위 통과|
|30초 실제 적분 pilot|baseline+15조건, 16/16 통과|baseline+15조건, 16/16 통과|
|452초 전체 T1|baseline와 Coulomb low, 2/2 통과|baseline, 1/1 통과|
|실제 ROS2 등록·실행|동일 구현의 Python 진입점 사용|colcon 빌드 후 ros2 run 실행 통과|

MuJoCo=3.2.7, NumPy=1.26.4다. Offline 원본 추가 토크 대비 최대 차이는 양쪽 모두 **2.979892066434786e-8 Nm**, 추가 힘/토크의 부호·중복 항등식 잔차는 **3.419486915845482e-14 Nm**였다. 전체 길이 실행의 최대 힘 평형/inverse 모터 토크 잔차는 **2.84e-14 Nm**, 포화 비율은 0이었다. 각 전체 실행은 226,000 물리 step과 45,201 저장 행을 가진다.

Pilot의 최대 힘 평형 잔차는 Windows 5.33e-15 Nm, Ubuntu 7.11e-15 Nm였다. 실제 qacc를 기록했으며 가속도 가이드라인 초과 여부를 metadata에 별도 저장했다. Ubuntu 검증은 네트워크를 차단하고 3 GiB 메모리·2 CPU로 제한한 임시 컨테이너에서 실행했다. worker 최대 관측 메모리는 Windows 약 217 MiB, Ubuntu 약 238 MiB였다. 이 수치는 다른 장치나 전체 조건의 최대 사용량을 보장하지 않는다.

실행 결과는 워크스페이스의 `output/friction_engine_ros2_verify_v3` 및 `output/friction_ros2_linux/results`에 있다. 입력 H:는 Linux 검증에서 읽기 전용으로 마운트했다. 그림의 렌더링과 저장 파일·체크섬을 확인했다. Linux의 혼합 matplotlib 설치 경고는 Axes3D에 관한 것이었으며, 사용하는 2D 그림과 검증 실행은 완료됐다.

전체 1,792회 캠페인은 아직 실행하지 않았고 `training_ready=false`다. Windows와 Linux의 같은 결정론적 검증을 독립적인 학습 반복으로 중복 집계하지 않는다. 3종 무하중 수집 범위는 각 256회, 합계 768회이며 전체 7개 도구·하중 조건은 1,792회다.

모델 감쇠 800/600, 원본 모터 제한, T1–T16의 기하학적 정의는 변경하지 않았다. 모든 마찰 조건의 목표와 150/20 제어기는 동일하다. 계수·수식·원본 생성기 해시·실행법은 [구현 문서](FRICTION_ROS2_V3.md)를 따른다.
