# A0509 friction simulation

독립적인 MuJoCo 시뮬레이션 전용 패키지다. 실로봇 연결, ROS 노드·토픽 송수신,
sim-to-real 실행기는 사용하지 않는다. 모델과 T1–T16의 수학적 정의만 재사용한다.

기본 실행은 저장소 루트에서 `scripts/run_friction_simulation.ps1` 또는
`python scripts/collect_friction_simulation.py`다. Python 가상환경만 있으면
ROS2와 Docker 없이 실행할 수 있다.

Ubuntu에서도 `python3 scripts/collect_friction_simulation.py --help`로 같은
시뮬레이터를 실행한다. ROS2 워크스페이스를 사용한다면 독립 패키지로 빌드한 뒤
`ros2 run a0509_friction_sim collect_friction --help`로 호출할 수도 있다.
이 호출 방식도 MuJoCo만 실행하며 로봇 제어 기능을 포함하지 않는다.

원본 damping 800/600과 모델·데이터를 보존하고 별도의 `friction_sim_v4`
출력에 기록한다. Analyze → Offline → Pilot → Collect 순서로 검증한다.

[분석·PowerShell·Ubuntu 실행 안내](../../docs/FRICTION_SIMULATION_V4.md)
