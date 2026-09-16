#!/bin/bash
# "go" 트리거만 발행. pick_and_place 노드는 토픽의 start_pose/target_pose 를 쓰지 않고
# ~/.ros/pick_and_place/scenario_positions.json 한 파일에서 두 pose 를 읽어 scenario 실행.
# 예시: cp scripts/scenario_positions.example.json ~/.ros/pick_and_place/scenario_positions.json
# 사용: ./scenario.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [ -f "${ROOT_DIR}/install/setup.bash" ]; then
  source "${ROOT_DIR}/install/setup.bash"
fi

ros2 topic pub --once /do_pick_and_place std_msgs/msg/String "{data: 'go'}"
