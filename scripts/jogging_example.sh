echo "[jogging_example] /jog_multi_axis 토픽으로 예시 조그 명령 발행 (앞으로 X 방향 jog)..."
ros2 topic pub /jog_multi_axis dsr_msgs2/msg/JogMultiAxis \
  "{jog_axis: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0], move_reference: 0, speed: 20.0}" \
  --node-name jogging_pub -r 5 --times 20

echo "[jogging_example] 조그 예시 완료. jogging 노드 종료..."
kill "${JOGGING_PID}" 2>/dev/null || true

echo "[jogging_example] 끝."

