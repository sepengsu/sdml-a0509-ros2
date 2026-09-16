import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, String
from dsr_msgs2.msg import JogMultiAxis
import websocket


GRIPPER_OPEN_POS = 0.0
GRIPPER_CLOSED_POS = 0.93


def _dict_to_jog_multi_axis(d: dict) -> JogMultiAxis:
    msg = JogMultiAxis()
    ax = d.get("jog_axis", [0.0] * 6)
    msg.jog_axis = [float(x) for x in ax][:6]
    if len(msg.jog_axis) < 6:
        msg.jog_axis += [0.0] * (6 - len(msg.jog_axis))
    msg.move_reference = int(d.get("move_reference", 0))
    msg.speed = float(d.get("speed", 0.0))
    return msg


def _dict_to_pick_and_place(d: dict) -> String:
    msg = String()
    msg.data = str(d.get("go", "") or d.get("object_name", "")).strip()
    return msg


def _dict_to_float64_multi_array(d: dict) -> Float64MultiArray:
    msg = Float64MultiArray()
    if any(k in d for k in ("open_percent", "gripper", "gripper_open_percent")):
        open_percent = d.get("open_percent", d.get("gripper", d.get("gripper_open_percent", 100.0)))
        p = max(0.0, min(100.0, float(open_percent)))
        msg.data = [GRIPPER_CLOSED_POS - (p / 100.0) * (GRIPPER_CLOSED_POS - GRIPPER_OPEN_POS)]
        return msg
    command = str(d.get("command", d.get("action", ""))).strip().lower()
    if command in ("grip", "close", "closed"):
        msg.data = [GRIPPER_CLOSED_POS]
        return msg
    if command in ("ungrip", "open", "opened", "release"):
        msg.data = [GRIPPER_OPEN_POS]
        return msg
    data = d.get("data", d.get("position", d.get("positions", [])))
    if isinstance(data, (int, float)):
        data = [data]
    msg.data = [float(x) for x in data]
    return msg


def _float_or_zero(x):
    """NaN/Inf → 0.0 so JSON is valid and ROS 1 / Unity can parse."""
    if x is None or (isinstance(x, float) and (math.isnan(x) or math.isinf(x))):
        return 0.0
    return float(x)


def _parse_downlink_json(raw, topic_name, logger):
    """Parse JSON from server. Tolerates outer single quotes and single-quoted keys."""
    if raw is None:
        raw = "{}"
    if isinstance(raw, dict):
        return raw
    s = str(raw).strip()
    if not s:
        return {}
    # 전체가 '...' 로 한 번 더 감싸져 오는 경우 (rosbridge/서버 직렬화)
    if len(s) >= 2 and s[0] == "'" and s[-1] == "'":
        s = s[1:-1].replace("\\'", "'")
    # Unity/서버에서 큰따옴표가 \\u0022(유니코드 이스케이프)로 오는 경우 → " 로 복원
    s = s.replace("\\u0022", '"')
    try:
        return json.loads(s)
    except (json.JSONDecodeError, TypeError) as e:
        # 키가 single-quote인 경우 ' → " 로 바꿔 재시도
        try:
            return json.loads(s.replace("'", '"'))
        except (json.JSONDecodeError, TypeError):
            logger.warn("Downlink JSON 파싱 실패 %s: %s (수신 앞 80자: %r)" % (topic_name, e, s[:80]))
            return None


def _joint_state_to_dict(msg: JointState) -> dict:
    # ROS 1 / rosbridge expects "nsec" (not "nanosec")
    return {
        "header": {
            "stamp": {"sec": int(msg.header.stamp.sec), "nsec": int(msg.header.stamp.nanosec)},
            "frame_id": msg.header.frame_id or "",
        },
        "name": list(msg.name),
        "position": [_float_or_zero(x) for x in msg.position],
        "velocity": [_float_or_zero(x) for x in msg.velocity],
        "effort": [_float_or_zero(x) for x in msg.effort],
    }


def _image_to_dict(msg: Image) -> dict:
    return {
        "header": {
            "stamp": {"sec": int(msg.header.stamp.sec), "nsec": int(msg.header.stamp.nanosec)},
            "frame_id": msg.header.frame_id or "",
        },
        "height": int(msg.height),
        "width": int(msg.width),
        "encoding": msg.encoding,
        "is_bigendian": int(msg.is_bigendian),
        "step": int(msg.step),
        "data": list(msg.data),
    }


class ServerBridgeNode(Node):
    # 서버(ROS 1)에서 구독하여 로봇(ROS 2)으로 뿌려줄 토픽들 (Downlink)
    # 서버는 std_msgs/String(JSON)만 사용. remote_topic은 서버/Unity 쪽 이름,
    # local_topic은 현재 ROS 2 bringup 내부 이름이다.
    SUBSCRIBE_TOPICS = [
        {
            "remote_topic": "/do_pick_and_place",
            "local_topic": "/do_pick_and_place",
            "type": "std_msgs/String",
            "msg_class": String,
        },
        {
            "remote_topic": "/jog_multi_axis",
            "local_topic": "/a0509/jog_multi_axis_cmd",
            "type": "std_msgs/String",
            "msg_class": JogMultiAxis,
        },
        {
            "remote_topic": "/a0509/jog_multi_axis",
            "local_topic": "/a0509/jog_multi_axis_cmd",
            "type": "std_msgs/String",
            "msg_class": JogMultiAxis,
        },
        {
            "remote_topic": "/a0509/jog_multi_axis_cmd",
            "local_topic": "/a0509/jog_multi_axis_cmd",
            "type": "std_msgs/String",
            "msg_class": JogMultiAxis,
        },
        {
            "remote_topic": "/gripper_command_topic",
            "local_topic": "/ag145/forward_position_controller/commands",
            "type": "std_msgs/String",
            "msg_class": Float64MultiArray,
        },
        {
            "remote_topic": "/ag145/forward_position_controller/commands",
            "local_topic": "/ag145/forward_position_controller/commands",
            "type": "std_msgs/String",
            "msg_class": Float64MultiArray,
        },
    ]

    # 로봇(ROS 2)에서 구독하여 서버(ROS 1)로 던져줄 토픽들 (Uplink)
    PUBLISH_TOPICS = [
        {
            "local_topic": "/moveit/joint_states",
            "remote_topic": "/joint_states",
            "type": "sensor_msgs/JointState",
            "msg_class": JointState,
        },
        {
            "local_topic": "/a0509/joint_states",
            "remote_topic": "/a0509/joint_states",
            "type": "sensor_msgs/JointState",
            "msg_class": JointState,
        },
        {
            "local_topic": "/ag145_gripper/joint_states",
            "remote_topic": "/ag145_gripper/joint_states",
            "type": "sensor_msgs/JointState",
            "msg_class": JointState,
        },
        {
            "local_topic": "/ag145/joint_states",
            "remote_topic": "/ag145/joint_states",
            "type": "sensor_msgs/JointState",
            "msg_class": JointState,
        },
        {
            "local_topic": "/a0509/camera/image_raw",
            "remote_topic": "/a0509/camera/image_raw",
            "type": "sensor_msgs/Image",
            "msg_class": Image,
            "qos": qos_profile_sensor_data,
        },
    ]

    _DOWNLINK_CONVERTERS = {
        "/do_pick_and_place": _dict_to_pick_and_place,
        "/jog_multi_axis": _dict_to_jog_multi_axis,
        "/a0509/jog_multi_axis": _dict_to_jog_multi_axis,
        "/a0509/jog_multi_axis_cmd": _dict_to_jog_multi_axis,
        "/gripper_command_topic": _dict_to_float64_multi_array,
        "/ag145/forward_position_controller/commands": _dict_to_float64_multi_array,
    }
    _GRIPPER_COMMAND_TOPICS = {
        "/gripper_command_topic",
        "/ag145/forward_position_controller/commands",
    }
    _UPLINK_TOPIC_TYPES = {item["remote_topic"]: item["type"] for item in PUBLISH_TOPICS}
    _UPLINK_CONVERTERS = {
        "sensor_msgs/JointState": _joint_state_to_dict,
        "sensor_msgs/Image": _image_to_dict,
    }

    def __init__(self):
        super().__init__("server_bridge_node")
        self.server_url = "ws://115.145.167.47:9090"
        self._ws = None
        self._ws_ready = threading.Lock()

        # 1. ROS 2 퍼블리셔 (서버 -> 로봇, Downlink)
        self.local_publishers = {}
        for item in self.SUBSCRIBE_TOPICS:
            self.local_publishers[item["remote_topic"]] = self.create_publisher(
                item["msg_class"], item["local_topic"], 10
            )
        self._downlink_local_topics = {
            item["remote_topic"]: item["local_topic"] for item in self.SUBSCRIBE_TOPICS
        }

        # 2. ROS 2 구독 (로봇 -> 서버, Uplink) — 콜백에서 ws.send
        self._remote_subscribers = {}
        for item in self.PUBLISH_TOPICS:
            local_topic = item["local_topic"]
            remote_topic = item["remote_topic"]
            self._remote_subscribers[local_topic] = self.create_subscription(
                item["msg_class"],
                local_topic,
                lambda m, t=remote_topic: self._uplink_cb(t, m),
                item.get("qos", 10),
            )

        # 3. WebSocket 스레드 (재연결 포함)
        self._run_ws_forever()

    def _run_ws_forever(self):
        def run():
            while rclpy.ok():
                self.ws = websocket.WebSocketApp(
                    self.server_url,
                    on_open=self._on_open,
                    on_message=self._on_message,
                    on_error=self._on_error,
                    on_close=self._on_close,
                )
                self.get_logger().info("연결 시도: %s" % self.server_url)
                self.ws.run_forever()
                self._on_close(self.ws, None, None)
                if not rclpy.ok():
                    break
                time.sleep(2.0)

        self.ws_thread = threading.Thread(target=run, daemon=True)
        self.ws_thread.start()

    def _on_open(self, ws):
        self._ws = ws
        self.get_logger().info("서버 연결 성공.")
        # Downlink: 서버에서 받을 토픽 구독
        for item in self.SUBSCRIBE_TOPICS:
            subscribe_msg = {
                "op": "subscribe",
                "topic": item["remote_topic"],
                "type": item["type"],
            }
            try:
                ws.send(json.dumps(subscribe_msg))
                self.get_logger().info(
                    "구독 시작: %s -> %s" % (item["remote_topic"], item["local_topic"])
                )
            except Exception as e:
                self.get_logger().error("구독 전송 실패 %s: %s" % (item["remote_topic"], e))
        # Uplink: 로봇→서버 토픽은 서버에 먼저 advertise (rosbridge는 advertise 후에만 publish 가능)
        for item in self.PUBLISH_TOPICS:
            advertise_msg = {
                "op": "advertise",
                "topic": item["remote_topic"],
                "type": item["type"],
            }
            try:
                ws.send(json.dumps(advertise_msg))
                self.get_logger().info(
                    "advertise: %s <- %s (%s)"
                    % (item["remote_topic"], item["local_topic"], item["type"])
                )
            except Exception as e:
                self.get_logger().error("advertise 전송 실패 %s: %s" % (item["remote_topic"], e))

    def _on_message(self, ws, message):
        try:
            data = json.loads(message)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().warn("JSON 파싱 실패: %s" % e)
            return
        if data.get("op") != "publish":
            return
        topic_name = data.get("topic")
        msg_content = data.get("msg")
        if topic_name is None or msg_content is None:
            return
        if topic_name not in self.local_publishers:
            return
        converter = self._DOWNLINK_CONVERTERS.get(topic_name)
        if converter is None:
            self.get_logger().warn("변환기 없음: %s" % topic_name)
            return
        # 서버는 std_msgs/String 전송 → msg.data 에 JSON 문자열 또는 "go" 트리거
        raw = msg_content.get("data") if isinstance(msg_content, dict) else msg_content
        if isinstance(raw, dict):
            payload_dict = raw
        elif topic_name == "/do_pick_and_place":
            s = str(raw).strip().strip("'").strip('"')
            if s == "go":
                payload_dict = {"go": "go"}
            else:
                payload_dict = _parse_downlink_json(raw, topic_name, self.get_logger())
                if payload_dict is None:
                    return
        else:
            payload_dict = _parse_downlink_json(raw, topic_name, self.get_logger())
            if payload_dict is None:
                return
        try:
            ros_msg = converter(payload_dict)
            self.local_publishers[topic_name].publish(ros_msg)
            local_topic = self._downlink_local_topics.get(topic_name, topic_name)
            if topic_name in self._GRIPPER_COMMAND_TOPICS:
                self.get_logger().info(
                    "Gripper downlink received: %s -> %s, payload=%s, joint=%s"
                    % (topic_name, local_topic, payload_dict, list(ros_msg.data))
                )
            self.get_logger().debug("Downlink publish %s -> %s" % (topic_name, local_topic))
        except Exception as e:
            self.get_logger().error("메시지 변환/발행 실패 %s: %s" % (topic_name, e))

    def _uplink_cb(self, topic: str, msg):
        w = getattr(self, "_ws", None) or getattr(self, "ws", None)
        if w is None:
            return
        try:
            msg_type = self._UPLINK_TOPIC_TYPES.get(topic, "")
            converter = self._UPLINK_CONVERTERS.get(msg_type)
            if converter is None:
                self.get_logger().warn("Uplink 변환기 없음 %s (%s)" % (topic, msg_type))
                return
            payload = {
                "op": "publish",
                "topic": topic,
                "type": msg_type,
                "msg": converter(msg),
            }
            w.send(json.dumps(payload))
        except Exception as e:
            self.get_logger().warn("Uplink 전송 실패 %s: %s" % (topic, e))

    def _on_error(self, ws, error):
        self.get_logger().error("에러: %s" % (error,))

    def _on_close(self, ws, close_status_code, close_msg):
        self._ws = None
        self.get_logger().warn("서버와의 연결이 종료되었습니다.")


def main(args=None):
    rclpy.init(args=args)
    node = ServerBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
