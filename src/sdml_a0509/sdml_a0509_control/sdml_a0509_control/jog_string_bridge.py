import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dsr_msgs2.msg import JogMultiAxis


class JogStringBridge(Node):
    def __init__(self):
        super().__init__("jog_string_bridge")
        self.declare_parameter("input_topic", "/a0509/jog_multi_axis")
        self.declare_parameter("output_topic", "/a0509/jog_multi_axis_cmd")

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        self.pub = self.create_publisher(JogMultiAxis, output_topic, 10)
        self.sub = self.create_subscription(String, input_topic, self.callback, 10)
        self.get_logger().info("Bridging std_msgs/String JSON %s -> JogMultiAxis %s" % (input_topic, output_topic))

    def callback(self, msg: String):
        try:
            payload = json.loads(msg.data or "{}")
            out = JogMultiAxis()
            axis = payload.get("jog_axis", [0.0] * 6)
            out.jog_axis = [float(v) for v in axis[:6]]
            if len(out.jog_axis) < 6:
                out.jog_axis += [0.0] * (6 - len(out.jog_axis))
            out.move_reference = int(payload.get("move_reference", 0))
            out.speed = float(payload.get("speed", 0.0))
        except Exception as exc:
            self.get_logger().warn("Invalid jog JSON ignored: %s" % exc)
            return
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JogStringBridge()
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
