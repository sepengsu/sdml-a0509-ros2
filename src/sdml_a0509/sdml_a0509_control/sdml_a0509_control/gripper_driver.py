#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

TOPIC = "/ag145/forward_position_controller/commands"
MIN_POS = 0.0
MAX_POS = 0.93


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def open_percent_to_pos(open_percent: float) -> float:
    p = clamp(float(open_percent), 0.0, 100.0)
    return MAX_POS - (p / 100.0) * (MAX_POS - MIN_POS)


class GripperPublisher(Node):
    def __init__(self, value: float):
        super().__init__("ag145_one_shot_pub")
        self.pub = self.create_publisher(Float64MultiArray, TOPIC, 10)
        msg = Float64MultiArray()
        msg.data = [float(value)]
        self._publish_and_exit(msg=msg)

    def _publish_and_exit(self, msg: Float64MultiArray):
        self.pub.publish(msg)
        self.get_logger().info(f"Published to {TOPIC}: {msg.data}")


def set_position(per: float):
    pos = open_percent_to_pos(per)
    node = GripperPublisher(pos)
    rclpy.spin_once(node)
    node.destroy_node()

