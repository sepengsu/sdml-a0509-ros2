from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraTopicRelay(Node):
    def __init__(self) -> None:
        super().__init__("sdml_camera_topic_relay")
        self.declare_parameter("input_image_topic", "/a0509/camera/color/image_raw")
        self.declare_parameter("output_image_topic", "/a0509/camera/image_raw")
        self.declare_parameter("input_camera_info_topic", "/a0509/camera/color/camera_info")
        self.declare_parameter("output_camera_info_topic", "/a0509/camera/camera_info")

        input_image_topic = str(self.get_parameter("input_image_topic").value)
        output_image_topic = str(self.get_parameter("output_image_topic").value)
        input_camera_info_topic = str(self.get_parameter("input_camera_info_topic").value)
        output_camera_info_topic = str(self.get_parameter("output_camera_info_topic").value)

        self.image_pub = self.create_publisher(Image, output_image_topic, qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, output_camera_info_topic, qos_profile_sensor_data)
        self.create_subscription(Image, input_image_topic, self.image_pub.publish, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, input_camera_info_topic, self.info_pub.publish, qos_profile_sensor_data)

        self.get_logger().info(f"Relaying image {input_image_topic} -> {output_image_topic}")
        self.get_logger().info(f"Relaying camera_info {input_camera_info_topic} -> {output_camera_info_topic}")


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = CameraTopicRelay()
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
