from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class VirtualCameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__("sdml_virtual_camera_publisher")
        self.declare_parameter("image_topic", "/a0509/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/a0509/camera/camera_info")
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15.0)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.width = max(1, int(self.get_parameter("width").value))
        self.height = max(1, int(self.get_parameter("height").value))
        self.fps = max(0.1, float(self.get_parameter("fps").value))
        self.frame_data = self.make_rgb8_blue_frame()

        self.image_pub = self.create_publisher(Image, self.image_topic, 10)
        self.info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)
        self.get_logger().info(f"Publishing virtual blue RGB image on {self.image_topic}")

    def publish_frame(self) -> None:
        stamp = self.get_clock().now().to_msg()

        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id
        image.height = self.height
        image.width = self.width
        image.encoding = "rgb8"
        image.is_bigendian = False
        image.step = self.width * 3
        image.data = self.frame_data

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.height = self.height
        info.width = self.width
        focal = float(self.width)
        cx = (self.width - 1.0) / 2.0
        cy = (self.height - 1.0) / 2.0
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.image_pub.publish(image)
        self.info_pub.publish(info)

    def make_rgb8_blue_frame(self) -> bytes:
        # Solid blue in rgb8: R=0, G=0, B=255
        return bytes([0, 0, 255]) * (self.width * self.height)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = VirtualCameraPublisher()
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
