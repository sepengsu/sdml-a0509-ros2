from typing import Optional

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class V4L2CameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__("sdml_v4l2_camera_publisher")
        self.declare_parameter("device", "/dev/video1")
        self.declare_parameter("image_topic", "/a0509/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/a0509/camera/camera_info")
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("width", 0)
        self.declare_parameter("height", 0)
        self.declare_parameter("fps", 15.0)

        self.device = str(self.get_parameter("device").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.requested_width = int(self.get_parameter("width").value)
        self.requested_height = int(self.get_parameter("height").value)
        self.fps = max(0.1, float(self.get_parameter("fps").value))
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera device: {self.device}")

        if self.requested_width > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.requested_width)
        if self.requested_height > 0:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.requested_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        self.image_pub = self.create_publisher(Image, self.image_topic, qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)
        self.get_logger().info(
            f"Publishing camera frames from {self.device} "
            f"({self.width}x{self.height}) on {self.image_topic}"
        )

    def publish_frame(self) -> None:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn(f"Failed to read frame from {self.device}")
            return

        stamp = self.get_clock().now().to_msg()
        image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image.header.stamp = stamp
        image.header.frame_id = self.frame_id

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.height = image.height
        info.width = image.width
        focal = float(image.width)
        cx = (image.width - 1.0) / 2.0
        cy = (image.height - 1.0) / 2.0
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.image_pub.publish(image)
        self.info_pub.publish(info)

    def destroy_node(self) -> bool:
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = V4L2CameraPublisher()
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
