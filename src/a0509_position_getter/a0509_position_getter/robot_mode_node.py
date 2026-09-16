#!/usr/bin/env python3
"""
robot_mode_node: SetRobotMode를 DSr 컨트롤러에 전달하는 노드.
서비스 'set_robot_mode'를 제공하여 GUI( position_getter ) 등에서 수동/자동 모드 전환 가능.
"""
import sys
import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import SetRobotMode


class RobotModeNode(Node):
    def __init__(self):
        super().__init__("robot_mode_node")
        self.declare_parameter(
            "dsr_set_robot_mode_service",
            "/system/set_robot_mode",
        )
        dsr_srv = (
            self.get_parameter("dsr_set_robot_mode_service")
            .get_parameter_value()
            .string_value
        )
        self._dsr_client = self.create_client(SetRobotMode, dsr_srv)

        self._server = self.create_service(
            SetRobotMode,
            "set_robot_mode",
            self._on_set_robot_mode,
        )
        self.get_logger().info(
            "robot_mode_node started. Forwarding set_robot_mode -> %s" % dsr_srv
        )

    def _on_set_robot_mode(self, request, response):
        if not self._dsr_client.wait_for_service(timeout_sec=1.0):
            response.success = False
            self.get_logger().warn("DSr set_robot_mode service not available.")
            return response
        req = SetRobotMode.Request()
        req.robot_mode = request.robot_mode
        future = self._dsr_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done() and future.result() is not None:
            response.success = future.result().success
            if response.success:
                self.get_logger().info(
                    "모드 전환 성공 (robot_mode=%d)" % request.robot_mode
                )
            else:
                self.get_logger().error("모드 전환 실패 (DSr 응답 success=false)")
        else:
            response.success = False
            self.get_logger().error("모드 전환 실패 (서비스 타임아웃/오류)")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotModeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
