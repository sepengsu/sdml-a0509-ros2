from queue import Queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from sdml_a0509_control.position_based_scenerio import do_position_based_scenerio
from sdml_a0509_control.scenerio import do_scenerio
from sdml_a0509_control.utils import (
    get_do_scenario_path,
    get_pick_place_config_path,
    get_pick_place_times_path,
)
from sdml_a0509_control.gripper_driver import open_percent_to_pos
from sdml_a0509_control.position_move import configure_service_names

DEFAULT_GRIPPER_COMMAND_TOPIC = "/ag145/forward_position_controller/commands"


class PickAndPlaceExecutor(Node):
    def __init__(self):
        super().__init__("pick_and_place_executor")
        self.declare_parameter("joint_states_topic", "")
        self.declare_parameter("scenario_type", "pick_place")
        self.declare_parameter("do_pick_and_place_topic", "/do_pick_and_place")
        self.declare_parameter("gripper_command_topic", DEFAULT_GRIPPER_COMMAND_TOPIC)
        self.declare_parameter("move_line_service", "/motion/move_line")
        self.declare_parameter("move_joint_service", "/motion/move_joint")
        self.declare_parameter("move_circle_service", "/motion/move_circle")

        configure_service_names(
            move_line_service=self._param_string("move_line_service"),
            move_joint_service=self._param_string("move_joint_service"),
            move_circle_service=self._param_string("move_circle_service"),
        )

        self.sub = self.create_subscription(
            String,
            self._param_string("do_pick_and_place_topic"),
            self.callback,
            10,
        )
        self._gripper_pub = self.create_publisher(
            Float64MultiArray,
            self._param_string("gripper_command_topic"),
            10,
        )
        self.queue = Queue()
        self.executing = False
        self.timer = self.create_timer(0.3, self._process_queue_once)

    def _param_string(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def set_gripper(self, open_percent: float):
        pos = open_percent_to_pos(open_percent)
        msg = Float64MultiArray()
        msg.data = [float(pos)]
        self._gripper_pub.publish(msg)
        self.get_logger().info("Gripper command: %.1f%% open -> pos %.3f" % (open_percent, pos))

    def callback(self, msg):
        go_text = (msg.data or "").strip()
        self.get_logger().info("Received command: go=%s" % go_text)
        self.queue.put(go_text)

    def _process_queue_once(self):
        if self.executing or self.queue.empty():
            return
        cmd = self.queue.get_nowait()
        self.executing = True
        try:
            self.execute(cmd)
        finally:
            self.executing = False

    def execute(self, cmd):
        if (cmd or "").strip() != "go":
            self.get_logger().info("go=%s (ignored, only 'go' runs scenario)" % (cmd or ""))
            return

        scenario_type = self.get_parameter("scenario_type").get_parameter_value().string_value or "pick_place"
        scenario_type = (scenario_type or "pick_place").strip().lower()

        try:
            if scenario_type == "list":
                path = get_do_scenario_path()
                self.get_logger().info("Executing scenario (list) from %s" % path)
                success = do_position_based_scenerio(self, path)
            else:
                config_path = get_pick_place_config_path()
                times_path = get_pick_place_times_path()
                self.get_logger().info("Executing scenario (pick_place) from %s, times %s" % (config_path, times_path))
                success = do_scenerio(self, path=config_path, times_path=times_path)
            if success:
                self.get_logger().info("Scenario finished successfully.")
            else:
                self.get_logger().warn("Scenario finished with errors.")
        except FileNotFoundError as e:
            self.get_logger().error("시나리오 파일을 찾을 수 없습니다: %s" % e)
        except Exception as e:
            self.get_logger().error("Scenario failed: %s" % e)


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceExecutor()
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

