"""Show the real Tk GUI layout with ROS calls stubbed for an offline preview."""

from __future__ import annotations

from pathlib import Path
import sys
import time
from types import ModuleType, SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/sim_real_gap_getting"))


class PreviewNode:
    def __init__(self, *args, **kwargs):
        pass

    def create_publisher(self, *args, **kwargs):
        return SimpleNamespace(publish=lambda message: None, get_subscription_count=lambda: 0)

    def create_subscription(self, *args, **kwargs):
        return object()

    def destroy_node(self):
        pass


def install_ros_stubs() -> None:
    rclpy = ModuleType("rclpy")
    rclpy.init = lambda: None
    rclpy.shutdown = lambda: None
    rclpy.spin_once = lambda node, timeout_sec=0.0: time.sleep(timeout_sec)
    node_module = ModuleType("rclpy.node")
    node_module.Node = PreviewNode
    parameter_module = ModuleType("rclpy.parameter")
    parameter_module.Parameter = lambda *args, **kwargs: object()
    sensor_module = ModuleType("sensor_msgs")
    sensor_msg_module = ModuleType("sensor_msgs.msg")
    sensor_msg_module.JointState = type("JointState", (), {})
    std_module = ModuleType("std_msgs")
    std_msg_module = ModuleType("std_msgs.msg")
    std_msg_module.Float64MultiArray = type("Float64MultiArray", (), {})
    sys.modules.update({
        "rclpy": rclpy, "rclpy.node": node_module, "rclpy.parameter": parameter_module,
        "sensor_msgs": sensor_module, "sensor_msgs.msg": sensor_msg_module,
        "std_msgs": std_module, "std_msgs.msg": std_msg_module,
    })


def main() -> None:
    install_ros_stubs()
    from tkinter import ttk
    from sim_real_gap_getting.gui import PracticeGui

    example = ROOT / "src/sim_real_gap_getting/examples/small_joint_1.json"
    gui = PracticeGui(str(example), real_domain_id=0, sim_gripper_type="none")
    gui.root.title("A0509 Sim → Real Gap Practice [Offline preview]")
    gui.status_var.set("OFFLINE PREVIEW — ROS 2 is not connected; controls are disabled.")

    def disable_controls(widget):
        for child in widget.winfo_children():
            if isinstance(child, (ttk.Button, ttk.Combobox)):
                child.configure(state="disabled")
            disable_controls(child)

    disable_controls(gui.root)
    gui.run()


if __name__ == "__main__":
    main()
