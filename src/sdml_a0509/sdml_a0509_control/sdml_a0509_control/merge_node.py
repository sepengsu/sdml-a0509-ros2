import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from copy import deepcopy


class JointStateMerger(Node):
    def __init__(self):
        super().__init__("joint_state_merger")
        self.declare_parameter("arm_joint_states_topic", "/joint_states")
        self.declare_parameter("gripper_joint_states_topic", "/ag145_gripper/joint_states")
        self.declare_parameter("merged_topic", "/moveit/joint_states")
        self.declare_parameter("gripper_joint_prefix", "ag145_")

        arm_topic = self.get_parameter("arm_joint_states_topic").get_parameter_value().string_value or "/joint_states"
        gripper_topic = self.get_parameter("gripper_joint_states_topic").get_parameter_value().string_value or "/ag145/joint_states"
        merged_topic = self.get_parameter("merged_topic").get_parameter_value().string_value or "/moveit/joint_states"
        self.gripper_joint_prefix = self.get_parameter("gripper_joint_prefix").get_parameter_value().string_value or "ag145_"

        # Keep this list explicit so only known gripper joints are renamed.
        self.gripper_joint_names = {
            "left_outer_knuckle_joint",
            "left_inner_knuckle_joint",
            "left_finger_joint",
            "right_outer_knuckle_joint",
            "right_inner_knuckle_joint",
            "right_finger_joint",
        }

        self.sub_arm = self.create_subscription(JointState, arm_topic, self.cb_arm, 50)
        self.sub_gripper = self.create_subscription(JointState, gripper_topic, self.cb_gripper, 50)
        self.pub = self.create_publisher(JointState, merged_topic, 50)

        self.last_arm = None
        self.last_gripper = None
        self.timer = self.create_timer(0.02, self.publish_merged)

    def cb_arm(self, msg: JointState):
        self.last_arm = msg

    def cb_gripper(self, msg: JointState):
        mapped = deepcopy(msg)
        renamed = []
        for joint_name in mapped.name:
            if joint_name in self.gripper_joint_names and not joint_name.startswith(self.gripper_joint_prefix):
                renamed.append(f"{self.gripper_joint_prefix}{joint_name}")
            else:
                renamed.append(joint_name)
        mapped.name = renamed
        self.last_gripper = mapped

    def publish_merged(self):
        if self.last_arm is None and self.last_gripper is None:
            return

        merged = JointState()
        if self.last_arm is not None:
            merged.header = deepcopy(self.last_arm.header)
        elif self.last_gripper is not None:
            merged.header = deepcopy(self.last_gripper.header)

        name, position, velocity, effort = [], [], [], []

        def append_from(src: JointState):
            if src is None:
                return
            n = len(src.name)
            pos = list(src.position) if len(src.position) == n else [0.0] * n
            vel = list(src.velocity) if len(src.velocity) == n else [0.0] * n
            eff = list(src.effort) if len(src.effort) == n else [0.0] * n
            for i in range(n):
                name.append(src.name[i])
                position.append(pos[i])
                velocity.append(vel[i])
                effort.append(eff[i])

        append_from(self.last_arm)
        append_from(self.last_gripper)

        merged.name = name
        merged.position = position
        merged.velocity = velocity
        merged.effort = effort
        self.pub.publish(merged)


def main():
    rclpy.init()
    node = JointStateMerger()
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

