#!/usr/bin/env python3
import csv
import os
import rclpy
from rclpy.node import Node

class GraspSweepNode(Node):
    def __init__(self):
        super().__init__("grasp_sweep")

        self.declare_parameter("scene_path", "")
        self.declare_parameter("results_csv", "results/grasp_results.csv")
        self.declare_parameter("grasp_z_list", [0.203, 0.212, 0.221])
        self.declare_parameter("yaw_deg_list", [0.0, 45.0, 90.0])
        self.declare_parameter("grip_force_list", [60.0, 100.0, 140.0])
        self.declare_parameter("lift_height", 0.05)
        self.declare_parameter("move_distance", 0.10)

        self.scene_path = self.get_parameter("scene_path").value
        self.results_csv = self.get_parameter("results_csv").value
        self.grasp_z_list = self.get_parameter("grasp_z_list").value
        self.yaw_deg_list = self.get_parameter("yaw_deg_list").value
        self.grip_force_list = self.get_parameter("grip_force_list").value
        self.lift_height = self.get_parameter("lift_height").value
        self.move_distance = self.get_parameter("move_distance").value

        os.makedirs(os.path.dirname(self.results_csv), exist_ok=True)
        self.run_experiments()

    def run_experiments(self):
        with open(self.results_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "exp_id", "grasp_z", "yaw_deg", "grip_force",
                "lift_success", "slip_mm", "rotation_deg",
                "move_success", "drop", "score"
            ])

            exp_id = 1
            for z in self.grasp_z_list:
                for yaw in self.yaw_deg_list:
                    for force in self.grip_force_list:
                        self.get_logger().info(
                            f"[EXP {exp_id}] z={z}, yaw={yaw}, force={force}"
                        )

                        result = self.evaluate_one_case(z, yaw, force)

                        writer.writerow([
                            exp_id, z, yaw, force,
                            result["lift_success"],
                            result["slip_mm"],
                            result["rotation_deg"],
                            result["move_success"],
                            result["drop"],
                            result["score"]
                        ])
                        exp_id += 1

        self.get_logger().info(f"Saved results to {self.results_csv}")

    def evaluate_one_case(self, z, yaw, force):
        # TODO:
        # 1. reset scene
        # 2. move robot/gripper to target pose
        # 3. close gripper with given force
        # 4. lift object
        # 5. move laterally
        # 6. measure slip / rotation / drop
        return {
            "lift_success": True,
            "slip_mm": 0.0,
            "rotation_deg": 0.0,
            "move_success": True,
            "drop": False,
            "score": 100.0
        }

def main():
    rclpy.init()
    node = GraspSweepNode()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()