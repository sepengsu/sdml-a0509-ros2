"""Read-only DRFL RT topic collection.

This module subscribes to RT data and publishes visualization markers. It
never creates a motion publisher, motion action client, or DRFL motion service
client. The --execute request is denied until all independent validation
evidence can be established.
"""

from __future__ import annotations

from dataclasses import replace
import math
import signal
from pathlib import Path
import subprocess
import time

from .dataset_exporter import create_trial, save_dynamics, save_metadata, save_trajectory
from .dynamics_logger import DynamicsRecorder, RT_FIELDS, TORQUE_DEFINITIONS
from .trajectory_generator import TrajectoryConfig, generate
from .trajectory_visualizer import save_dynamics_plot, save_trajectory_plot


def validate_real_execution(config: TrajectoryConfig, latest: dict) -> list[str]:
    """Return every failed preflight category. No unchecked category may pass."""
    reasons = []
    position = latest.get("actual_joint_position")
    velocity = latest.get("actual_joint_velocity")
    robot_state = latest.get("robot_state")
    if position is None or velocity is None or robot_state is None:
        reasons.append("connection: fresh RT position, velocity and robot_state data required")
    elif int(robot_state[0]) != 1:
        reasons.append("connection: robot must be in STANDBY state")
    if latest.get("control_authority_verified") is not True:
        reasons.append("control authority: controller ownership not independently verified")
    if latest.get("tcp_tool_payload_verified") is not True:
        reasons.append("TCP, tool and payload settings not independently verified")
    if latest.get("collision_workspace_singularity_verified") is not True:
        reasons.append("full-path collision, workspace and singularity validation unavailable")
    reasons.append("motion dispatch is disabled until a certified full-path validator and driver are integrated")
    if config.trajectory_id == "T4" and position is not None:
        q = tuple(math.radians(v) for v in position)
        limit = (3.14, 1.3, 2.0, 3.14, 2.0, 3.14)
        if any(abs(q[i]) + math.radians(config.joint_amplitude_deg) > limit[i] for i in config.joint_indices):
            reasons.append("joint position: proposed excitation exceeds conservative limits")
        if config.joint_start_rad and max(abs(q[i] - config.joint_start_rad[i]) for i in range(6)) > math.radians(2):
            reasons.append("current joint pose differs from trajectory start")
        if velocity is not None and max(abs(math.radians(v)) for v in velocity) > 0.05:
            reasons.append("robot must be stationary before starting")
        plan = generate(config)
        peak_velocity = [max(abs(point.dq_rad_s[i]) for point in plan.points) for i in range(6)]
        peak_acceleration = [max(abs(point.ddq_rad_s2[i]) for point in plan.points) for i in range(6)]
        velocity_limit = (3.67, 3.32, 3.67, 6.98, 6.98, 10.47)
        acceleration_limit = (0.734, 0.664, 0.734, 1.396, 1.396, 2.094)
        if any(peak_velocity[i] > velocity_limit[i] or peak_acceleration[i] > acceleration_limit[i]
               for i in range(6)):
            reasons.append("joint velocity or acceleration exceeds conservative limits")
    else:
        reasons.append("Cartesian full-path IK, position, velocity and acceleration validation unavailable")
        tcp = latest.get("actual_tcp_position")
        if tcp is not None:
            first = generate(config).points[0].xyz_mm
            if math.dist(tcp[:3], first) > 5.0:
                reasons.append("current TCP differs from trajectory start by more than 5 mm")
    if latest.get("operator_approved") is not True:
        reasons.append("final operator approval not recorded")
    return reasons


def execute(config: TrajectoryConfig, root: str, gripper_type: str, payload_mass: float | None,
            duration_s: float, requested_motion: bool, connection_timeout_s: float = 10.0,
            reference_explicit: bool = False) -> tuple[Path, bool]:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = None
    bag = None
    try:
        node = Node("a0509_dynamics_real_logger")
        recorder = DynamicsRecorder("real", config.trajectory_id, "pending", gripper_type, payload_mass,
                                    config.sample_hz, max_age_s=0.03)

        def on_rt(message: Float32MultiArray, key: str) -> None:
            try:
                recorder.ingest_rt(key, message.data)
            except ValueError as error:
                node.get_logger().warning(str(error))

        def on_joint_state(message: JointState) -> None:
            try:
                recorder.ingest_joint_state(message.name, message.position, message.velocity, message.effort)
            except ValueError:
                pass

        subscriptions = [node.create_subscription(Float32MultiArray, f"/rt_topic/{key}",
                                                  lambda msg, key=key: on_rt(msg, key), 100) for key in RT_FIELDS]
        subscriptions.append(node.create_subscription(JointState, "/joint_states", on_joint_state, 100))
        deadline = time.monotonic() + connection_timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if recorder.rt.get("actual_joint_position") and recorder.rt.get("actual_joint_velocity"):
                break
        else:
            raise RuntimeError("No RT position/velocity topics. Enable use_rt_topic_pub and 10 ms timer in dsr_controller2 before recording.")

        initial = {key: value[1] for key, value in recorder.rt.items()}
        if config.trajectory_id == "T4":
            config = replace(config, joint_start_rad=tuple(math.radians(v) for v in initial["actual_joint_position"]))
        elif "actual_tcp_position" in initial and not reference_explicit:
            config = replace(config, reference_tcp_mm=tuple(initial["actual_tcp_position"][:3]))
        plan = generate(config)
        folder, trial_id = create_trial(root, "real", config.trajectory_id)
        recorder.trial_id = trial_id
        save_trajectory(plan, folder)
        save_trajectory_plot(plan, folder)
        bag_path = folder / "rosbag"
        bag = subprocess.Popen(["ros2", "bag", "record", "-o", str(bag_path), "/joint_states",
                                "/a0509_dynamics_experiment/trajectory_markers",
                                *[f"/rt_topic/{key}" for key in RT_FIELDS]])
        rclpy.spin_once(node, timeout_sec=0.25)
        if bag.poll() is not None:
            raise RuntimeError(f"ros2 bag record exited early with code {bag.returncode}")
        from .simulation_executor import _markers
        marker_publisher = _markers(node, plan)
        blocked_reasons = validate_real_execution(config, initial) if requested_motion else []
        if requested_motion:
            node.get_logger().error("Real movement blocked: " + "; ".join(blocked_reasons))
        start = time.monotonic()
        next_sample = start
        while time.monotonic() - start < duration_s:
            rclpy.spin_once(node, timeout_sec=min(0.01, max(0, next_sample - time.monotonic())))
            now = time.monotonic()
            if now >= next_sample:
                recorder.snapshot(time.time(), node.get_clock().now().nanoseconds / 1e9, arrival_s=now)
                next_sample += 1.0 / config.sample_hz
        if bag.poll() is not None:
            raise RuntimeError(f"ros2 bag record exited during logging with code {bag.returncode}")
        bag.send_signal(signal.SIGINT)
        bag.wait(timeout=20)
        bag = None
        if not (bag_path / "metadata.yaml").is_file():
            raise RuntimeError("ROS 2 Bag metadata missing after recorder stopped")
        save_dynamics(recorder.rows, folder)
        measured_tcp = [row["tcp_position"] for row in recorder.rows if row.get("tcp_position") is not None]
        measured_joint = [tuple(row[f"q{i}"] for i in range(1, 7)) for row in recorder.rows
                          if all(row.get(f"q{i}") is not None for i in range(1, 7))]
        measured_times = [row["timestamp"] - recorder.rows[0]["timestamp"] for row in recorder.rows
                          if all(row.get(f"q{i}") is not None for i in range(1, 7))]
        save_trajectory_plot(plan, folder, measured_xyz_mm=measured_tcp,
                             measured_joint_rad=measured_joint, measured_time_s=measured_times)
        save_dynamics_plot(recorder.rows, folder)
        save_metadata(folder, plan, "real", trial_id, {
            "quality": recorder.quality(), "rosbag": str(bag_path),
            "gripper_type": gripper_type, "payload_mass_kg": payload_mass,
            "tool_model_status": ("mDPG mass/inertia, mount, TCP and pneumatic IO are not independently validated"
                                  if gripper_type == "mdpg_1s1" else None),
            "rt_source": "dsr_controller2 /rt_topic/<key> Float32MultiArray (10 ms configured, arrival-time joined)",
            "raw_rt_units": {"actual_joint_position": "deg", "actual_joint_velocity": "deg/s",
                             "actual_motor_torque": "Nm, motor-current/gear conversion",
                             "actual_joint_torque": "Nm, controller estimate",
                             "external_joint_torque": "Nm, controller estimate",
                             "actual_tcp_position": "mm for XYZ; deg for ZYZ orientation",
                             "target_joint_position": "deg", "joint_temperature": "C"},
            "acceleration_method": "finite difference of recent RT joint velocity; first and missing samples blank",
            "torque_definitions": TORQUE_DEFINITIONS, "requested_motion": requested_motion,
            "motion_blocked": requested_motion, "blocking_reasons": blocked_reasons,
            "movement_commands_sent": 0,
        })
        return folder, bool(blocked_reasons)
    finally:
        if bag is not None and bag.poll() is None:
            bag.send_signal(signal.SIGINT)
            try: bag.wait(timeout=10)
            except subprocess.TimeoutExpired: bag.terminate()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
