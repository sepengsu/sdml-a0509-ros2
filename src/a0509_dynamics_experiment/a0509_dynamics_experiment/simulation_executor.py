"""MuJoCo-only ROS 2 execution with an isolated ROS domain.

The existing a0509_mujoco_sim launch owns the actual simulator. A second
MuJoCo model is used solely for IK, FK and inverse-dynamics observation; it
never replaces or claims to read the simulator's private actuator state.
"""

from __future__ import annotations

from dataclasses import replace
import math
import os
from pathlib import Path
import signal
import subprocess
import time

from .dataset_exporter import create_trial, save_dynamics, save_metadata, save_trajectory
from .dynamics_logger import DynamicsRecorder, TORQUE_DEFINITIONS
from .trajectory_generator import JOINT_NAMES, TrajectoryConfig, generate
from .trajectory_visualizer import save_dynamics_plot, save_trajectory_plot


class MujocoObserver:
    def __init__(self, scene_path: str | Path | None = None, tcp_site: str = "ag145_tcp"):
        import mujoco
        import numpy as np

        self.mj = mujoco
        self.np = np
        if scene_path is None:
            from ament_index_python.packages import get_package_share_directory
            scene = Path(get_package_share_directory("dsr_description2")) / "mujoco_models" / "a0509" / "scene.xml"
        else:
            scene = Path(scene_path)
        if not scene.is_file():
            raise RuntimeError(f"A0509 MuJoCo scene not found: {scene}")
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.qpos_addr = []
        self.dof_addr = []
        for name in JOINT_NAMES:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise RuntimeError(f"MuJoCo model missing {name}")
            self.qpos_addr.append(int(self.model.jnt_qposadr[joint_id]))
            self.dof_addr.append(int(self.model.jnt_dofadr[joint_id]))
        self.body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link_6")
        if self.body_id < 0:
            raise RuntimeError("MuJoCo model missing link_6")
        self.tcp_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, tcp_site)
        # Conservative research limits from a0509_mujoco_sim/config/joint_limits.yaml.
        self.limits = [(-3.14, 3.14), (-1.3, 1.3), (-2.0, 2.0),
                       (-3.14, 3.14), (-2.0, 2.0), (-3.14, 3.14)]
        self.reference_rotation = None

    def _set_state(self, q, dq=None, ddq=None):
        self.data.qpos[:] = self.model.qpos0
        self.data.qvel[:] = 0
        self.data.qacc[:] = 0
        for i in range(6):
            self.data.qpos[self.qpos_addr[i]] = q[i]
            if dq is not None:
                self.data.qvel[self.dof_addr[i]] = dq[i]
        self.mj.mj_forward(self.model, self.data)
        if ddq is not None:
            for i in range(6):
                self.data.qacc[self.dof_addr[i]] = ddq[i]

    def tcp_mm(self, q):
        self._set_state(q)
        return tuple(float(x * 1000) for x in self._tcp_pos())

    def _tcp_pos(self):
        return self.data.site_xpos[self.tcp_site_id] if self.tcp_site_id >= 0 else self.data.xpos[self.body_id]

    def _tcp_rotation(self):
        matrix = self.data.site_xmat[self.tcp_site_id] if self.tcp_site_id >= 0 else self.data.xmat[self.body_id]
        return matrix.reshape(3, 3)

    def links_mm(self, q):
        self._set_state(q)
        result = [(0.0, 0.0, 0.0)]
        for i in range(1, 7):
            body = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_BODY, f"link_{i}")
            result.append(tuple(float(x * 1000) for x in self.data.xpos[body]))
        if self.tcp_site_id >= 0:
            result.append(tuple(float(x * 1000) for x in self._tcp_pos()))
        return result

    def set_ik_reference(self, q):
        self._set_state(q)
        self.reference_rotation = self._tcp_rotation().copy()

    def require_contact_free(self, q):
        self._set_state(q)
        if self.data.ncon:
            raise RuntimeError(f"MuJoCo model reports {self.data.ncon} contact(s); trajectory command blocked")
        return tuple(float(x * 1000) for x in self._tcp_pos())

    def solve_ik(self, xyz_mm, seed):
        np = self.np
        q = np.asarray(seed, dtype=float).copy()
        self._set_state(q)
        desired_rotation = (self.reference_rotation if self.reference_rotation is not None else
                            self._tcp_rotation().copy())
        target = np.asarray(xyz_mm, dtype=float) / 1000.0
        for _ in range(100):
            self._set_state(q)
            current = self._tcp_pos().copy()
            rotation = self._tcp_rotation()
            error_pos = target - current
            error_rot = 0.5 * sum((np.cross(rotation[:, i], desired_rotation[:, i]) for i in range(3)), np.zeros(3))
            if np.linalg.norm(error_pos) < 0.001 and np.linalg.norm(error_rot) < 0.035:
                return tuple(float(x) for x in q)
            jac_pos = np.zeros((3, self.model.nv))
            jac_rot = np.zeros((3, self.model.nv))
            if self.tcp_site_id >= 0:
                self.mj.mj_jacSite(self.model, self.data, jac_pos, jac_rot, self.tcp_site_id)
            else:
                self.mj.mj_jacBody(self.model, self.data, jac_pos, jac_rot, self.body_id)
            columns = self.dof_addr
            jacobian = np.vstack((jac_pos[:, columns], 0.3 * jac_rot[:, columns]))
            error = np.concatenate((error_pos, 0.3 * error_rot))
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 0.0025 * np.eye(6), error)
            q += np.clip(step, -0.08, 0.08)
            for i, (low, high) in enumerate(self.limits):
                q[i] = np.clip(q[i], low, high)
        raise RuntimeError(f"IK failed for {tuple(round(x, 2) for x in xyz_mm)} mm; simulation command blocked")

    def observe(self, q, dq=None, ddq=None):
        self._set_state(q, dq, ddq)
        positions = tuple(float(self.data.qpos[i]) for i in self.qpos_addr)
        velocities = tuple(float(self.data.qvel[i]) for i in self.dof_addr) if dq is not None else None
        acceleration = tuple(float(self.data.qacc[i]) for i in self.dof_addr) if ddq is not None else None
        bias = tuple(float(self.data.qfrc_bias[i]) for i in self.dof_addr)
        inverse = None
        if dq is not None and ddq is not None:
            self.mj.mj_inverse(self.model, self.data)
            inverse = tuple(float(self.data.qfrc_inverse[i]) for i in self.dof_addr)
        return {"qpos": positions, "qvel": velocities, "qacc": acceleration,
                "qfrc_inverse": inverse, "qfrc_actuator": None, "qfrc_bias": bias,
                "tcp_position": tuple(float(x * 1000) for x in self._tcp_pos()),
                "body_mass": tuple(float(x) for x in self.model.body_mass),
                "body_inertia": tuple(tuple(float(v) for v in row) for row in self.model.body_inertia),
                "dof_frictionloss": tuple(float(self.model.dof_frictionloss[i]) for i in self.dof_addr),
                "dof_damping": tuple(float(self.model.dof_damping[i]) for i in self.dof_addr)}


def _markers(node, plan, planned_xyz_mm=None):
    from geometry_msgs.msg import Point
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from visualization_msgs.msg import Marker, MarkerArray
    from .urdf_kinematics import A0509Urdf, find_a0509_urdf

    publisher = node.create_publisher(MarkerArray, "/a0509_dynamics_experiment/trajectory_markers",
                                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    markers = MarkerArray()
    line = Marker()
    line.header.frame_id = "base_link"
    line.header.stamp = node.get_clock().now().to_msg()
    line.ns = "planned"
    line.id = 0
    line.type = Marker.LINE_STRIP
    line.action = Marker.ADD
    line.scale.x = 0.005
    line.color.r, line.color.g, line.color.b, line.color.a = (0.1, 0.3, 1.0, 1.0)
    urdf_path = find_a0509_urdf() if plan.config.trajectory_id == "T4" else None
    urdf = A0509Urdf(urdf_path) if urdf_path else None
    for index, item in enumerate(plan.points):
        xyz = (item.xyz_mm if item.xyz_mm else
               planned_xyz_mm[index] if planned_xyz_mm else
               urdf.links_mm(item.q_rad)[-1] if urdf else None)
        if xyz:
            p = Point()
            p.x, p.y, p.z = (v / 1000.0 for v in xyz)
            line.points.append(p)
    markers.markers.append(line)
    publisher.publish(markers)
    return publisher


def _prepare_cartesian_home(node, publisher, latest, launch, sample_hz: int, observer: MujocoObserver):
    """Move the simulator only to a reachable reference pose before data capture."""
    import rclpy
    from std_msgs.msg import Float64MultiArray

    home = (-0.5, -0.5, 1.6, 0.0, -0.6, 0.0)
    deadline = time.monotonic() + 15.0
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if launch is not None and launch.poll() is not None:
            raise RuntimeError("MuJoCo launch exited before controller became ready")
    if publisher.get_subscription_count() == 0:
        raise RuntimeError("MuJoCo position controller is not subscribed; no commands sent")
    origin = latest["q"]
    if max(abs(home[i] - origin[i]) for i in range(6)) < 0.03:
        return home
    start_ros = node.get_clock().now().nanoseconds / 1e9
    start_wall = time.monotonic()
    duration = 6.0
    for k in range(1, int(duration * sample_hz) + 1):
        target_time = start_ros + k / sample_hz
        while node.get_clock().now().nanoseconds / 1e9 < target_time:
            if time.monotonic() - start_wall > duration + 15:
                raise RuntimeError("MuJoCo clock stopped during reference positioning")
            rclpy.spin_once(node, timeout_sec=0.005)
        u = k / (duration * sample_hz)
        scale = 10 * u**3 - 15 * u**4 + 6 * u**5
        target = [origin[i] + (home[i] - origin[i]) * scale for i in range(6)]
        observer.require_contact_free(target)
        publisher.publish(Float64MultiArray(data=target))
    settle_until = node.get_clock().now().nanoseconds / 1e9 + 1.0
    settle_wall_deadline = time.monotonic() + 10.0
    while node.get_clock().now().nanoseconds / 1e9 < settle_until:
        if time.monotonic() > settle_wall_deadline:
            raise RuntimeError("MuJoCo clock stopped while settling at the reference pose")
        rclpy.spin_once(node, timeout_sec=0.01)
    if latest["q"] is None or max(abs(home[i] - latest["q"][i]) for i in range(6)) > 0.05:
        raise RuntimeError("MuJoCo did not reach the Cartesian experiment reference pose")
    return tuple(latest["q"])


def execute(config: TrajectoryConfig, root: str, gripper_type: str, payload_mass: float,
            sim_domain_id: int, connect_existing: bool, launch_timeout_s: float = 60.0,
            reference_explicit: bool = False, record_video: bool = False) -> Path:
    if not 0 <= sim_domain_id <= 232:
        raise ValueError("ROS_DOMAIN_ID must be 0..232")
    if gripper_type not in ("none", "ag145"):
        raise ValueError("simulation gripper type must be none or ag145")
    if gripper_type == "ag145" and connect_existing:
        raise ValueError("AG145 simulation requires a newly launched model")
    if "rclpy" in __import__("sys").modules and os.environ.get("ROS_DOMAIN_ID") != str(sim_domain_id):
        raise RuntimeError("simulation domain cannot change after rclpy has been imported")
    os.environ["ROS_DOMAIN_ID"] = str(sim_domain_id)
    precreated_trial = None
    scene_path = None
    gripper_model_info = None
    if gripper_type == "ag145":
        from .ag145_model import AG145_MASS_KG, AG145_TCP_OFFSET_MM, build_ag145_scene
        precreated_trial = create_trial(root, "simulation", config.trajectory_id)
        scene_path = build_ag145_scene(precreated_trial[0] / "ag145_scene.xml")
        gripper_model_info = {"configuration": "fixed open pose", "mass_kg": AG145_MASS_KG,
                              "inertia": "uncalibrated box approximation",
                              "collision": "approximate bounding boxes",
                              "visual_orientation": "URDF fixed-axis RPY converted to MJCF quaternion",
                              "tcp_site": "ag145_tcp", "tcp_offset_mm": list(AG145_TCP_OFFSET_MM),
                              "tcp_calibrated": False}
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    launch = None
    bag = None
    video = None
    node = None
    rclpy.init()
    try:
        if not connect_existing:
            launch_args = ["ros2", "launch", "a0509_mujoco_sim", "mujoco_sim.launch.py",
                           "use_sim_time:=true", f"gripper_type:={gripper_type}"]
            if scene_path is not None:
                launch_args.append(f"mujoco_model_path:={scene_path}")
            launch = subprocess.Popen(launch_args, env=os.environ.copy())
        node = Node("a0509_dynamics_simulation", parameter_overrides=[Parameter("use_sim_time", value=True)],
                    automatically_declare_parameters_from_overrides=True)
        latest = {"q": None, "dq": None, "effort": None, "arrival": None}

        def on_joint_state(message: JointState):
            try:
                from .dynamics_logger import ordered
                latest["q"] = ordered(message.position, message.name)
                latest["dq"] = ordered(message.velocity, message.name) if len(message.velocity) == len(message.name) else None
                latest["effort"] = ordered(message.effort, message.name) if len(message.effort) == len(message.name) else None
                latest["arrival"] = time.monotonic()
                recorder.ingest_joint_state(message.name, message.position, message.velocity, message.effort,
                                            arrival_s=latest["arrival"],
                                            source_time_s=message.header.stamp.sec + message.header.stamp.nanosec / 1e9)
            except ValueError:
                pass

        recorder = DynamicsRecorder("simulation", config.trajectory_id, "pending", gripper_type, payload_mass,
                                    config.sample_hz, max_age_s=0.1)
        node.create_subscription(JointState, "/joint_states", on_joint_state, 50)
        publisher = node.create_publisher(Float64MultiArray, "/dsr_position_controller/commands", 10)
        deadline = time.monotonic() + launch_timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if launch is not None and launch.poll() is not None:
                raise RuntimeError(f"a0509_mujoco_sim launch exited with code {launch.returncode}")
            if latest["q"] is not None and node.get_clock().now().nanoseconds > 0:
                break
        else:
            raise RuntimeError("MuJoCo /joint_states and /clock did not become available")
        names = {name for name, _namespace in node.get_node_names_and_namespaces()}
        if "dsr_controller2" in names or "run_emulator" in names:
            raise RuntimeError("real/virtual Doosan controller detected in the simulation ROS domain; commands blocked")
        if "controller_manager" not in names:
            raise RuntimeError("expected MuJoCo controller manager missing; commands blocked")
        controller_deadline = time.monotonic() + 15.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < controller_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError("MuJoCo position controller is not subscribed; commands blocked")
        observer = MujocoObserver(scene_path)
        start_q = latest["q"]
        if config.trajectory_id == "T4":
            config = replace(config, joint_start_rad=start_q)
        else:
            start_q = _prepare_cartesian_home(node, publisher, latest, launch, config.sample_hz, observer)
            observer.set_ik_reference(start_q)
            measured_reference = observer.tcp_mm(start_q)
            if reference_explicit:
                gap = math.dist(config.reference_tcp_mm, measured_reference)
                if gap > 5.0:
                    raise RuntimeError(f"reference TCP differs from current simulated TCP by {gap:.1f} mm")
            else:
                config = replace(config, reference_tcp_mm=measured_reference)
        plan = generate(config)
        targets = []
        planned_tcp = []
        previous = start_q
        for point in plan.points:
            if point.q_rad is not None:
                target = point.q_rad
            else:
                target = observer.solve_ik(point.xyz_mm, previous)
            planned_tcp.append(observer.require_contact_free(target))
            targets.append(target)
            previous = target
        folder, trial_id = precreated_trial or create_trial(root, "simulation", config.trajectory_id)
        recorder.trial_id = trial_id
        save_trajectory(plan, folder)
        snapshots = [observer.links_mm(targets[i]) for i in
                     sorted({0, len(targets) // 4, len(targets) // 2, 3 * len(targets) // 4, len(targets) - 1})]
        save_trajectory_plot(plan, folder, robot_links_mm=observer.links_mm(start_q),
                             robot_snapshots_mm=snapshots,
                             planned_xyz_mm=planned_tcp if config.trajectory_id == "T4" else None)
        if record_video:
            from .video_recorder import SimulationVideoRecorder
            video = SimulationVideoRecorder(folder / "simulation.mp4")
            video.start()
        bag_path = folder / "rosbag"
        bag = subprocess.Popen(["ros2", "bag", "record", "-o", str(bag_path), "/clock", "/joint_states",
                                "/dsr_position_controller/commands", "/a0509_dynamics_experiment/trajectory_markers"],
                               env=os.environ.copy())
        ready_deadline = time.monotonic() + 10.0
        while time.monotonic() < ready_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if bag.poll() is not None:
                raise RuntimeError(f"ros2 bag record exited early with code {bag.returncode}")
            if bag_path.is_dir() and latest["arrival"] is not None and time.monotonic() - latest["arrival"] < 0.1:
                break
        else:
            raise RuntimeError("ROS 2 Bag or fresh joint feedback did not become ready")
        marker_publisher = _markers(node, plan, planned_tcp)
        measured = []
        previous_dq = None
        previous_at = None

        def wait_for_fresh_feedback() -> None:
            fresh_deadline = time.monotonic() + 2.0
            while latest["arrival"] is None or time.monotonic() - latest["arrival"] > 0.2:
                if time.monotonic() > fresh_deadline:
                    raise RuntimeError("Simulation joint feedback stale; commands stopped")
                if launch is not None and launch.poll() is not None:
                    raise RuntimeError("MuJoCo process exited while waiting for joint feedback")
                rclpy.spin_once(node, timeout_sec=0.01)

        wait_for_fresh_feedback()
        start_ros = node.get_clock().now().nanoseconds / 1e9
        for point, target in zip(plan.points, targets):
            wait_deadline = time.monotonic() + 30.0
            while node.get_clock().now().nanoseconds / 1e9 - start_ros < point.time_s:
                if time.monotonic() > wait_deadline:
                    raise RuntimeError("MuJoCo clock stalled during trajectory execution")
                if launch is not None and launch.poll() is not None:
                    raise RuntimeError("MuJoCo process exited during execution")
                rclpy.spin_once(node, timeout_sec=0.005)
            wait_for_fresh_feedback()
            message = Float64MultiArray()
            message.data = list(target)
            publisher.publish(message)
            q = latest["q"]
            dq = latest["dq"]
            now = time.monotonic()
            ddq = None
            if dq is not None and previous_dq is not None and previous_at is not None and now > previous_at:
                ddq = tuple((dq[i] - previous_dq[i]) / (now - previous_at) for i in range(6))
            previous_dq, previous_at = dq, now
            physics = observer.observe(q, dq, ddq)
            physics["qfrc_applied"] = latest["effort"]
            measured.append(physics["tcp_position"])
            recorder.snapshot(time.time(), node.get_clock().now().nanoseconds / 1e9, target, physics, now)
        # Hold at the final target after recording ends.
        publisher.publish(Float64MultiArray(data=list(targets[-1])))
        if bag.poll() is not None:
            raise RuntimeError(f"ros2 bag record exited during trajectory with code {bag.returncode}")
        bag.send_signal(signal.SIGINT)
        bag.wait(timeout=20)
        bag = None
        if not (bag_path / "metadata.yaml").is_file():
            raise RuntimeError("ROS 2 Bag metadata missing after recorder stopped")
        save_dynamics(recorder.rows, folder)
        measured_joint = [tuple(row[f"q{i}"] for i in range(1, 7)) for row in recorder.rows
                          if all(row.get(f"q{i}") is not None for i in range(1, 7))]
        measured_times = [row["ros_time_s"] - recorder.rows[0]["ros_time_s"] for row in recorder.rows
                          if all(row.get(f"q{i}") is not None for i in range(1, 7))]
        save_trajectory_plot(plan, folder, measured_xyz_mm=measured,
                             measured_joint_rad=measured_joint, measured_time_s=measured_times,
                             robot_links_mm=observer.links_mm(start_q), robot_snapshots_mm=snapshots,
                             planned_xyz_mm=planned_tcp if config.trajectory_id == "T4" else None)
        save_dynamics_plot(recorder.rows, folder)
        video_info = video.stop() if video is not None else None
        video = None
        save_metadata(folder, plan, "simulation", trial_id, {
            "quality": recorder.quality(), "ros_domain_id": sim_domain_id,
            "time_sync": "JointState header and ros_time_s use MuJoCo simulation time; callback arrival time is used for freshness. Bag storage timestamps use wall time.",
            "gripper_type": gripper_type, "payload_mass_kg": payload_mass,
            "rosbag": str(bag_path), "rosbag_relative_path": "rosbag",
            "sim_model": ("ag145_scene.xml (generated from dsr_description2 A0509 and dh_ag145_description)"
                          if gripper_type == "ag145" else "dsr_description2/mujoco_models/a0509/scene.xml"),
            "gripper_model": gripper_model_info,
            "physics_source": "Separate MuJoCo model reconstructed from measured q/dq/ddq; qfrc_actuator unavailable from upstream bridge",
            "acceleration_method": "finite difference of observed JointState velocity; blank if velocity unavailable",
            "sim_initialization": "Cartesian trials first move the simulator to (-0.5, -0.5, 1.6, 0, -0.6, 0) rad; this setup move is outside the recorded trajectory",
            "torque_definitions": TORQUE_DEFINITIONS,
            "validation": "IK position/orientation convergence and conservative joint limits; no physical robot involved",
            "video": video_info,
        })
        return folder
    finally:
        if video is not None:
            try: video.stop()
            except RuntimeError: pass
        if bag is not None and bag.poll() is None:
            bag.send_signal(signal.SIGINT)
            try: bag.wait(timeout=10)
            except subprocess.TimeoutExpired: bag.terminate()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        if launch is not None and launch.poll() is None:
            launch.send_signal(signal.SIGINT)
            try: launch.wait(timeout=10)
            except subprocess.TimeoutExpired: launch.terminate()
