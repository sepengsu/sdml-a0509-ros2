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
from .trajectory_generator import (DYNAMICS10_IDS, JOINT_NAMES, TrajectoryConfig,
                                   excitation_metadata, generate, is_joint_trajectory)
from .dynamics10_safety import (ACCELERATION_LIMITS, POSITION_LIMITS,
                                VELOCITY_LIMITS, validate_joint_plan)
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
        full_mass = self.np.empty((self.model.nv, self.model.nv), dtype=float)
        self.mj.mj_fullM(self.model, full_mass, self.data.qM)
        mass_matrix = tuple(tuple(float(full_mass[i, j]) for j in self.dof_addr)
                            for i in self.dof_addr)
        inverse = None
        if dq is not None and ddq is not None:
            self.mj.mj_inverse(self.model, self.data)
            inverse = tuple(float(self.data.qfrc_inverse[i]) for i in self.dof_addr)
        return {"qpos": positions, "qvel": velocities, "qacc": acceleration,
                "qfrc_inverse": inverse, "qfrc_actuator": None, "qfrc_bias": bias,
                "mass_matrix": mass_matrix,
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


def _prepare_joint_home(node, publisher, latest, launch, sample_hz: int,
                        observer: MujocoObserver, home):
    """Validate and move the simulator to a reachable pose before capture."""
    import rclpy
    from std_msgs.msg import Float64MultiArray

    deadline = time.monotonic() + 15.0
    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if launch is not None and launch.poll() is not None:
            raise RuntimeError("MuJoCo launch exited before controller became ready")
    if publisher.get_subscription_count() == 0:
        raise RuntimeError("MuJoCo position controller is not subscribed; no commands sent")
    origin = latest["q"]
    if max(abs(home[i] - origin[i]) for i in range(6)) < 0.03:
        return tuple(origin)
    # Check the entire setup path before publishing its first point.
    for k in range(1, int(6.0 * sample_hz) + 1):
        u = k / (6.0 * sample_hz)
        scale = 10 * u**3 - 15 * u**4 + 6 * u**5
        ds = 30*u**2 - 60*u**3 + 30*u**4
        dds = 60*u - 180*u**2 + 120*u**3
        setup_q = [origin[i] + (home[i] - origin[i]) * scale for i in range(6)]
        for i in range(6):
            if not POSITION_LIMITS[i][0] <= setup_q[i] <= POSITION_LIMITS[i][1]:
                raise RuntimeError(f"J{i+1} setup position exceeds conservative limit")
            if (abs((home[i]-origin[i])*ds/6.0) > VELOCITY_LIMITS[i] or
                    abs((home[i]-origin[i])*dds/36.0) > ACCELERATION_LIMITS[i]):
                raise RuntimeError(f"J{i+1} setup velocity/acceleration exceeds conservative limit")
        observer.require_contact_free(setup_q)
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
    if latest["q"] is None or max(abs(home[i] - latest["q"][i]) for i in range(6)) > 0.03:
        raise RuntimeError("MuJoCo did not reach the experiment start pose")
    return tuple(latest["q"])


def _prepare_cartesian_home(node, publisher, latest, launch, sample_hz: int, observer):
    return _prepare_joint_home(node, publisher, latest, launch, sample_hz, observer,
                               (-0.5, -0.5, 1.6, 0.0, -0.6, 0.0))


class SimTimeVelocityFilter:
    """Least-squares acceleration from five latest unique ROS simulation stamps."""

    def __init__(self, width: int = 5):
        from collections import deque
        self.samples = deque(maxlen=width)

    def add(self, stamp_s: float, velocity) -> None:
        if velocity is None or not math.isfinite(stamp_s):
            return
        if self.samples and stamp_s <= self.samples[-1][0]:
            return
        self.samples.append((stamp_s, tuple(velocity)))

    def acceleration(self):
        if len(self.samples) < 3:
            return None
        times = [item[0] for item in self.samples]
        mean_t = sum(times) / len(times)
        delta = [t - mean_t for t in times]
        denom = sum(x*x for x in delta)
        if denom <= 1e-12:
            return None
        return tuple(sum(delta[k] * self.samples[k][1][joint]
                         for k in range(len(times))) / denom for joint in range(6))


def execute(config: TrajectoryConfig, root: str, gripper_type: str, payload_mass: float,
            sim_domain_id: int, connect_existing: bool, launch_timeout_s: float = 60.0,
            reference_explicit: bool = False, record_video: bool = False,
            mdpg_mount_xyz=None, mdpg_mount_rpy=None,
            mdpg_dynamics_model: str = "visual",
            payload_spec_file: str | None = None) -> Path:
    if not 0 <= sim_domain_id <= 232:
        raise ValueError("ROS_DOMAIN_ID must be 0..232")
    if gripper_type not in ("none", "ag145", "mdpg_1s1"):
        raise ValueError("unsupported simulation gripper type")
    if gripper_type == "mdpg_1s1" and (mdpg_dynamics_model != "box_approx"
                                       or mdpg_mount_xyz is None or mdpg_mount_rpy is None):
        raise ValueError("mDPG dynamics requires explicit box_approx and mount pose")
    if gripper_type in ("ag145", "mdpg_1s1") and connect_existing:
        raise ValueError("tool simulation requires a newly launched matching model")
    payload_spec = None
    if payload_mass > 0:
        if not payload_spec_file:
            raise ValueError("positive payload mass requires --payload-spec-file")
        from .payload_model import load_payload_spec
        payload_spec = load_payload_spec(payload_spec_file, gripper_type, payload_mass)
    if "rclpy" in __import__("sys").modules and os.environ.get("ROS_DOMAIN_ID") != str(sim_domain_id):
        raise RuntimeError("simulation domain cannot change after rclpy has been imported")
    os.environ["ROS_DOMAIN_ID"] = str(sim_domain_id)
    precreated_trial = None
    scene_path = None
    gripper_model_info = None
    if gripper_type == "ag145":
        from .ag145_model import (AG145_BOX_GRASP_ANGLE_RAD, AG145_MASS_KG,
                                 AG145_TCP_OFFSET_MM, build_ag145_scene)
        precreated_trial = create_trial(root, "simulation", config.trajectory_id)
        grasp_angle = AG145_BOX_GRASP_ANGLE_RAD if payload_spec is not None else None
        scene_path = build_ag145_scene(precreated_trial[0] / "ag145_scene.xml",
                                       grasp_angle_rad=grasp_angle)
        gripper_model_info = {"configuration": "fixed grasp pose" if grasp_angle is not None else "fixed open pose",
                              "grasp_angle_rad": grasp_angle,
                              "grasp_model": "ideal fixed attachment; no finger actuation/contact force" if grasp_angle is not None else None,
                              "mass_kg": AG145_MASS_KG,
                              "inertia": "uncalibrated box approximation",
                              "collision": "approximate bounding boxes",
                              "visual_orientation": "URDF fixed-axis RPY converted to MJCF quaternion",
                              "tcp_site": "ag145_tcp", "tcp_offset_mm": list(AG145_TCP_OFFSET_MM),
                              "tcp_calibrated": False}
    elif gripper_type == "mdpg_1s1":
        from mdpg_1s1_description.model import (
            BOX_COM_M, BOX_INERTIA_KG_M2, SPEC_MASS_KG,
            SPEC_MAX_PAYLOAD_KG, build_mdpg_scene)

        precreated_trial = create_trial(root, "simulation", config.trajectory_id)
        scene_path = build_mdpg_scene(precreated_trial[0] / "mdpg_scene.xml",
                                      mdpg_mount_xyz, mdpg_mount_rpy,
                                      dynamics_model=mdpg_dynamics_model)
        gripper_model_info = {
            "dynamics_model": "box_approx", "mass_kg": SPEC_MASS_KG,
            "mass_source": "user-supplied product specification image",
            "center_of_mass_m": list(BOX_COM_M),
            "inertia_diagonal_kg_m2": list(BOX_INERTIA_KG_M2),
            "inertia_source": "uniform-density 65x70x81 mm box estimate, not measured",
            "max_payload_rating_kg": SPEC_MAX_PAYLOAD_KG,
            "mount_xyz_m": list(mdpg_mount_xyz), "mount_rpy_rad": list(mdpg_mount_rpy),
            "mount_verified": False, "tcp_calibrated": False,
            "pneumatic_io": None, "suction_contact": None,
            "label_use": "synthetic approximation only; not real torque ground truth",
        }
    if payload_spec is not None:
        from .payload_model import attach_payload
        attach_payload(scene_path, payload_spec)
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
            if gripper_type == "mdpg_1s1":
                launch_args.extend((
                    f"mdpg_mount_xyz:={' '.join(str(value) for value in mdpg_mount_xyz)}",
                    f"mdpg_mount_rpy:={' '.join(str(value) for value in mdpg_mount_rpy)}",
                    "mdpg_dynamics_model:=box_approx"))
            launch = subprocess.Popen(launch_args, env=os.environ.copy())
        node = Node("a0509_dynamics_simulation", parameter_overrides=[Parameter("use_sim_time", value=True)],
                    automatically_declare_parameters_from_overrides=True)
        latest = {"q": None, "dq": None, "effort": None, "arrival": None}
        velocity_filter = SimTimeVelocityFilter()

        def on_joint_state(message: JointState):
            try:
                from .dynamics_logger import ordered
                latest["q"] = ordered(message.position, message.name)
                latest["dq"] = ordered(message.velocity, message.name) if len(message.velocity) == len(message.name) else None
                latest["effort"] = ordered(message.effort, message.name) if len(message.effort) == len(message.name) else None
                latest["arrival"] = time.monotonic()
                velocity_filter.add(message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
                                    latest["dq"])
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
        observer = MujocoObserver(scene_path, tcp_site=("mdpg_tcp" if gripper_type == "mdpg_1s1"
                                                        else "ag145_tcp"))
        start_q = latest["q"]
        preflight = None
        if config.trajectory_id in DYNAMICS10_IDS:
            plan = generate(config)
            preflight = validate_joint_plan(plan, observer)
            start_q = _prepare_joint_home(node, publisher, latest, launch, config.sample_hz,
                                          observer, config.joint_start_rad)
        elif config.trajectory_id == "T4":
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
        if config.trajectory_id in DYNAMICS10_IDS:
            preflight = validate_joint_plan(plan, observer)
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
        previous_recorded_ros = -math.inf
        for point, target in zip(plan.points, targets):
            wait_deadline = time.monotonic() + 30.0
            while (node.get_clock().now().nanoseconds / 1e9 - start_ros < point.time_s or
                   node.get_clock().now().nanoseconds / 1e9 <= previous_recorded_ros):
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
            ddq = velocity_filter.acceleration() if dq is not None else None
            physics = observer.observe(q, dq, ddq)
            physics["qfrc_applied"] = latest["effort"]
            measured.append(physics["tcp_position"])
            recorded_ros = node.get_clock().now().nanoseconds / 1e9
            if recorded_ros <= previous_recorded_ros:
                raise RuntimeError("MuJoCo simulation time did not advance before sample")
            recorder.snapshot(time.time(), recorded_ros, target, physics, now)
            previous_recorded_ros = recorded_ros
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
                             planned_xyz_mm=planned_tcp if is_joint_trajectory(config.trajectory_id) else None)
        save_dynamics_plot(recorder.rows, folder)
        video_info = video.stop() if video is not None else None
        video = None
        save_metadata(folder, plan, "simulation", trial_id, {
            "quality": recorder.quality(), "ros_domain_id": sim_domain_id,
            "time_sync": "JointState header and ros_time_s use MuJoCo simulation time; callback arrival time is used for freshness. Bag storage timestamps use wall time.",
            "gripper_type": gripper_type, "payload_mass_kg": payload_mass,
            "payload_model": ({**payload_spec, "contact_geometry": None,
                               "attachment_model": "ideal fixed body; no grasp or suction failure model"}
                              if payload_spec else None),
            "rosbag": str(bag_path), "rosbag_relative_path": "rosbag",
            "sim_model": ("ag145_scene.xml (generated from dsr_description2 A0509 and dh_ag145_description)"
                          if gripper_type == "ag145" else
                          "mdpg_scene.xml (0.62 kg uniform-box approximation)"
                          if gripper_type == "mdpg_1s1" else
                          "dsr_description2/mujoco_models/a0509/scene.xml"),
            "gripper_model": gripper_model_info,
            "physics_source": "Separate MuJoCo model reconstructed from measured q/dq/ddq; qfrc_actuator unavailable from upstream bridge",
            "acceleration_method": "least-squares slope of up to five unique JointState velocities in ROS simulation time; blank until three samples or when velocity unavailable",
            "acceleration_filter_samples": 5,
            "trajectory_kind": "joint" if is_joint_trajectory(config.trajectory_id) else "cartesian",
            "dynamics10": excitation_metadata(config.trajectory_id) if config.trajectory_id in DYNAMICS10_IDS else None,
            "preflight": preflight,
            "execution_success": True,
            "sample_count": len(recorder.rows),
            "sim_initialization": ("Joint experiment start posture reached by a 6 s quintic setup move, checked for contact; setup is outside recorded trajectory"
                                   if config.trajectory_id in DYNAMICS10_IDS else
                                   "Cartesian trials first move the simulator to (-0.5, -0.5, 1.6, 0, -0.6, 0) rad; setup is outside recorded trajectory"),
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
