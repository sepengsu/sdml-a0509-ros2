"""Headless, simulation-only multi-A0509 MuJoCo collection.

Each lane has independent joints, actuators, tool and payload in one MjModel.
No ROS graph, robot IP or hardware control service is used.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

from .ag145_model import build_ag145_scene
from .dataset_exporter import save_dynamics, save_metadata, save_trajectory
from .dynamics10_safety import (ACCELERATION_LIMITS, POSITION_LIMITS,
                                VELOCITY_LIMITS, validate_joint_plan)
from .dynamics_logger import DynamicsRecorder, TORQUE_DEFINITIONS
from .payload_model import attach_payload, load_payload_spec
from .simulation_executor import MujocoObserver
from .trajectory_generator import (BASE_HOME, EXCITATIONS, JOINT_NAMES, TrajectoryConfig,
                                   generate, is_joint_trajectory)
from .trajectory_visualizer import save_dynamics_plot, save_trajectory_plot

CONDITIONS = (
    ("bare_p0", "none", 0.0),
    ("ag145_p0", "ag145", 0.0),
    ("ag145_p05", "ag145", 0.5),
    ("ag145_p10", "ag145", 1.0),
    ("mdpg_p0", "mdpg_1s1", 0.0),
    ("mdpg_p05", "mdpg_1s1", 0.5),
    ("mdpg_p10", "mdpg_1s1", 1.0),
)
TORQUE_LIMITS_NM = (194.0, 194.0, 163.0, 50.0, 50.0, 50.0)
# Quarter-step factors retain exactly 100 Hz after time stretching. They are
# conservative simulation-only responses to the original MJCF's high damping.
GRID_TIME_SCALE = {"T1": 1.0, "T2": 3.0, "T3": 1.5, "T4": 1.25,
                   "T5": 1.25, "T6": 2.25, "T7": 2.75, "T8": 2.5,
                   "T9": 2.0, "T10": 2.0, "T11": 1.75, "T12": 2.25,
                   "T13": 2.0, "T14": 2.5, "T15": 1.0, "T16": 1.0}


def _arm_scene() -> Path:
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("dsr_description2")) / "mujoco_models/a0509/scene.xml"


def _bare_scene(output: Path) -> Path:
    source = _arm_scene()
    scene = ET.parse(source).getroot()
    include = scene.find("include")
    arm = ET.parse(source.parent / include.get("file")).getroot()
    scene.remove(include)
    scene.find("compiler").attrib.pop("meshdir", None)
    for mesh in arm.find("asset").findall("mesh"):
        mesh.set("file", (source.parent.parent / mesh.get("file")).resolve().as_posix())
    scene.find("asset").extend(list(arm.find("asset")))
    scene.find("worldbody").extend(list(arm.find("worldbody")))
    scene.append(arm.find("actuator"))
    ET.ElementTree(scene).write(output, encoding="utf-8", xml_declaration=True)
    return output


def build_condition_scenes(folder: Path, mount_xyz, mount_rpy, spec_path: Path) -> dict[str, Path]:
    from mdpg_1s1_description.model import build_mdpg_scene
    folder.mkdir(parents=True, exist_ok=True)
    scenes = {}
    for condition, gripper, payload in CONDITIONS:
        path = folder / f"{condition}.xml"
        if gripper == "none":
            _bare_scene(path)
        elif gripper == "ag145":
            build_ag145_scene(path, grasp_angle_rad=0.60 if payload else None)
        else:
            build_mdpg_scene(path, mount_xyz, mount_rpy, dynamics_model="box_approx")
        if payload:
            attach_payload(path, load_payload_spec(spec_path, gripper, payload))
        scenes[condition] = path
    return scenes


def _prefix_references(element: ET.Element, prefix: str, named: set[str]) -> None:
    for part in element.iter():
        if part.get("name") in named:
            part.set("name", prefix + part.get("name"))
        for key in ("mesh", "material", "texture", "joint", "body", "site", "geom", "target"):
            if part.get(key) in named:
                part.set(key, prefix + part.get(key))


def build_grid_scene(scenes: dict[str, Path], lanes: list[dict], output: Path,
                     columns: int, spacing_m: float = 2.5) -> Path:
    """Clone all lane assets and kinematic trees under separated fixed bases."""
    if columns < 1 or not 2.0 <= spacing_m <= 10.0:
        raise ValueError("grid spacing must be a declared 2..10 m simulation layout")
    root = ET.parse(next(iter(scenes.values()))).getroot()
    root.set("model", "a0509 grid simulation")
    assets = root.find("asset")
    world = root.find("worldbody")
    for item in list(assets):
        if item.tag not in ("texture", "material") or item.get("name") != "groundplane":
            assets.remove(item)
    for item in list(world):
        if item.tag != "light" and item.get("name") != "floor":
            world.remove(item)
    old_act = root.find("actuator")
    if old_act is not None:
        root.remove(old_act)
    actuators = ET.SubElement(root, "actuator")
    for index, lane in enumerate(lanes):
        source = ET.parse(scenes[lane["condition"]]).getroot()
        prefix = f"r{index:02d}_"
        named = {part.get("name") for part in source.iter() if part.get("name")}
        # Prefix names and references within each copied subtree.  The common
        # floor texture/material stays global and is never copied per robot.
        for element in source.find("asset"):
            if element.get("name") in (None, "groundplane"):
                continue
            clone = copy.deepcopy(element)
            _prefix_references(clone, prefix, named)
            assets.append(clone)
        wrapper = ET.SubElement(world, "body", name=prefix + "base",
                                pos=f"{index % columns * spacing_m:.12g} {index // columns * spacing_m:.12g} 0")
        for element in source.find("worldbody"):
            if element.tag in ("light",) or element.get("name") == "floor":
                continue
            clone = copy.deepcopy(element)
            _prefix_references(clone, prefix, named)
            wrapper.append(clone)
        for element in source.find("actuator"):
            clone = copy.deepcopy(element)
            _prefix_references(clone, prefix, named)
            actuators.append(clone)
        lane["prefix"] = prefix
        lane["base_xy_m"] = [index % columns * spacing_m, index // columns * spacing_m]
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    return output


def _lane_ids(mj, model, lane):
    prefix = lane["prefix"]
    joints = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, prefix + name) for name in JOINT_NAMES]
    if any(i < 0 for i in joints):
        raise RuntimeError(f"missing grid joint for {prefix}")
    lane["qpos"] = [int(model.jnt_qposadr[i]) for i in joints]
    lane["dof"] = [int(model.jnt_dofadr[i]) for i in joints]
    lane["act"] = [next(k for k in range(model.nu) if model.actuator_trnid[k, 0] == i)
                   for i in joints]
    tool = "mdpg_tcp" if lane["gripper"] == "mdpg_1s1" else "ag145_tcp"
    lane["site"] = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, prefix + tool)
    lane["flange"] = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, prefix + "link_6")
    lane["body_ids"] = [i for i in range(1, model.nbody)
                        if (mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, i) or "").startswith(prefix)]


def _plan(observer, trajectory_id):
    import numpy as np
    if trajectory_id in ("T1", "T2", "T3"):
        observer.set_ik_reference(BASE_HOME)
        # The model's high passive damping makes the default 4 s Cartesian
        # segments incompatible with the stated wrist torque limits.
        config = TrajectoryConfig(trajectory_id, reference_tcp_mm=observer.tcp_mm(BASE_HOME),
                                  segment_seconds=10.0, time_scale=GRID_TIME_SCALE[trajectory_id])
    else:
        config = TrajectoryConfig(trajectory_id, joint_start_rad=BASE_HOME if trajectory_id == "T4" else (0.0,) * 6,
                                  time_scale=GRID_TIME_SCALE[trajectory_id])
    plan = generate(config)
    if is_joint_trajectory(trajectory_id):
        validate_joint_plan(plan, observer)
    targets, tcp = [], []
    previous = plan.points[0].q_rad or BASE_HOME
    for point in plan.points:
        q = point.q_rad if point.q_rad is not None else _precise_ik(observer, point.xyz_mm, previous)
        tcp.append(observer.require_contact_free(q))
        targets.append(q)
        previous = q
    if plan.points[0].dq_rad_s is None:
        from scipy.signal import savgol_filter
        positions = np.asarray(targets)
        # IK's finite positional tolerance makes a 10 ms finite difference
        # artificially spike acceleration. Differentiate a local quintic fit.
        velocities = savgol_filter(positions, 51, 5, deriv=1, delta=0.01, axis=0)
        accelerations = savgol_filter(positions, 51, 5, deriv=2, delta=0.01, axis=0)
    else:
        velocities = np.asarray([point.dq_rad_s for point in plan.points])
        accelerations = np.asarray([point.ddq_rad_s2 for point in plan.points])
    for k, q in enumerate(targets):
        for j in range(6):
            if not POSITION_LIMITS[j][0] <= q[j] <= POSITION_LIMITS[j][1]:
                raise RuntimeError(f"{trajectory_id} J{j+1} IK position limit at sample {k}")
            if (abs(velocities[k, j]) > VELOCITY_LIMITS[j] or
                    abs(accelerations[k, j]) > ACCELERATION_LIMITS[j]):
                raise RuntimeError(f"{trajectory_id} J{j+1} IK derivative limit at sample {k}: "
                                   f"dq={velocities[k,j]:.4g}, ddq={accelerations[k,j]:.4g}")
    return plan, targets, tcp, velocities, accelerations


def _precise_ik(observer, xyz_mm, seed):
    """Refine legacy 1 mm IK tolerance for continuous 100 Hz joint targets."""
    np, mj = observer.np, observer.mj
    q = np.asarray(observer.solve_ik(xyz_mm, seed), dtype=float)
    target = np.asarray(xyz_mm) / 1000.0
    desired = observer.reference_rotation
    for _ in range(100):
        observer._set_state(q)
        rotation = observer._tcp_rotation()
        error_pos = target - observer._tcp_pos()
        error_rot = 0.5 * sum((np.cross(rotation[:, i], desired[:, i]) for i in range(3)), np.zeros(3))
        if np.linalg.norm(error_pos) < 0.00005 and np.linalg.norm(error_rot) < 0.005:
            return tuple(float(value) for value in q)
        jac_pos = np.zeros((3, observer.model.nv))
        jac_rot = np.zeros((3, observer.model.nv))
        if observer.tcp_site_id >= 0:
            mj.mj_jacSite(observer.model, observer.data, jac_pos, jac_rot, observer.tcp_site_id)
        else:
            mj.mj_jacBody(observer.model, observer.data, jac_pos, jac_rot, observer.body_id)
        columns = observer.dof_addr
        jacobian = np.vstack((jac_pos[:, columns], 0.3 * jac_rot[:, columns]))
        error = np.concatenate((error_pos, 0.3 * error_rot))
        step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 0.0002 * np.eye(6), error)
        q += np.clip(step, -0.04, 0.04)
        for i, (low, high) in enumerate(observer.limits):
            q[i] = np.clip(q[i], low, high)
    raise RuntimeError(f"precise IK failed at {xyz_mm} mm")


def _bag_writer(path: Path):
    import rosbag2_py
    from rosbag2_py import ConverterOptions, StorageOptions, TopicMetadata
    writer = rosbag2_py.SequentialWriter()
    writer.open(StorageOptions(uri=str(path), storage_id="sqlite3"), ConverterOptions("cdr", "cdr"))
    for topic, kind in (("/clock", "rosgraph_msgs/msg/Clock"),
                        ("/joint_states", "sensor_msgs/msg/JointState"),
                        ("/target_joint_states", "sensor_msgs/msg/JointState")):
        writer.create_topic(TopicMetadata(name=topic, type=kind, serialization_format="cdr"))
    return writer


def _bag_sample(writer, sim_s, lanes, data):
    from builtin_interfaces.msg import Time
    from rclpy.serialization import serialize_message
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    stamp = Time(sec=int(sim_s), nanosec=round((sim_s % 1) * 1e9))
    writer.write("/clock", serialize_message(Clock(clock=stamp)), round(sim_s * 1e9))
    actual, target = JointState(), JointState()
    actual.header.stamp = target.header.stamp = stamp
    for lane in lanes:
        actual.name.extend(lane["prefix"] + name for name in JOINT_NAMES)
        target.name.extend(lane["prefix"] + name for name in JOINT_NAMES)
        actual.position.extend(float(data.qpos[i]) for i in lane["qpos"])
        actual.velocity.extend(float(data.qvel[i]) for i in lane["dof"])
        actual.effort.extend(float(data.qfrc_actuator[i] + data.qfrc_applied[i]) for i in lane["dof"])
        target.position.extend(lane["target"])
    writer.write("/joint_states", serialize_message(actual), round(sim_s * 1e9))
    writer.write("/target_joint_states", serialize_message(target), round(sim_s * 1e9))


def _physics(mj, model, data, inverse_data, lane, full_mass):
    d, q = lane["dof"], lane["qpos"]
    pos = data.site_xpos[lane["site"]] if lane["site"] >= 0 else data.xpos[lane["flange"]]
    return {
        "qpos": tuple(float(data.qpos[i]) for i in q),
        "qvel": tuple(float(data.qvel[i]) for i in d),
        "qacc": tuple(float(data.qacc[i]) for i in d),
        "qfrc_inverse": tuple(float(inverse_data.qfrc_inverse[i]) for i in d),
        "qfrc_actuator": tuple(float(data.qfrc_actuator[i]) for i in d),
        "qfrc_applied": tuple(float(data.qfrc_applied[i]) for i in d),
        "qfrc_bias": tuple(float(data.qfrc_bias[i]) for i in d),
        "mass_matrix": tuple(tuple(float(full_mass[i, j]) for j in d) for i in d),
        "tcp_position": tuple(float((pos[k] - (lane["base_xy_m"][k] if k < 2 else 0)) * 1000) for k in range(3)),
        "body_mass": tuple(float(model.body_mass[i]) for i in lane["body_ids"]),
        "body_inertia": tuple(tuple(float(v) for v in model.body_inertia[i]) for i in lane["body_ids"]),
        "dof_frictionloss": tuple(float(model.dof_frictionloss[i]) for i in d),
        "dof_damping": tuple(float(model.dof_damping[i]) for i in d),
    }


def collect_batch(root: Path, scenes: dict[str, Path], lanes: list[dict], trajectory_id: str | None,
                  batch_index: int, columns: int, pilot: bool = False) -> dict:
    import mujoco as mj
    import numpy as np
    model_name = (f"batch_{batch_index:02d}.xml" if trajectory_id else
                  f"parallel_{lanes[0]['condition']}_trial_{lanes[0]['repeat']:03d}.xml")
    scene = build_grid_scene(scenes, lanes, root / "models" / model_name, columns)
    model = mj.MjModel.from_xml_path(str(scene))
    if abs(model.opt.timestep - 0.002) > 1e-9:
        raise RuntimeError(f"expected 0.002 s MuJoCo timestep; got {model.opt.timestep}")
    data, inverse_data, feedforward_data = mj.MjData(model), mj.MjData(model), mj.MjData(model)
    plan_cache = {}
    for lane in lanes:
        _lane_ids(mj, model, lane)
        lane["trajectory_id"] = lane.get("trajectory", trajectory_id)
        if lane["trajectory_id"] is None:
            raise ValueError("each mixed lane needs a trajectory ID")
        cache_key = (lane["condition"], lane["trajectory_id"])
        if cache_key not in plan_cache:
            tool = "mdpg_tcp" if lane["gripper"] == "mdpg_1s1" else "ag145_tcp"
            observer = MujocoObserver(scenes[lane["condition"]], tcp_site=tool)
            plan_cache[cache_key] = _plan(observer, lane["trajectory_id"])
        (lane["plan"], lane["targets"], lane["planned_tcp"],
         lane["target_dq"], lane["target_ddq"]) = plan_cache[cache_key]
        lane["recorder"] = DynamicsRecorder("simulation", lane["trajectory_id"],
                                            f"trial_{lane['repeat']:03d}", lane["gripper"],
                                            lane["payload_kg"], max_age_s=1.0)
        lane["max_tracking_error_rad"] = 0.0
    count = max(len(lane["plan"].points) for lane in lanes)
    if trajectory_id is not None and any(len(lane["plan"].points) != count for lane in lanes):
        raise RuntimeError("same-trajectory lanes have different durations")
    for lane in lanes:
        for i, q in zip(lane["qpos"], lane["targets"][0]):
            data.qpos[i] = q
        for i, q in zip(lane["act"], lane["targets"][0]):
            data.ctrl[i] = q
    mj.mj_forward(model, data)
    if data.ncon:
        raise RuntimeError(f"grid starts with {data.ncon} contacts")
    bag_group = ((Path(f"batch_{batch_index:02d}") / trajectory_id) if trajectory_id else
                 (Path(lanes[0]["condition"]) / f"trial_{lanes[0]['repeat']:03d}"))
    bag = root / "bags" / bag_group / f"run_{int(time.time())}"
    bag.parent.mkdir(parents=True, exist_ok=True)
    writer = _bag_writer(bag)
    max_error, max_command, cross_contacts, torque_limit_samples = 0.0, 0.0, 0, 0
    max_command_by_joint = [0.0] * 6
    limit_samples_by_joint = [0] * 6
    started = time.monotonic()
    for sample in range(count):
        for lane in lanes:
            lane["sample_index"] = min(sample, len(lane["plan"].points) - 1)
            lane["target"] = lane["targets"][lane["sample_index"]]
        if sample:
            # Compute desired generalized force from the same assembled model.
            feedforward_data.qpos[:] = data.qpos
            feedforward_data.qvel[:] = 0
            feedforward_data.qacc[:] = 0
            for lane in lanes:
                for j, addr in enumerate(lane["qpos"]):
                    feedforward_data.qpos[addr] = lane["target"][j]
                for j, addr in enumerate(lane["dof"]):
                    feedforward_data.qvel[addr] = lane["target_dq"][lane["sample_index"], j]
                    feedforward_data.qacc[addr] = lane["target_ddq"][lane["sample_index"], j]
            mj.mj_forward(model, feedforward_data)
            for lane in lanes:
                for j, addr in enumerate(lane["dof"]):
                    feedforward_data.qacc[addr] = lane["target_ddq"][lane["sample_index"], j]
            mj.mj_inverse(model, feedforward_data)
            for _ in range(5):
                for lane in lanes:
                    for j, act in enumerate(lane["act"]):
                        data.ctrl[act] = lane["target"][j]
                mj.mj_forward(model, data)
                for lane in lanes:
                    for j, (qaddr, daddr, act) in enumerate(zip(lane["qpos"], lane["dof"], lane["act"])):
                        desired = (feedforward_data.qfrc_inverse[daddr]
                                   + 150 * (lane["target"][j] - data.qpos[qaddr])
                                   + 20 * (lane["target_dq"][lane["sample_index"], j] - data.qvel[daddr]))
                        max_command = max(max_command, abs(float(desired)))
                        max_command_by_joint[j] = max(max_command_by_joint[j], abs(float(desired)))
                        if abs(desired) > TORQUE_LIMITS_NM[j]:
                            torque_limit_samples += 1
                            limit_samples_by_joint[j] += 1
                            desired = np.clip(desired, -TORQUE_LIMITS_NM[j], TORQUE_LIMITS_NM[j])
                        data.qfrc_applied[daddr] = desired - data.qfrc_actuator[daddr]
                mj.mj_step(model, data)
                if data.ncon:
                    for k in range(data.ncon):
                        c = data.contact[k]
                        b1 = model.geom_bodyid[c.geom1]
                        b2 = model.geom_bodyid[c.geom2]
                        n1 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, b1) or ""
                        n2 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, b2) or ""
                        if n1[:4] != n2[:4] and n1.startswith("r") and n2.startswith("r"):
                            cross_contacts += 1
                    raise RuntimeError(f"MuJoCo contact during {trajectory_id or 'mixed'} sample {sample}")
        # mj_step integrates qpos/qvel after computing derived arrays. Refresh
        # FK, bias, mass and actuator force for the state saved in this row.
        mj.mj_forward(model, data)
        if data.ncon:
            raise RuntimeError(f"MuJoCo contact at saved {trajectory_id or 'mixed'} sample {sample}")
        inverse_data.qpos[:] = data.qpos
        inverse_data.qvel[:] = data.qvel
        inverse_data.ctrl[:] = data.ctrl
        mj.mj_forward(model, inverse_data)
        inverse_data.qacc[:] = data.qacc
        mj.mj_inverse(model, inverse_data)
        full_mass = np.empty((model.nv, model.nv))
        mj.mj_fullM(model, full_mass, data.qM)
        sim_s = sample / 100.0
        for lane in lanes:
            q = [float(data.qpos[i]) for i in lane["qpos"]]
            dq = [float(data.qvel[i]) for i in lane["dof"]]
            error = max(abs(q[j] - lane["target"][j]) for j in range(6))
            max_error = max(max_error, error)
            lane["max_tracking_error_rad"] = max(lane["max_tracking_error_rad"], error)
            if sample >= len(lane["plan"].points):
                continue  # shorter trajectory holds its final command in the shared scene
            recorder = lane["recorder"]
            recorder.ingest_joint_state(JOINT_NAMES, q, dq, arrival_s=sim_s,
                                        source_time_s=sim_s + 1e-6)
            recorder.snapshot(time.time(), sim_s, lane["target"],
                              _physics(mj, model, data, inverse_data, lane, full_mass), arrival_s=sim_s)
        _bag_sample(writer, sim_s, lanes, data)
    del writer
    if not (bag / "metadata.yaml").is_file():
        raise RuntimeError("shared ROS bag did not finalize")
    report = {"samples": count, "lanes": len(lanes), "trajectory": trajectory_id or "mixed",
              "trajectory_samples": {lane["trajectory_id"]: len(lane["plan"].points) for lane in lanes},
              "trajectory_max_tracking_error_rad": {lane["trajectory_id"]: lane["max_tracking_error_rad"] for lane in lanes},
              "max_tracking_error_rad": max_error, "max_command_nm": max_command,
              "max_command_nm_by_joint": max_command_by_joint,
              "torque_limit_samples": torque_limit_samples,
              "torque_limit_samples_by_joint": limit_samples_by_joint,
              "cross_robot_contacts": cross_contacts,
              "wall_seconds": time.monotonic() - started,
              "sim_seconds": (count - 1) / 100.0}
    if max_error > 0.05 or torque_limit_samples:
        raise RuntimeError(f"grid quality gate failed: {json.dumps(report)}")
    for lane in lanes:
        lane_trajectory = lane["trajectory_id"]
        folder = root / "simulation" / lane_trajectory / lane["condition"] / f"trial_{lane['repeat']:03d}"
        folder.mkdir(parents=True, exist_ok=True)
        save_trajectory(lane["plan"], folder)
        save_dynamics(lane["recorder"].rows, folder)
        measured = [row["tcp_position"] for row in lane["recorder"].rows]
        measured_joint = [tuple(row[f"q{i}"] for i in range(1, 7)) for row in lane["recorder"].rows]
        save_trajectory_plot(lane["plan"], folder, measured_xyz_mm=measured,
                             measured_joint_rad=measured_joint,
                             measured_time_s=[i / 100 for i in range(len(lane["plan"].points))],
                             planned_xyz_mm=lane["planned_tcp"] if is_joint_trajectory(lane_trajectory) else None)
        save_dynamics_plot(lane["recorder"].rows, folder)
        save_metadata(folder, lane["plan"], "simulation", f"trial_{lane['repeat']:03d}", {
            "condition_id": lane["condition"], "gripper_type": lane["gripper"],
            "payload_mass_kg": lane["payload_kg"], "quality": lane["recorder"].quality(),
            "dataset_split": EXCITATIONS[lane_trajectory]["split"] if lane_trajectory in EXCITATIONS else "diagnostic",
            "physics_source": "direct MuJoCo state in one isolated multi-robot MjModel",
            "sim_initialization": "Each lane qpos is set to its own first target before the first sample; no setup movement is recorded.",
            "trajectory_design_origin": ("A0509 project extension motivated by state-space coverage in Zhang et al., 2026; not a trajectory specified by that paper"
                                         if lane_trajectory in ("T15", "T16") else
                                         "existing A0509 project trajectory; not a trajectory specified by Zhang et al., 2026"),
            "study_reference_doi": "10.1016/j.knosys.2026.116861",
            "controller": "inverse-dynamics feedforward plus 150 Nm/rad position and 20 Nm s/rad velocity feedback, clipped to MJCF joint force limits; qfrc_applied stores external controller effort",
            "torque_definitions": TORQUE_DEFINITIONS,
            "time_sync": "sample index / 100 Hz is MuJoCo time; wall timestamp is recorded separately",
            "target_joint_derivative_method": "T1-T3: 51-sample fifth-order Savitzky-Golay derivative of precise IK targets; T4-T16: analytic joint trajectory derivatives",
            "grid_time_scale": GRID_TIME_SCALE[lane_trajectory],
            "rosbag": str(bag), "rosbag_relative_path": bag.relative_to(root).as_posix(),
            "rosbag_topics": ["/clock", "/joint_states", "/target_joint_states"],
            "grid": {"batch": batch_index, "lane": lane["prefix"], "base_xy_m": lane["base_xy_m"],
                     "spacing_m": 2.5, "simulation_layout_only": True,
                     "simultaneous_trajectories": trajectory_id is None,
                     "shared_scene_duration_s": (count - 1) / 100.0},
            "limitations": "Tool attachment and payload are fixed; no grasp, suction or pneumatic IO model. This is a distinct direct-MuJoCo controller/data source from the ROS bridge campaign.",
            "report": report,
        })
    return report


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--mount-xyz", nargs=3, type=float, required=True)
    parser.add_argument("--mount-rpy", nargs=3, type=float, required=True)
    parser.add_argument("--payload-spec", type=Path, required=True)
    parser.add_argument("--pilot", action="store_true")
    parser.add_argument("--play", action="store_true")
    parser.add_argument("--campaign", choices=("legacy", "trajectory16"), default="legacy")
    parser.add_argument("--max-batches", type=int, default=2)
    parser.add_argument("--max-runs", type=int, default=21)
    parser.add_argument("--trajectory", choices=("all", *(f"T{i}" for i in range(1, 17))), default="all")
    args = parser.parse_args(argv)
    if args.pilot == args.play:
        parser.error("choose exactly one of --pilot or --play")
    if args.campaign == "trajectory16" and args.trajectory != "all":
        parser.error("trajectory16 always assigns T1-T16 to the 16 lanes")
    if args.max_runs < 1 or args.max_runs > 21:
        parser.error("max-runs must be between 1 and 21")
    if not all(math.isfinite(v) for v in (*args.mount_xyz, *args.mount_rpy)):
        parser.error("mount values must be finite")
    root = args.results.resolve()
    root.mkdir(parents=True, exist_ok=True)
    scenes = build_condition_scenes(root / "models" / "conditions", args.mount_xyz,
                                    args.mount_rpy, args.payload_spec)
    fingerprint_hash = hashlib.sha256()
    for path in (Path(__file__), Path(__file__).with_name("trajectory_generator.py"),
                 args.payload_spec, *scenes.values()):
        fingerprint_hash.update(path.read_bytes())
    fingerprint = fingerprint_hash.hexdigest()
    if args.pilot:
        if args.campaign == "trajectory16":
            c, g, p = CONDITIONS[2]  # mixed-duration pilot with one tool/payload condition
            lanes = [dict(condition=c, gripper=g, payload_kg=p, repeat=1, trajectory=t)
                     for t in ("T1", "T5", "T15", "T16")]
            report = collect_batch(root / "pilot", scenes, lanes, None, 0, 2, pilot=True)
        else:
            lanes = [dict(condition=c, gripper=g, payload_kg=p, repeat=1)
                     for c, g, p in (CONDITIONS[0], CONDITIONS[2], CONDITIONS[4], CONDITIONS[6])]
            report = collect_batch(root / "pilot", scenes, lanes, "T5", 0, 2, pilot=True)
        report["pass"] = (report["max_tracking_error_rad"] <= 0.05
                          and report["cross_robot_contacts"] == 0
                          and report["torque_limit_samples"] == 0)
        report["fingerprint"] = fingerprint
        report["campaign"] = args.campaign
        (root / "pilot_report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2), flush=True)
        return 0 if report["pass"] else 2
    report_path = root / "pilot_report.json"
    if (not report_path.is_file() or not json.loads(report_path.read_text()).get("pass")
            or json.loads(report_path.read_text()).get("fingerprint") != fingerprint
            or json.loads(report_path.read_text()).get("campaign", "legacy") != args.campaign):
        raise RuntimeError("2x2 pilot must pass before 4x4 collection")
    shards = [dict(condition=c, gripper=g, payload_kg=p, repeat=repeat)
              for c, g, p in CONDITIONS for repeat in range(1, 4)]
    def complete(folder):
        metadata_path = folder / "metadata.json"
        if not metadata_path.is_file() or not (folder / "dynamics.csv").is_file():
            return False
        try:
            metadata = json.loads(metadata_path.read_text())
            bag_relative = metadata["rosbag_relative_path"]
            report = metadata["report"]
            bag_path = (root / bag_relative).resolve()
            return (bag_path.is_relative_to(root) and (bag_path / "metadata.yaml").is_file()
                    and report["max_tracking_error_rad"] <= 0.05
                    and report["torque_limit_samples"] == 0
                    and report["cross_robot_contacts"] == 0)
        except (KeyError, ValueError, OSError, TypeError):
            return False
    if args.campaign == "trajectory16":
        runs = [(c, g, p, repeat) for c, g, p in CONDITIONS for repeat in range(1, 4)]
        for run_index, (condition, gripper, payload, repeat) in enumerate(runs):
            if run_index >= args.max_runs:
                break
            lanes = [dict(condition=condition, gripper=gripper, payload_kg=payload,
                          repeat=repeat, trajectory=f"T{i}") for i in range(1, 17)]
            folders = [root / "simulation" / lane["trajectory"] / condition /
                       f"trial_{repeat:03d}" for lane in lanes]
            if all(complete(folder) for folder in folders):
                print(f"skip complete scene {run_index+1}/21: {condition} repeat {repeat}", flush=True)
                continue
            print(f"collect scene {run_index+1}/21: {condition} repeat {repeat}, T1-T16 in parallel", flush=True)
            report = collect_batch(root, scenes, lanes, None, run_index, 4)
            print(json.dumps(report), flush=True)
        return 0
    for batch_index, start in enumerate(range(0, len(shards), 16)):
        if batch_index >= args.max_batches:
            break
        lanes = shards[start:start + 16]
        trajectories = (f"T{i}" for i in range(1, 15)) if args.trajectory == "all" else (args.trajectory,)
        for trajectory in trajectories:
            folders = [root / "simulation" / trajectory / lane["condition"] /
                       f"trial_{lane['repeat']:03d}" for lane in lanes]
            if all(complete(folder) for folder in folders):
                print(f"skip completed batch {batch_index} {trajectory}", flush=True)
                continue
            print(f"collect batch {batch_index} {trajectory}: {len(lanes)} lanes", flush=True)
            report = collect_batch(root, scenes, lanes, trajectory, batch_index, 4)
            print(json.dumps(report), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
