"""Generate bounded, repeatable A0509 left/right research trajectories.

The output is deliberately simulation-only. A physical tool, nearby obstacles,
the robot's current state, and the 100 Hz real command path need on-site review.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from .trajectory import (ACCELERATION_LIMITS, JOINT_NAMES, POSITION_LIMITS,
                         VELOCITY_LIMITS, Point, load_trajectory)

HZ = 100
FORWARD_MM = 100.0
ZERO = (0.0,) * 6
HOME = (0.0, -0.4, 1.4, -0.07, 1.8, math.radians(45.0))
REAR_WALL_X_M = -0.35
REAL_VELOCITY_CAP = math.radians(30.0)
REAL_ACCELERATION_CAP = math.radians(30.0)
JERK_CAP_RAD_S3 = 5.0  # experiment design cap; no verified A0509 jerk rating

PROFILES = {
    "p1_slow": (8.0, 8.0, 8.0, 8.0),
    "p2_medium": (5.0, 5.0, 5.0, 5.0),
    "p3_variable": (8.0, 5.0, 7.0, 5.0),
    "p4_reversal": (6.0, 6.0, 6.0, 6.0),
    "p5_multijoint": (7.0, 7.0, 7.0, 7.0),
}


def _scene_path() -> Path:
    local = (Path(__file__).resolve().parents[2] / "doosan-robot2" /
             "dsr_description2" / "mujoco_models" / "a0509" / "scene_forward_wall.xml")
    if local.is_file():
        return local
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = (Path(get_package_share_directory("dsr_description2")) /
                     "mujoco_models" / "a0509" / "scene_forward_wall.xml")
        if installed.is_file():
            return installed
    except (ImportError, LookupError):
        pass
    raise FileNotFoundError("A0509 rear-wall MuJoCo scene is unavailable")


def _observer(scene: Path):
    from a0509_dynamics_experiment.simulation_executor import MujocoObserver
    # No physical gripper is known. The bare flange (link_6) is the model TCP.
    return MujocoObserver(scene, tcp_site="no_tcp_site_on_bare_arm")


def _quintic(u: float) -> tuple[float, float, float, float]:
    return (10*u**3 - 15*u**4 + 6*u**5,
            30*u**2 - 60*u**3 + 30*u**4,
            60*u - 180*u**2 + 120*u**3,
            60 - 360*u + 360*u**2)


def _point_dict(point: Point) -> dict:
    return {"time_s": round(point.time_s, 8),
            "q_rad": [round(v, 12) for v in point.q_rad],
            "dq_rad_s": [round(v, 12) for v in point.dq_rad_s],
            "ddq_rad_s2": [round(v, 12) for v in point.ddq_rad_s2]}


def _position_ik(observer, target_mm, seed):
    import numpy as np

    # Keep wrist posture while solving translational IK with J1-J3. This
    # avoids unnecessary large wrist excursions in the narrow workspace.
    q = np.asarray(seed, dtype=float).copy()
    target_m = np.asarray(target_mm) / 1000.0
    for _ in range(200):
        observer._set_state(q)
        error = target_m - observer._tcp_pos()
        if np.linalg.norm(error) < .0001:
            return q
        linear = np.zeros((3, observer.model.nv))
        angular = np.zeros_like(linear)
        observer.mj.mj_jacBody(observer.model, observer.data, linear, angular,
                               observer.body_id)
        jac = linear[:, observer.dof_addr[:3]]
        step = jac.T @ np.linalg.solve(jac @ jac.T + .000001 * np.eye(3), error)
        q[:3] += np.clip(step, -.05, .05)
    raise ValueError(f"position IK failed for {target_mm} mm")


def _waypoints(observer, half_width_mm: float, center_offset_mm: float,
               start_joint_rad, forward_mm: float):
    import numpy as np

    start = np.asarray(start_joint_rad, dtype=float)
    if start.shape != (6,) or not np.isfinite(start).all():
        raise ValueError("start_joint_rad needs six finite joint values")
    start_tcp = np.asarray(observer.require_contact_free(start))
    forward = _position_ik(observer, start_tcp + [forward_mm, 0, 0], start)
    center = forward.copy()
    observer.require_contact_free(forward)
    if center_offset_mm:
        target = np.asarray(observer.tcp_mm(center)) + np.array([0, center_offset_mm, 0])
        center = _position_ik(observer, target, center)
    center_tcp = np.asarray(observer.require_contact_free(center))
    right = _position_ik(observer, center_tcp + [0, half_width_mm, 0], center)
    left = _position_ik(observer, center_tcp - [0, half_width_mm, 0], center)
    for label, q in (("center", center), ("right", right), ("left", left)):
        observer.require_contact_free(q)
        for j, (low, high) in enumerate(POSITION_LIMITS):
            if not low + 0.05 <= q[j] <= high - 0.05:
                raise ValueError(f"{label} J{j+1} has less than 0.05 rad joint margin")
    return start, forward, center, right, left, start_tcp, center_tcp


def _segments(profile: str, center, right, left):
    import numpy as np

    stops = [center, right, center, left, center]
    if profile == "p5_multijoint":
        # Wrist variation adds joint excitation while the full arm and TCP are
        # still checked at every sample. If this is infeasible P5 is omitted.
        right = center + .90 * (right - center) + np.array([0, 0, 0.02, 0.06, -0.04, 0.08])
        left = center + .90 * (left - center) + np.array([0, 0, -0.02, -0.06, 0.04, -0.08])
        stops = [center, right, center, left, center]
    return stops


def make_cycle(profile: str, center, right, left, durations=None):
    if profile not in PROFILES:
        raise ValueError(f"unknown profile {profile}")
    import numpy as np

    waypoints = _segments(profile, center, right, left)
    samples = [Point(0.0, tuple(float(v) for v in center), ZERO, ZERO)]
    intervals = []
    elapsed_ticks = 0
    peak_jerk = [0.0] * 6
    segment_times = PROFILES[profile] if durations is None else durations
    if (not isinstance(segment_times, (list, tuple)) or len(segment_times) != 4 or
            any(not isinstance(t, (int, float)) or not math.isfinite(t) or t <= 0
                for t in segment_times)):
        raise ValueError("each profile needs four positive finite segment durations")
    for part, (a, b, duration) in enumerate(zip(waypoints, waypoints[1:], segment_times)):
        ticks = round(duration * HZ)
        if abs(ticks / HZ - duration) > 1e-9:
            raise ValueError("segment duration must be a 10 ms multiple")
        begin = elapsed_ticks / HZ
        delta = b - a
        for k in range(1, ticks + 1):
            s, ds, dds, jerk = _quintic(k / ticks)
            q = a + delta * s
            dq = delta * ds / duration
            ddq = delta * dds / duration**2
            for j in range(6):
                peak_jerk[j] = max(peak_jerk[j], abs(float(delta[j] * jerk / duration**3)))
            samples.append(Point((elapsed_ticks + k) / HZ,
                                 tuple(float(v) for v in q), tuple(float(v) for v in dq),
                                 tuple(float(v) for v in ddq)))
        elapsed_ticks += ticks
        intervals.append({"kind": "motion", "index": part + 1,
                          "start_s": begin, "end_s": elapsed_ticks / HZ})
        if profile == "p4_reversal" and part in (0, 2):
            begin = elapsed_ticks / HZ
            for k in range(1, HZ + 1):
                samples.append(Point((elapsed_ticks + k) / HZ,
                                     tuple(float(v) for v in b), ZERO, ZERO))
            elapsed_ticks += HZ
            intervals.append({"kind": "hold", "start_s": begin,
                              "end_s": elapsed_ticks / HZ})
    return samples, intervals, peak_jerk


def make_setup(start, forward, center):
    import numpy as np

    stops = [(np.asarray(start), np.asarray(forward), 10.0, "forward_100mm")]
    if not np.allclose(forward, center, atol=1e-10):
        stops.append((np.asarray(forward), np.asarray(center), 6.0, "center_offset"))
    samples = [Point(0.0, tuple(float(v) for v in start), ZERO, ZERO)]
    intervals, peak_jerk = [], [0.0] * 6
    elapsed_ticks = 0
    for a, b, duration, kind in stops:
        ticks = round(duration * HZ)
        delta = b - a
        begin = elapsed_ticks / HZ
        for k in range(1, ticks + 1):
            s, ds, dds, jerk = _quintic(k / ticks)
            q, dq, ddq = a + delta*s, delta*ds/duration, delta*dds/duration**2
            for j in range(6):
                peak_jerk[j] = max(peak_jerk[j], abs(float(delta[j]*jerk/duration**3)))
            samples.append(Point((elapsed_ticks + k)/HZ,
                                 tuple(float(v) for v in q), tuple(float(v) for v in dq),
                                 tuple(float(v) for v in ddq)))
        elapsed_ticks += ticks
        intervals.append({"kind": kind, "start_s": begin, "end_s": elapsed_ticks/HZ})
    return samples, intervals, peak_jerk


def _jacobian_singular_value(observer):
    import numpy as np

    linear = np.zeros((3, observer.model.nv))
    angular = np.zeros_like(linear)
    observer.mj.mj_jacBody(observer.model, observer.data, linear, angular, observer.body_id)
    jac = np.vstack((linear[:, observer.dof_addr], 0.3 * angular[:, observer.dof_addr]))
    return float(np.linalg.svd(jac, compute_uv=False)[-1])


def validate_cycle(observer, samples, peak_jerk, half_width_mm, center_tcp_mm,
                   *, closed: bool = True, max_x_span_mm: float = 30.0,
                   expected_forward_mm: float | None = None,
                   max_total_y_mm: float = 300.0,
                   max_z_span_mm: float = 30.0):
    import numpy as np

    if closed and not np.allclose(samples[0].q_rad, samples[-1].q_rad, atol=1e-10):
        raise ValueError("cycle endpoints differ")
    for end in (samples[0], samples[-1]):
        if max(abs(v) for v in (*end.dq_rad_s, *end.ddq_rad_s2)) > 1e-8:
            raise ValueError("cycle boundary velocity or acceleration is nonzero")
    if any(value > JERK_CAP_RAD_S3 for value in peak_jerk):
        raise ValueError("experiment jerk cap exceeded")
    q = np.asarray([p.q_rad for p in samples])
    dq = np.asarray([p.dq_rad_s for p in samples])
    ddq = np.asarray([p.ddq_rad_s2 for p in samples])
    if not (np.isfinite(q).all() and np.isfinite(dq).all() and np.isfinite(ddq).all()):
        raise ValueError("nonfinite point")
    times = np.asarray([p.time_s for p in samples])
    if np.max(np.abs(np.diff(times) - .01)) > 1e-8:
        raise ValueError("100 Hz time gap or duplicate")
    for j, (low, high) in enumerate(POSITION_LIMITS):
        if q[:, j].min() < low + .05 or q[:, j].max() > high - .05:
            raise ValueError(f"J{j+1} position or 0.05 rad margin violated")
        if (np.abs(dq[:, j]).max() > min(VELOCITY_LIMITS[j], REAL_VELOCITY_CAP) or
                np.abs(ddq[:, j]).max() > min(ACCELERATION_LIMITS[j], REAL_ACCELERATION_CAP)):
            raise ValueError(f"J{j+1} velocity or acceleration cap violated")
    # Compare interior finite differences with the analytic derivatives.
    dq_fd = np.gradient(q, .01, axis=0, edge_order=2)
    ddq_fd = np.gradient(dq, .01, axis=0, edge_order=2)
    if (np.max(np.abs(dq_fd[1:-1] - dq[1:-1])) > .002 or
            np.max(np.abs(ddq_fd[1:-1] - ddq[1:-1])) > .004):
        raise ValueError("position/velocity/acceleration are numerically inconsistent")
    xyz = np.empty((len(samples), 3))
    min_singular = math.inf
    link_y = [math.inf, -math.inf]
    for index, point in enumerate(samples):
        xyz[index] = observer.require_contact_free(point.q_rad)
        min_singular = min(min_singular, _jacobian_singular_value(observer))
        for name in ("link_1", "link_2", "link_3", "link_4", "link_5", "link_6"):
            body = observer.mj.mj_name2id(observer.model, observer.mj.mjtObj.mjOBJ_BODY, name)
            y = float(observer.data.xpos[body, 1] * 1000)
            link_y[0], link_y[1] = min(link_y[0], y), max(link_y[1], y)
    if min_singular < .03:
        raise ValueError(f"full Jacobian near singularity: minimum {min_singular:.4f}")
    y_offset = xyz[:, 1] - center_tcp_mm[1]
    if max(abs(y_offset.min()), abs(y_offset.max())) > half_width_mm + 2.0:
        raise ValueError("TCP exceeded its configured lateral band")
    if np.ptp(xyz[:, 1]) > max_total_y_mm:
        raise ValueError(f"TCP exceeded {max_total_y_mm:g} mm total lateral workspace")
    if np.ptp(xyz[:, 0]) > max_x_span_mm or np.ptp(xyz[:, 2]) > max_z_span_mm:
        raise ValueError("TCP X or Z excursion exceeded the design cap")
    if expected_forward_mm is not None:
        if abs((xyz[-1, 0] - xyz[0, 0]) - expected_forward_mm) > 1.0:
            raise ValueError("setup endpoint is not the requested forward distance")
        if xyz[:, 0].min() < xyz[0, 0] - 2.0:
            raise ValueError("setup moves behind its starting TCP X")
    return {"sample_count": len(samples), "duration_s": samples[-1].time_s,
            "tcp_min_mm": xyz.min(axis=0).round(3).tolist(),
            "tcp_max_mm": xyz.max(axis=0).round(3).tolist(),
            "tcp_span_mm": np.ptp(xyz, axis=0).round(3).tolist(),
            "tcp_center_mm": np.asarray(center_tcp_mm).round(3).tolist(),
            "model_link_origin_y_range_mm": [round(v, 3) for v in link_y],
            "minimum_weighted_jacobian_singular_value": round(min_singular, 6),
            "max_velocity_rad_s": np.abs(dq).max(axis=0).round(6).tolist(),
            "max_acceleration_rad_s2": np.abs(ddq).max(axis=0).round(6).tolist(),
            "max_jerk_rad_s3": [round(v, 6) for v in peak_jerk],
            "position_min_rad": q.min(axis=0).round(6).tolist(),
            "position_max_rad": q.max(axis=0).round(6).tolist(),
            "model_contact_checks": len(samples),
            "time_grid_passed": True, "derivative_check_passed": True,
            "boundary_rest_passed": True, "cycle_boundary_c2_passed": closed,
            "model_collision_free": True,
            "joint_and_kinematic_checks_passed": True}, xyz


def simulate_cycle(observer, samples, *, contact_is_blocking=None):
    """Direct MuJoCo preflight using the grid collector's feedforward/PD law."""
    import numpy as np

    mj, model = observer.mj, observer.model
    if contact_is_blocking is None:
        contact_is_blocking = lambda state: state.ncon > 0
    data, target = mj.MjData(model), mj.MjData(model)
    data.qpos[:] = model.qpos0
    for j, addr in enumerate(observer.qpos_addr):
        data.qpos[addr] = samples[0].q_rad[j]
    mj.mj_forward(model, data)
    if contact_is_blocking(data):
        raise ValueError("MuJoCo initial state has contact")
    peak_error = peak_command = 0.0
    torque_limit_samples = contact_steps = 0
    limits = (194, 194, 163, 50, 50, 50)
    if abs(model.opt.timestep - .002) > 1e-9:
        raise ValueError("MuJoCo scene timestep is not 2 ms")
    for point in samples[1:]:
        target.qpos[:] = data.qpos
        target.qvel[:] = 0
        target.qacc[:] = 0
        for j, (qa, da) in enumerate(zip(observer.qpos_addr, observer.dof_addr)):
            target.qpos[qa] = point.q_rad[j]
            target.qvel[da] = point.dq_rad_s[j]
            target.qacc[da] = point.ddq_rad_s2[j]
        mj.mj_forward(model, target)
        for j, da in enumerate(observer.dof_addr):
            target.qacc[da] = point.ddq_rad_s2[j]
        mj.mj_inverse(model, target)
        for _ in range(5):
            data.ctrl[:] = point.q_rad
            mj.mj_forward(model, data)
            for j, (qa, da) in enumerate(zip(observer.qpos_addr, observer.dof_addr)):
                command = (target.qfrc_inverse[da] + 150 * (point.q_rad[j] - data.qpos[qa]) +
                           20 * (point.dq_rad_s[j] - data.qvel[da]))
                peak_command = max(peak_command, abs(float(command)))
                if abs(command) > limits[j]:
                    torque_limit_samples += 1
                data.qfrc_applied[da] = np.clip(command, -limits[j], limits[j]) - data.qfrc_actuator[da]
            mj.mj_step(model, data)
            if contact_is_blocking(data):
                contact_steps += 1
                break
        if contact_steps:
            break
        peak_error = max(peak_error, max(abs(float(data.qpos[qa] - point.q_rad[j]))
                                         for j, qa in enumerate(observer.qpos_addr)))
    return {"passed": contact_steps == 0 and torque_limit_samples == 0 and peak_error <= .05,
            "max_tracking_error_rad": round(peak_error, 6),
            "max_command_nm": round(peak_command, 4),
            "torque_limit_samples": torque_limit_samples,
            "contact_steps": contact_steps,
            "simulated_100hz_setpoints": len(samples) - 1 if not contact_steps else None,
            "controller": "MuJoCo inverse dynamics + 150/20 PD, 2 ms physics step"}


def _plot(path: Path, samples, xyz, intervals, profile):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    t = np.asarray([p.time_s for p in samples])
    q = np.asarray([p.q_rad for p in samples])
    dq = np.asarray([p.dq_rad_s for p in samples])
    ddq = np.asarray([p.ddq_rad_s2 for p in samples])
    fig = plt.figure(figsize=(15, 12), constrained_layout=True)
    grid = fig.add_gridspec(3, 2)
    ax3 = fig.add_subplot(grid[0, 0], projection="3d")
    ax3.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], lw=1.5)
    ax3.set(xlabel="X mm", ylabel="Y mm", zlabel="Z mm", title="Bare-flange TCP path")
    ax = [fig.add_subplot(grid[0, 1]), fig.add_subplot(grid[1, 0]),
          fig.add_subplot(grid[1, 1]), fig.add_subplot(grid[2, :])]
    for j, name in enumerate("XYZ"):
        ax[0].plot(t, xyz[:, j], label=name)
    for panel, values, unit in zip(ax[1:], (q, dq, ddq), ("rad", "rad/s", "rad/s²")):
        for j in range(6):
            panel.plot(t, values[:, j], label=f"J{j+1}", lw=.8)
        panel.set_ylabel(unit)
    for panel in ax:
        for segment in intervals:
            if segment["kind"] == "hold":
                panel.axvspan(segment["start_s"], segment["end_s"], color="gray", alpha=.2)
            else:
                panel.axvline(segment["end_s"], color="gray", alpha=.15, lw=.7)
        panel.set_xlabel("Cycle time (s)")
        panel.legend(ncol=6, fontsize=7, loc="best")
    role = "one-time forward setup" if profile == "forward_10cm_setup" else "one 100 Hz cycle"
    fig.suptitle(f"A0509 {profile} | {role} | simulation-only")
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _plot_repeats(path: Path, samples, xyz, intervals, profile, repeats):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    shown = min(repeats, 10)
    cycle_time = samples[-1].time_s
    t = np.asarray([p.time_s for p in samples])
    fig, ax = plt.subplots(figsize=(14, 3.8), constrained_layout=True)
    for repeat in range(shown):
        offset = repeat * cycle_time
        ax.plot(t + offset, xyz[:, 1], color="tab:blue", lw=.8)
        ax.axvline(offset, color="gray", alpha=.35, lw=.8)
        for interval in intervals:
            if interval["kind"] == "hold":
                ax.axvspan(offset + interval["start_s"], offset + interval["end_s"],
                           color="tab:orange", alpha=.25)
    ax.axvline(shown * cycle_time, color="gray", alpha=.35, lw=.8)
    ax.set(xlabel="Time (s)", ylabel="Bare-flange TCP Y (mm)",
           title=f"{profile}: {shown} of {repeats} configured cycles; orange = deliberate hold")
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _cycle_json(profile, samples):
    setup = profile == "forward_10cm_setup"
    return {"name": "a0509_lr_" + profile,
            "joint_names": list(JOINT_NAMES), "units": "rad",
            "description": ("100 Hz C2 forward setup, run once before lateral cycles; bare-flange MuJoCo preview only"
                            if setup else
                            "100 Hz closed C2 left/right research cycle; bare-flange MuJoCo preview only"),
            "rear_wall_x_m": REAR_WALL_X_M, "forward_axis": "DR_BASE +X",
            "lateral_axis": "DR_BASE Y", "gripper_type": "none",
            "trajectory_role": "setup_once" if setup else "repeatable_cycle",
            "execution_scope": "simulation_only", "real_motion_validated": False,
            "physical_tool_verified": False,
            "simulation_scene": "dsr_description2/mujoco_models/a0509/scene_forward_wall.xml",
            "points": [_point_dict(p) for p in samples]}


def _expand(samples, repeats):
    cycle_ticks = len(samples) - 1
    if cycle_ticks * repeats > 600 * HZ or cycle_ticks * repeats + 1 > 100000:
        raise ValueError("expanded path exceeds existing JSON loader limits")
    result = list(samples)
    for repeat in range(1, repeats):
        result.extend(Point((repeat * cycle_ticks + i) / HZ, point.q_rad,
                            point.dq_rad_s, point.ddq_rad_s2)
                      for i, point in enumerate(samples[1:], start=1))
    return result


def _setup_plus_cycles(setup_samples, cycle_samples, repeats):
    cycles = _expand(cycle_samples, repeats)
    setup_ticks = len(setup_samples) - 1
    if (setup_ticks + len(cycles) - 1) > 600 * HZ:
        raise ValueError("setup plus expanded cycles exceed the 600 s JSON loader limit")
    combined = list(setup_samples)
    combined.extend(Point((setup_ticks + i) / HZ, point.q_rad,
                          point.dq_rad_s, point.ddq_rad_s2)
                    for i, point in enumerate(cycles[1:], start=1))
    return combined


def generate_dataset(output: Path, *, half_width_mm: float = 100.0,
                     center_offset_mm: float = 0.0, repeats: int = 10,
                     expand_repeats: bool = False,
                     cycle_segment_seconds: dict | None = None,
                     start_joint_rad=HOME) -> dict:
    if (not math.isfinite(half_width_mm) or not 0 < half_width_mm <= 120 or
            not math.isfinite(center_offset_mm) or
            abs(center_offset_mm) + half_width_mm > 150):
        raise ValueError("lateral band must fit inside center ±150 mm; half-width is at most 120 mm")
    if not isinstance(repeats, int) or isinstance(repeats, bool) or not 1 <= repeats <= 1000:
        raise ValueError("repeats must be a finite integer from 1 to 1000")
    if cycle_segment_seconds is not None and not isinstance(cycle_segment_seconds, dict):
        raise ValueError("cycle_segment_seconds must be a profile-to-durations object")
    durations = {**PROFILES, **(cycle_segment_seconds or {})}
    if expand_repeats:
        longest_cycle_s = max(sum(durations[name]) + (2 if name == "p4_reversal" else 0)
                              for name in PROFILES)
        if longest_cycle_s * repeats + 16 > 600:
            raise ValueError("expanded repeats exceed the existing 600 s JSON loader limit")
    scene = _scene_path()
    observer = _observer(scene)
    start, forward, center, right, left, start_tcp, center_tcp = _waypoints(
        observer, half_width_mm, center_offset_mm, start_joint_rad, FORWARD_MM)
    output.mkdir(parents=True, exist_ok=True)
    report = {"status": "simulation_only", "model_scene": str(scene),
              "assumed_tcp": "bare link_6 origin; physical tool unknown",
              "rear_wall_x_m": REAR_WALL_X_M, "lateral_axis": "DR_BASE Y",
              "configured_half_width_mm": half_width_mm,
              "configured_center_offset_mm": center_offset_mm,
              "setup_forward_mm": FORWARD_MM,
              "configured_start_joint_rad": [float(v) for v in start],
              "physical_workspace_verified": False, "physical_gripper_verified": False,
              "real_current_pose_verified": False, "real_100hz_interface_verified": False,
              "real_robot_executable": False,
              "notes": ["MuJoCo model includes the bare arm, floor and x=-0.35 m rear wall.",
                        "Other lab obstacles and the mounted tool geometry are unknown.",
                        "Original practice J5=115.02 deg is within the 2.1 rad configured limit but only about 0.093 rad below it; the default candidate uses 1.8 rad.",
                        "No real robot command was sent."],
              "setup": {}, "paths": {}}
    setup_samples = None
    try:
        setup_samples, setup_intervals, setup_jerk = make_setup(start, forward, center)
        setup_result, setup_xyz = validate_cycle(
            observer, setup_samples, setup_jerk, half_width_mm, start_tcp,
            closed=False, max_x_span_mm=110.0, expected_forward_mm=FORWARD_MM)
        setup_sim = simulate_cycle(observer, setup_samples)
        if not setup_sim["passed"]:
            raise ValueError(f"MuJoCo setup preflight failed: {setup_sim}")
        setup_result.update({"intervals": setup_intervals,
                             "simulation_preflight": setup_sim,
                             "simulation_usable": True,
                             "real_robot_executable": False})
        setup_json = output / "a0509_lr_forward_10cm_setup.json"
        setup_json.write_text(json.dumps(_cycle_json("forward_10cm_setup", setup_samples),
                                         ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        load_trajectory(setup_json)
        setup_result["json"] = setup_json.name
        setup_plot = output / "a0509_lr_forward_10cm_setup_preview.png"
        _plot(setup_plot, setup_samples, setup_xyz, setup_intervals, "forward_10cm_setup")
        setup_result["preview"] = setup_plot.name
        report["setup"] = setup_result
    except (ValueError, RuntimeError) as error:
        report["setup"] = {"status": "failed", "reason": str(error),
                           "real_robot_executable": False}
        setup_samples = None
        for stale in (output / "a0509_lr_forward_10cm_setup.json",
                      output / "a0509_lr_forward_10cm_setup_preview.png"):
            if stale.is_file():
                stale.unlink()
    for profile in PROFILES:
        try:
            if setup_samples is None:
                raise ValueError("forward setup did not pass; lateral cycle omitted")
            samples, intervals, peak_jerk = make_cycle(profile, center, right, left,
                                                        durations.get(profile))
            result, xyz = validate_cycle(observer, samples, peak_jerk, half_width_mm, center_tcp)
            sim = simulate_cycle(observer, samples)
            result.update({"intervals": intervals, "simulation_preflight": sim,
                           "simulation_usable": sim["passed"],
                           "real_robot_executable": False})
            if not sim["passed"]:
                raise ValueError(f"MuJoCo preflight failed: {sim}")
            json_path = output / f"a0509_lr_{profile}.json"
            json_path.write_text(json.dumps(_cycle_json(profile, samples),
                                            ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            load_trajectory(json_path)
            result["json"] = json_path.name
            plot_path = output / f"a0509_lr_{profile}_preview.png"
            _plot(plot_path, samples, xyz, intervals, profile)
            result["preview"] = plot_path.name
            if repeats > 1:
                repeat_plot = output / f"a0509_lr_{profile}_repeats_preview.png"
                _plot_repeats(repeat_plot, samples, xyz, intervals, profile, repeats)
                result["repeat_preview"] = repeat_plot.name
            if expand_repeats:
                expanded = output / f"a0509_lr_{profile}_setup_{repeats}cycles.json"
                expanded.write_text(json.dumps(_cycle_json(profile, _setup_plus_cycles(setup_samples, samples, repeats)),
                                               ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                load_trajectory(expanded)
                result["expanded_json"] = expanded.name
            report["paths"][profile] = result
        except (ValueError, RuntimeError) as error:
            for stale in (output / f"a0509_lr_{profile}.json",
                          output / f"a0509_lr_{profile}_preview.png",
                          output / f"a0509_lr_{profile}_repeats_preview.png",
                          output / f"a0509_lr_{profile}_setup_{repeats}cycles.json"):
                if stale.is_file():
                    stale.unlink()
            report["paths"][profile] = {"status": "failed", "reason": str(error),
                                         "real_robot_executable": False}
    config = {"experiment_id": "a0509_lr_sim2real", "sample_hz": HZ,
              "cycle_repeats": repeats, "repeat_mode": "one_cycle_json_plus_config",
              "expanded_json_generated": expand_repeats,
              "setup_file": report["setup"].get("json"),
              "setup_once_before_cycles": True,
              "setup_forward_mm": FORWARD_MM,
              "start_joint_rad": [float(v) for v in start],
              "start_pose_source": "configured_candidate; not read from live robot",
              "cycle_segment_seconds": {name: list(durations[name]) for name in PROFILES},
              "lateral_axis": "DR_BASE Y", "half_width_mm": half_width_mm,
              "center_offset_mm": center_offset_mm,
              "maximum_total_lateral_width_mm": 300,
              "supported_half_width_mm": [100, 120],
              "rear_wall_x_m": REAR_WALL_X_M,
              "gripper_type": "none", "payload_mass_kg": 0.0,
              "control_mode": "unverified_100hz_joint_stream",
              "execution_scope": "simulation_only", "real_auto_run": False,
              "cycle_files": [item["json"] for item in report["paths"].values() if "json" in item],
              "record_schema": ["experiment_id", "cycle_index", "mode", "command_time_s",
                                "measurement_time_s", "target_q_rad", "target_dq_rad_s",
                                "target_ddq_rad_s2", "actual_q_rad", "actual_dq_rad_s",
                                "actual_ddq_est_rad_s2",
                                "torque_signal", "torque_signal_definition", "gripper_type",
                                "payload_mass_kg", "control_mode"],
              "analysis_policy": {
                  "tracking_error": "compare target_q_rad with actual_q_rad per joint",
                  "dynamics_gap": "apply actual q, dq and estimated ddq at measurement time to the matching MuJoCo condition; compare qfrc_inverse with a separately characterized real torque signal",
                  "torque_ground_truth_verified": False,
                  "split_metrics": ["joint_tracking_error", "torque_model_residual"]},
              "stop_conditions": ["communication_loss", "stale_feedback", "joint_limit_margin",
                                  "protective_stop", "unexpected_tracking_error"]}
    (output / "a0509_lr_run_config.json").write_text(
        json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (output / "a0509_lr_validation.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Generate simulation-only A0509 lateral cycles")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--config", type=Path, help="read width, center and repeats from a run config JSON")
    parser.add_argument("--half-width-mm", type=float, default=None)
    parser.add_argument("--center-offset-mm", type=float, default=None)
    parser.add_argument("--repeats", type=int, default=None)
    parser.add_argument("--expand-repeats", action="store_true")
    args = parser.parse_args(argv)
    settings = json.loads(args.config.read_text(encoding="utf-8")) if args.config else {}
    output = args.output or (args.config.parent if args.config else
                            Path.cwd() / "a0509_lr_outputs")
    result = generate_dataset(output,
                              half_width_mm=(args.half_width_mm if args.half_width_mm is not None
                                             else settings.get("half_width_mm", 100.0)),
                              center_offset_mm=(args.center_offset_mm if args.center_offset_mm is not None
                                                else settings.get("center_offset_mm", 0.0)),
                              repeats=(args.repeats if args.repeats is not None
                                       else settings.get("cycle_repeats", 10)),
                              expand_repeats=args.expand_repeats,
                              cycle_segment_seconds=settings.get("cycle_segment_seconds"),
                              start_joint_rad=settings.get("start_joint_rad", HOME))
    print(json.dumps({name: {key: value for key, value in path.items()
                             if key in ("sample_count", "duration_s", "tcp_span_mm", "simulation_usable", "reason")}
                      for name, path in result["paths"].items()}, indent=2))
    return 0 if ("json" in result["setup"] and
                 all("json" in path and path["simulation_usable"]
                     for path in result["paths"].values())) else 1


if __name__ == "__main__":
    raise SystemExit(main())
