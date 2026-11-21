"""Four forward-region A0509 research cycles for a narrow left/right workspace.

All plans are MuJoCo model candidates. No real robot command is sent here.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from .lr_trajectory import (FORWARD_MM, HOME, HZ, ZERO, _observer, _plot,
                            _position_ik, _quintic, _scene_path, make_setup,
                            simulate_cycle, validate_cycle)
from .trajectory import JOINT_NAMES, Point, load_trajectory

PROFILE_NAMES = ("f1_slow_forward", "f2_medium_forward",
                 "f3_front_postures", "f4_wrist_and_holds")
DEFAULT_SEGMENTS = {
    "f1_slow_forward": (10.0, 10.0),
    "f2_medium_forward": (7.0, 7.0),
    "f3_front_postures": (9.0, 7.0, 10.0, 7.0, 9.0),
    "f4_wrist_and_holds": (9.0, 7.5, 7.5, 7.5, 7.5, 9.0),
}
MAX_TOTAL_LATERAL_MM = 400.0
FRONT_DESIGN_CAP_MM = 260.0
DEFAULT_OPERATING_LIFT_MM = 60.0
F3_LATERAL_OFFSET_MM = 20.0  # Narrow one-sided shift; use upper free space.


def _plan_waypoints(observer, center, center_tcp_mm):
    import numpy as np

    def pose(dx, dy=0, dz=0):
        return _position_ik(observer, center_tcp_mm + [dx, dy, dz], center)

    far = pose(200)
    multi = [pose(120, F3_LATERAL_OFFSET_MM, 20), pose(200, F3_LATERAL_OFFSET_MM, 20),
             pose(200), pose(120)]
    wrist_a = far + np.array([0, 0, 0, .15, -.12, .30])
    wrist_b = far + np.array([0, 0, 0, .30, -.24, -.30])
    return {
        "f1_slow_forward": [center, far, center],
        "f2_medium_forward": [center, far, center],
        "f3_front_postures": [center, *multi, center],
        "f4_wrist_and_holds": [center, far, wrist_a, far, wrist_b, far, center],
    }


def _sample_cycle(profile, waypoints, durations):
    if (len(durations) != len(waypoints) - 1 or
            any(not isinstance(t, (float, int)) or not math.isfinite(t) or t <= 0 or
                abs(round(t * HZ) / HZ - t) > 1e-9 for t in durations)):
        raise ValueError(f"{profile} requires positive 10 ms segment durations")
    samples = [Point(0.0, tuple(float(v) for v in waypoints[0]), ZERO, ZERO)]
    intervals, peak_jerk = [], [0.0] * 6
    elapsed_ticks = 0
    for index, (a, b, duration) in enumerate(zip(waypoints, waypoints[1:], durations)):
        ticks = round(duration * HZ)
        delta = b - a
        start_s = elapsed_ticks / HZ
        for k in range(1, ticks + 1):
            s, ds, dds, jerk = _quintic(k / ticks)
            q, dq, ddq = a + delta * s, delta * ds / duration, delta * dds / duration**2
            for joint in range(6):
                peak_jerk[joint] = max(peak_jerk[joint],
                                       abs(float(delta[joint] * jerk / duration**3)))
            samples.append(Point((elapsed_ticks + k) / HZ,
                                 tuple(float(v) for v in q),
                                 tuple(float(v) for v in dq),
                                 tuple(float(v) for v in ddq)))
        elapsed_ticks += ticks
        intervals.append({"kind": "motion", "index": index + 1,
                          "start_s": start_s, "end_s": elapsed_ticks / HZ})
        if ((profile == "f4_wrist_and_holds" and index in (1, 3)) or
                (profile == "f3_front_postures" and index in (1, 2))):
            start_s = elapsed_ticks / HZ
            for k in range(1, HZ + 1):
                samples.append(Point((elapsed_ticks + k) / HZ,
                                     tuple(float(v) for v in b), ZERO, ZERO))
            elapsed_ticks += HZ
            intervals.append({"kind": "hold", "start_s": start_s,
                              "end_s": elapsed_ticks / HZ})
    return samples, intervals, peak_jerk


def _trajectory_json(name, samples, *, setup: bool):
    return {
        "name": name, "joint_names": list(JOINT_NAMES), "units": "rad",
        "description": ("One-time +X 100 mm setup, then configured +Z lift before four forward-region cycles" if setup else
                        "100 Hz closed C2 forward-region research cycle"),
        "rear_wall_x_m": -.35, "forward_axis": "DR_BASE +X",
        "lateral_axis": "DR_BASE Y", "gripper_type": "none",
        "trajectory_role": "setup_once" if setup else "repeatable_cycle",
        "execution_scope": "simulation_only", "real_motion_validated": False,
        "physical_tool_verified": False,
        "simulation_scene": "dsr_description2/mujoco_models/a0509/scene_forward_wall.xml",
        "points": [{"time_s": round(p.time_s, 8),
                    "q_rad": [round(v, 12) for v in p.q_rad],
                    "dq_rad_s": [round(v, 12) for v in p.dq_rad_s],
                    "ddq_rad_s2": [round(v, 12) for v in p.ddq_rad_s2]}
                   for p in samples],
    }


def _plot_repeats(path, samples, xyz, intervals, profile, repeats):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    shown = min(repeats, 10)
    duration = samples[-1].time_s
    t = np.asarray([p.time_s for p in samples])
    fig, ax = plt.subplots(figsize=(14, 4), constrained_layout=True)
    for repeat in range(shown):
        offset = repeat * duration
        ax.plot(t + offset, xyz[:, 0], color="tab:blue", lw=.8)
        ax.axvline(offset, color="gray", alpha=.4, lw=.8)
        for interval in intervals:
            if interval["kind"] == "hold":
                ax.axvspan(offset + interval["start_s"], offset + interval["end_s"],
                           color="tab:orange", alpha=.3)
    ax.axvline(shown * duration, color="gray", alpha=.4, lw=.8)
    ax.set(xlabel="Time (s)", ylabel="Bare-flange TCP X (mm)",
           title=f"{profile}: {shown} of {repeats} cycles; orange = hold")
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _setup_plus_cycles(setup_samples, cycle_samples, repeats):
    setup_ticks, cycle_ticks = len(setup_samples) - 1, len(cycle_samples) - 1
    if setup_ticks + cycle_ticks * repeats > 600 * HZ:
        raise ValueError("setup plus repeats exceed the existing 600 s JSON loader limit")
    result = list(setup_samples)
    for repeat in range(repeats):
        result.extend(Point((setup_ticks + repeat * cycle_ticks + i) / HZ,
                            point.q_rad, point.dq_rad_s, point.ddq_rad_s2)
                      for i, point in enumerate(cycle_samples[1:], start=1))
    return result


def generate_front_dataset(output: Path, *, repeats: int = 10,
                           start_joint_rad=HOME,
                           segment_seconds: dict | None = None,
                           operating_lift_mm: float = DEFAULT_OPERATING_LIFT_MM,
                           expand_repeats: bool = False) -> dict:
    import numpy as np

    if not isinstance(repeats, int) or isinstance(repeats, bool) or not 1 <= repeats <= 1000:
        raise ValueError("repeats must be an integer from 1 to 1000")
    if segment_seconds is not None and not isinstance(segment_seconds, dict):
        raise ValueError("segment_seconds must be a profile-to-durations object")
    if (not isinstance(operating_lift_mm, (float, int)) or
            not math.isfinite(operating_lift_mm) or not 0 <= operating_lift_mm <= 100):
        raise ValueError("operating_lift_mm must be between 0 and 100 mm")
    durations = {**DEFAULT_SEGMENTS, **(segment_seconds or {})}
    setup_seconds = 10 + (6 if operating_lift_mm else 0)
    if expand_repeats and (max(sum(durations[name]) + (2 if name in
                                                 ("f3_front_postures", "f4_wrist_and_holds") else 0)
                               for name in PROFILE_NAMES) * repeats + setup_seconds > 600):
        raise ValueError("expanded plan exceeds the 600 s JSON loader limit")
    scene = _scene_path()
    observer = _observer(scene)
    start = np.asarray(start_joint_rad, dtype=float)
    if start.shape != (6,) or not np.isfinite(start).all():
        raise ValueError("start_joint_rad must contain six finite numbers")
    start_tcp = np.asarray(observer.require_contact_free(start))
    forward = _position_ik(observer, start_tcp + [FORWARD_MM, 0, 0], start)
    forward_tcp = np.asarray(observer.require_contact_free(forward))
    center = (_position_ik(observer, forward_tcp + [0, 0, operating_lift_mm], forward)
              if operating_lift_mm else forward)
    center_tcp = np.asarray(observer.require_contact_free(center))
    output.mkdir(parents=True, exist_ok=True)
    report = {"status": "simulation_only", "scene": str(scene),
              "start_joint_rad": start.tolist(),
              "prepared_tcp_mm": center_tcp.round(3).tolist(),
              "rear_wall_x_m": -.35, "total_allowed_lateral_width_mm": MAX_TOTAL_LATERAL_MM,
              "maximum_designed_forward_span_mm": FRONT_DESIGN_CAP_MM,
              "operating_lift_mm": operating_lift_mm,
              "setup_order": (["forward_100mm", "lift_to_operating_height"]
                              if operating_lift_mm else ["forward_100mm"]),
              "actual_tool_and_obstacles_verified": False,
              "candidate_tool_variants": ["ag145", "mdpg_1s1"],
              "gripper_clearance_audit_file": "a0509_front_gripper_clearance.json",
              "real_current_pose_verified": False,
              "real_100hz_interface_verified": False,
              "real_robot_executable": False,
              "setup": {}, "paths": {}}
    setup_samples = None
    try:
        setup_samples, setup_intervals, setup_jerk = make_setup(start, forward, center)
        if operating_lift_mm:
            setup_intervals[-1]["kind"] = "lift_to_operating_height"
        setup_result, setup_xyz = validate_cycle(
            observer, setup_samples, setup_jerk, 150, start_tcp, closed=False,
            max_x_span_mm=110, max_total_y_mm=MAX_TOTAL_LATERAL_MM,
            max_z_span_mm=operating_lift_mm + 10,
            expected_forward_mm=FORWARD_MM)
        setup_sim = simulate_cycle(observer, setup_samples)
        if not setup_sim["passed"]:
            raise ValueError(f"setup MuJoCo preflight failed: {setup_sim}")
        setup_result.update({"simulation_preflight": setup_sim,
                             "intervals": setup_intervals, "simulation_usable": True,
                             "real_robot_executable": False})
        setup_path = output / "a0509_front_setup_10cm.json"
        setup_path.write_text(json.dumps(_trajectory_json("a0509_front_setup_10cm", setup_samples,
                                                           setup=True), indent=2) + "\n", encoding="utf-8")
        load_trajectory(setup_path)
        setup_result["json"] = setup_path.name
        preview = output / "a0509_front_setup_10cm_preview.png"
        _plot(preview, setup_samples, setup_xyz, setup_intervals, "front_setup_10cm")
        setup_result["preview"] = preview.name
        report["setup"] = setup_result
    except (ValueError, RuntimeError) as error:
        setup_samples = None
        report["setup"] = {"status": "failed", "reason": str(error),
                           "real_robot_executable": False}
        for stale in (output / "a0509_front_setup_10cm.json",
                      output / "a0509_front_setup_10cm_preview.png"):
            if stale.is_file():
                stale.unlink()
    try:
        waypoints = _plan_waypoints(observer, center, center_tcp)
        waypoint_error = None
    except (ValueError, RuntimeError) as error:
        waypoints, waypoint_error = None, str(error)
    for profile in PROFILE_NAMES:
        stem = f"a0509_front_{profile}"
        try:
            if setup_samples is None:
                raise ValueError("forward setup failed")
            if waypoints is None:
                raise ValueError(f"forward-region IK failed: {waypoint_error}")
            samples, intervals, jerk = _sample_cycle(profile, waypoints[profile], durations[profile])
            result, xyz = validate_cycle(
                observer, samples, jerk, 150, center_tcp, max_x_span_mm=FRONT_DESIGN_CAP_MM,
                max_total_y_mm=MAX_TOTAL_LATERAL_MM, max_z_span_mm=40)
            if xyz[:, 0].min() < center_tcp[0] - 2:
                raise ValueError("trajectory moves behind the prepared forward pose")
            sim = simulate_cycle(observer, samples)
            if not sim["passed"]:
                raise ValueError(f"MuJoCo preflight failed: {sim}")
            result.update({"intervals": intervals, "simulation_preflight": sim,
                           "simulation_usable": True, "real_robot_executable": False})
            path = output / f"{stem}.json"
            path.write_text(json.dumps(_trajectory_json(stem, samples, setup=False),
                                       indent=2) + "\n", encoding="utf-8")
            load_trajectory(path)
            result["json"] = path.name
            preview = output / f"{stem}_preview.png"
            _plot(preview, samples, xyz, intervals, profile)
            result["preview"] = preview.name
            if repeats > 1:
                repeated = output / f"{stem}_repeats_preview.png"
                _plot_repeats(repeated, samples, xyz, intervals, profile, repeats)
                result["repeat_preview"] = repeated.name
            if expand_repeats:
                expanded = output / f"{stem}_setup_{repeats}cycles.json"
                expanded.write_text(json.dumps(_trajectory_json(
                    stem + f"_setup_{repeats}cycles",
                    _setup_plus_cycles(setup_samples, samples, repeats), setup=False),
                    indent=2) + "\n", encoding="utf-8")
                load_trajectory(expanded)
                result["expanded_json"] = expanded.name
            report["paths"][profile] = result
        except (ValueError, RuntimeError) as error:
            report["paths"][profile] = {"status": "failed", "reason": str(error),
                                         "real_robot_executable": False}
            for stale in (output / f"{stem}.json", output / f"{stem}_preview.png",
                          output / f"{stem}_repeats_preview.png",
                          output / f"{stem}_setup_{repeats}cycles.json"):
                if stale.is_file():
                    stale.unlink()
    config = {"experiment_id": "a0509_front_sim2real", "sample_hz": HZ,
              "cycle_repeats": repeats,
              "repeat_mode": "one_setup_then_repeat_one_cycle_json",
              "setup_file": report["setup"].get("json"),
              "setup_once_before_cycles": True,
              "start_joint_rad": start.tolist(),
              "start_pose_source": "configured_previous_candidate; not live robot state",
              "operating_lift_mm": operating_lift_mm,
              "lower_obstacle_top_z_mm": None,
              "planning_clearance_mm": 30.0,
              "physical_corridor_y_mm": None,
              "candidate_tool_variants": ["ag145", "mdpg_1s1"],
              "gripper_clearance_audit_file": "a0509_front_gripper_clearance.json",
              "segment_seconds": {name: list(durations[name]) for name in PROFILE_NAMES},
              "total_allowed_lateral_width_mm": MAX_TOTAL_LATERAL_MM,
              "designed_tcp_lateral_band_mm": [-40, 40],
              "f3_tcp_lateral_offset_mm": F3_LATERAL_OFFSET_MM,
              "maximum_designed_forward_span_mm": FRONT_DESIGN_CAP_MM,
              "rear_wall_x_m": -.35, "gripper_type": "none", "payload_mass_kg": 0.0,
              "control_mode": "unverified_100hz_joint_stream",
              "execution_scope": "simulation_only", "real_auto_run": False,
              "cycle_files": [item["json"] for item in report["paths"].values() if "json" in item],
              "comparison_policy": {
                  "tracking_gap": "compare measured q with the same commanded path in simulation and real",
                  "dynamics_residual": "apply measured real q,dq,estimated ddq to matching MuJoCo model; compare only with a characterized real torque signal",
                  "torque_ground_truth_verified": False}}
    (output / "a0509_front_run_config.json").write_text(
        json.dumps(config, indent=2) + "\n", encoding="utf-8")
    (output / "a0509_front_validation.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Generate four simulation-only forward-region cycles")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--repeats", type=int)
    parser.add_argument("--expand-repeats", action="store_true")
    args = parser.parse_args(argv)
    settings = json.loads(args.config.read_text(encoding="utf-8")) if args.config else {}
    output = args.output or (args.config.parent if args.config else Path.cwd() / "a0509_front_outputs")
    report = generate_front_dataset(output,
                                    repeats=(args.repeats if args.repeats is not None else
                                             settings.get("cycle_repeats", 10)),
                                    start_joint_rad=settings.get("start_joint_rad", HOME),
                                    segment_seconds=settings.get("segment_seconds"),
                                    operating_lift_mm=settings.get("operating_lift_mm", DEFAULT_OPERATING_LIFT_MM),
                                    expand_repeats=args.expand_repeats)
    print(json.dumps({name: {key: value for key, value in path.items()
                             if key in ("sample_count", "duration_s", "tcp_span_mm",
                                        "simulation_usable", "reason")}
                      for name, path in report["paths"].items()}, indent=2))
    return 0 if ("json" in report["setup"] and
                 all("json" in path for path in report["paths"].values())) else 1


if __name__ == "__main__":
    raise SystemExit(main())
