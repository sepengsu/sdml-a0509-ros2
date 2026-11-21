"""Collect a compressed, model-only 100 Hz A0509 front-path replay.

This direct MuJoCo collector records simulated joint feedback. It never
connects to ROS or a physical robot. Its effort field is a controller command,
not a measured motor or joint torque.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path

from .front_gripper_audit import (_default_trajectory_dir, _moving_robot_contact_count,
                                  _moving_robot_geoms)
from .front_trajectory import PROFILE_NAMES
from .trajectory import load_trajectory


TORQUE_LIMITS_NM = (194.0, 194.0, 163.0, 50.0, 50.0, 50.0)


def _unique_file(folder: Path, stem: str) -> Path:
    candidate = folder / f"{stem}.jsonl.gz"
    index = 1
    while candidate.exists() or candidate.with_suffix("").with_suffix(".summary.json").exists():
        candidate = folder / f"{stem}_{index:03d}.jsonl.gz"
        index += 1
    return candidate


def collect_front_sim(trajectory_dir: Path, output_dir: Path, *, tool: str,
                      profile: str, repeats: int = 10,
                      stop_requested=None, progress=None) -> Path:
    import mujoco
    import numpy as np

    if tool not in ("ag145", "mdpg_1s1"):
        raise ValueError("tool must be ag145 or mdpg_1s1")
    if profile not in PROFILE_NAMES:
        raise ValueError(f"profile must be one of {PROFILE_NAMES}")
    if not isinstance(repeats, int) or isinstance(repeats, bool) or not 1 <= repeats <= 100:
        raise ValueError("repeats must be an integer from 1 to 100")
    audit_file = trajectory_dir / "a0509_front_gripper_clearance.json"
    audit = json.loads(audit_file.read_text(encoding="utf-8"))
    if audit["status"] != "model_preflight_passed_physical_unverified":
        raise ValueError("model clearance preflight has not passed")
    tool_report = audit["tools"][tool]
    scene = Path(tool_report["scene"])
    if not scene.is_file():
        raise FileNotFoundError("tool scene is unavailable; run audit_front_grippers on this machine")
    for name in ("setup_10cm", profile):
        if not tool_report["paths"][name]["tool_mass_simulation_preflight"]["passed"]:
            raise ValueError(f"{name} tool-mass preflight failed")
    setup_file = trajectory_dir / "a0509_front_setup_10cm.json"
    cycle_file = trajectory_dir / f"a0509_front_{profile}.json"
    setup, cycle = load_trajectory(setup_file), load_trajectory(cycle_file)
    if any(abs(a-b) > 1e-8 for a, b in zip(setup.points[-1].q_rad,
                                            cycle.points[0].q_rad)):
        raise ValueError("setup does not join the cycle")

    model = mujoco.MjModel.from_xml_path(str(scene))
    moving_geoms = _moving_robot_geoms(model)
    if abs(model.opt.timestep - .002) > 1e-9:
        raise ValueError("the scene must use a 2 ms physics step")
    data, target = mujoco.MjData(model), mujoco.MjData(model)
    qaddr, daddr = [], []
    for joint in range(1, 7):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{joint}")
        if jid < 0:
            raise ValueError(f"scene missing joint_{joint}")
        qaddr.append(int(model.jnt_qposadr[jid]))
        daddr.append(int(model.jnt_dofadr[jid]))
    data.qpos[:] = model.qpos0
    for index, address in enumerate(qaddr):
        data.qpos[address] = setup.points[0].q_rad[index]
    mujoco.mj_forward(model, data)
    if _moving_robot_contact_count(data, moving_geoms):
        raise ValueError("initial simulation pose has robot model contact")

    output_dir.mkdir(parents=True, exist_ok=True)
    stem = f"a0509_front_{profile}_{tool}_{repeats}cycles"
    samples_file = _unique_file(output_dir, stem)
    experiment_id = samples_file.name.removesuffix(".jsonl.gz")
    summary_file = samples_file.with_suffix("").with_suffix(".summary.json")
    summary = {
        "experiment_id": experiment_id,
        "mode": "simulation",
        "execution_scope": "simulation_only",
        "tool_model": tool,
        "physical_tool_calibrated": False,
        "torque_signal_kind": "simulated controller command; not a measured torque",
        "setup_trajectory": str(setup_file.resolve()),
        "cycle_trajectory": str(cycle_file.resolve()),
        "setup_sha256": hashlib.sha256(setup_file.read_bytes()).hexdigest(),
        "cycle_sha256": hashlib.sha256(cycle_file.read_bytes()).hexdigest(),
        "scene": str(scene.resolve()),
        "clearance_report": str(audit_file.resolve()),
        "whole_campaign_lateral_span_mm": tool_report["whole_campaign_lateral_span_mm"],
        "whole_campaign_lateral_span_passed": tool_report["whole_campaign_lateral_span_passed"],
        "sample_hz": 100,
        "physics_timestep_s": .002,
        "repeats": repeats,
        "planned_rows": len(setup.points) + repeats * (len(cycle.points)-1),
        "written_rows": 0,
        "maximum_tracking_error_rad": 0.0,
        "torque_saturation_steps": 0,
        "contact_steps": 0,
        "background_contact_steps": 0,
        "samples_file": str(samples_file.resolve()),
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "result": "running",
    }

    def row(point, *, phase: str, repeat: int, sample_index: int, torque_nm) -> dict:
        q = [float(data.qpos[address]) for address in qaddr]
        dq = [float(data.qvel[address]) for address in daddr]
        tracking = max(abs(q[j] - point.q_rad[j]) for j in range(6))
        summary["maximum_tracking_error_rad"] = max(summary["maximum_tracking_error_rad"], tracking)
        return {
            "experiment_id": experiment_id,
            "mode": "simulation", "tool_model": tool,
            "profile": profile, "phase": phase, "repeat_no": repeat,
            "sample_index": sample_index,
            "time_s": round(sample_index / 100, 8),
            "target_q_rad": list(point.q_rad),
            "target_dq_rad_s": list(point.dq_rad_s),
            "target_ddq_rad_s2": list(point.ddq_rad_s2),
            "sim_q_rad": q,
            "sim_dq_rad_s": dq,
            "sim_command_torque_nm": list(torque_nm),
            "tracking_error_max_rad": tracking,
        }

    def step(point):
        target.qpos[:] = data.qpos
        target.qvel[:] = 0
        target.qacc[:] = 0
        for joint, (qa, da) in enumerate(zip(qaddr, daddr)):
            target.qpos[qa] = point.q_rad[joint]
            target.qvel[da] = point.dq_rad_s[joint]
            target.qacc[da] = point.ddq_rad_s2[joint]
        mujoco.mj_forward(model, target)
        for joint, address in enumerate(daddr):
            target.qacc[address] = point.ddq_rad_s2[joint]
        mujoco.mj_inverse(model, target)
        applied = np.zeros(6, dtype=float)
        for _ in range(5):
            data.ctrl[:] = point.q_rad
            mujoco.mj_forward(model, data)
            for joint, (qa, da) in enumerate(zip(qaddr, daddr)):
                command = (target.qfrc_inverse[da] +
                           150 * (point.q_rad[joint] - data.qpos[qa]) +
                           20 * (point.dq_rad_s[joint] - data.qvel[da]))
                if abs(command) > TORQUE_LIMITS_NM[joint]:
                    summary["torque_saturation_steps"] += 1
                applied[joint] = np.clip(command, -TORQUE_LIMITS_NM[joint],
                                         TORQUE_LIMITS_NM[joint])
                data.qfrc_applied[da] = applied[joint] - data.qfrc_actuator[da]
            mujoco.mj_step(model, data)
            if _moving_robot_contact_count(data, moving_geoms):
                summary["contact_steps"] += 1
                raise RuntimeError("MuJoCo reported model contact during collection")
            if data.ncon:
                summary["background_contact_steps"] += 1
        return applied.tolist()

    try:
        with gzip.open(samples_file, "wt", encoding="utf-8", compresslevel=6) as stream:
            def write(item):
                stream.write(json.dumps(item, separators=(",", ":"), allow_nan=False) + "\n")
                summary["written_rows"] += 1
                if progress is not None and (summary["written_rows"] % 500 == 0 or
                                             summary["written_rows"] == summary["planned_rows"]):
                    progress(summary["written_rows"], summary["planned_rows"])

            write(row(setup.points[0], phase="setup", repeat=0, sample_index=0,
                      torque_nm=[0.0] * 6))
            sample_index = 0
            for point in setup.points[1:]:
                if stop_requested is not None and stop_requested():
                    raise RuntimeError("simulation collection stopped by user")
                applied = step(point)
                sample_index += 1
                write(row(point, phase="setup", repeat=0,
                          sample_index=sample_index, torque_nm=applied))
            for repeat in range(1, repeats + 1):
                for point in cycle.points[1:]:
                    if stop_requested is not None and stop_requested():
                        raise RuntimeError("simulation collection stopped by user")
                    applied = step(point)
                    sample_index += 1
                    write(row(point, phase="cycle", repeat=repeat,
                              sample_index=sample_index, torque_nm=applied))
        if (summary["written_rows"] != summary["planned_rows"] or
                summary["torque_saturation_steps"] or
                summary["maximum_tracking_error_rad"] > .05):
            raise RuntimeError("collection failed row count, torque, or tracking check")
        summary["result"] = "completed"
    except Exception as error:
        summary["result"] = "aborted"
        summary["reason"] = str(error)
        raise
    finally:
        summary["ended_utc"] = datetime.now(timezone.utc).isoformat()
        summary_file.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n",
                                encoding="utf-8")
    return summary_file


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Collect compressed MuJoCo-only front data")
    parser.add_argument("--trajectory-dir", type=Path, default=_default_trajectory_dir())
    parser.add_argument("--output-dir", type=Path, default=Path.cwd() / "results/front_sim_data")
    parser.add_argument("--tool", choices=("ag145", "mdpg_1s1"), required=True)
    parser.add_argument("--profile", choices=PROFILE_NAMES, required=True)
    parser.add_argument("--repeats", type=int, default=10)
    args = parser.parse_args(argv)
    path = collect_front_sim(args.trajectory_dir, args.output_dir,
                             tool=args.tool, profile=args.profile, repeats=args.repeats)
    print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
