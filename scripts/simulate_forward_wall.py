"""Run the forward-wall JSON through MuJoCo dynamics and save measured states."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

from build_forward_wall_trajectory import ROOT, tcp, wall_model

sys.path.insert(0, str(ROOT / "src/sim_real_gap_getting"))
from sim_real_gap_getting.trajectory import load_trajectory, resample_uniform  # noqa: E402


def simulate(trajectory_path: Path, output_dir: Path) -> dict:
    trajectory = load_trajectory(trajectory_path)
    samples = resample_uniform(trajectory)
    model = wall_model()
    step_count = round(0.01 / model.opt.timestep)
    if abs(step_count * model.opt.timestep - 0.01) > 1e-9:
        raise RuntimeError("MuJoCo timestep must divide 10 ms")
    data = mujoco.MjData(model)
    desired = mujoco.MjData(model)
    data.qpos[:] = samples[0].q_rad
    data.ctrl[:] = data.qpos
    mujoco.mj_forward(model, data)
    if data.ncon:
        raise RuntimeError("initial robot pose contacts the rear wall")
    force_limits = np.array([194.0, 194.0, 163.0, 50.0, 50.0, 50.0])
    kp = np.array([500.0, 4000.0, 4000.0, 500.0, 1500.0, 1500.0])
    kd = np.array([60.0, 160.0, 160.0, 60.0, 100.0, 100.0])
    positions = np.zeros((len(samples), 6))
    velocities = np.zeros_like(positions)
    tcp_positions = np.zeros((len(samples), 3))
    maximum_torque = np.zeros(6)
    maximum_tracking_error = np.zeros(6)
    saturated_substeps = 0
    mass_acceleration = np.zeros(model.nv)
    for index, point in enumerate(samples):
        q_target = np.asarray(point.q_rad)
        dq_target = np.asarray(point.dq_rad_s)
        desired.qpos[:] = q_target
        desired.qvel[:] = dq_target
        desired.qacc[:] = point.ddq_rad_s2
        mujoco.mj_forward(model, desired)
        mujoco.mj_mulM(model, desired, mass_acceleration, desired.qacc)
        for _ in range(step_count):
            data.ctrl[:] = q_target
            mujoco.mj_forward(model, data)
            torque = (desired.qfrc_bias + mass_acceleration - desired.qfrc_passive +
                      kp * (q_target - data.qpos) + kd * (dq_target - data.qvel))
            maximum_torque = np.maximum(maximum_torque, np.abs(torque))
            if np.any(np.abs(torque) > force_limits):
                saturated_substeps += 1
            torque = np.clip(torque, -force_limits, force_limits)
            data.qfrc_applied[:] = torque - data.qfrc_actuator
            mujoco.mj_step(model, data)
            if data.ncon:
                raise RuntimeError(f"MuJoCo contact at sample {index}")
        mujoco.mj_forward(model, data)
        positions[index] = data.qpos
        velocities[index] = data.qvel
        tcp_positions[index] = tcp(model, data)
        maximum_tracking_error = np.maximum(maximum_tracking_error, np.abs(q_target - data.qpos))
    if np.max(np.abs(positions[:, 0])) > 1e-4:
        raise RuntimeError(f"J1 moved in dynamic simulation: {np.degrees(np.max(np.abs(positions[:, 0]))):.6f} deg")
    output_dir.mkdir(parents=True, exist_ok=True)
    state_path = output_dir / "forward_wall_simulation.npz"
    np.savez_compressed(state_path,
                        time_s=np.array([point.time_s for point in samples]),
                        q_rad=positions, dq_rad_s=velocities, tcp_m=tcp_positions)
    start = tcp_positions[0]
    end = tcp_positions[-1]
    report = {
        "kind": "MuJoCo dynamic simulation",
        "trajectory_json": str(trajectory_path),
        "state_file": str(state_path),
        "samples": len(samples),
        "physics_steps_per_sample": step_count,
        "j1_max_abs_deg": float(np.max(np.abs(np.degrees(positions[:, 0])))),
        "tcp_forward_mm": float((end[0] - start[0]) * 1000),
        "tcp_min_backward_step_mm": float(np.min(np.diff(tcp_positions[:, 0])) * 1000),
        "tcp_max_lateral_drift_mm": float(np.max(np.abs(tcp_positions[:, 1] - start[1])) * 1000),
        "tcp_max_height_drift_mm": float(np.max(np.abs(tcp_positions[:, 2] - start[2])) * 1000),
        "max_joint_tracking_error_deg": np.degrees(maximum_tracking_error).tolist(),
        "max_command_torque_nm": maximum_torque.tolist(),
        "torque_saturated_substeps": saturated_substeps,
        "wall_contact_steps": 0,
    }
    (output_dir / "dynamic_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", type=Path,
                        default=ROOT / "src/sim_real_gap_getting/examples/forward_150mm_wall.json")
    parser.add_argument("--output-dir", type=Path, default=ROOT / "results/forward_wall")
    args = parser.parse_args()
    print(json.dumps(simulate(args.trajectory.resolve(), args.output_dir.resolve()), indent=2))


if __name__ == "__main__":
    main()
