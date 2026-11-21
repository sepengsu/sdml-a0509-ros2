"""Plan a wall-aware A0509 forward-only practice trajectory with J1 fixed."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/sim_real_gap_getting"))
from sim_real_gap_getting.home_pose import HOME_JOINT_RAD  # noqa: E402
from sim_real_gap_getting.trajectory import load_trajectory, resample_uniform  # noqa: E402

SCENE = ROOT / "src/doosan-robot2/dsr_description2/mujoco_models/a0509/scene.xml"
WALL_SCENE = SCENE.with_name("scene_forward_wall.xml")
WALL_X_M = -0.35
WALL_FRONT_X_M = WALL_X_M + 0.02
FORWARD_DISTANCE_M = 0.15
DURATION_S = 6.0


def wall_model() -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(WALL_SCENE))


def tcp(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    return data.xpos[model.body("link_6").id].copy()


def solve_forward_end(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    data = mujoco.MjData(model)
    start = np.asarray(HOME_JOINT_RAD, dtype=float)
    data.qpos[:] = start
    mujoco.mj_forward(model, data)
    start_tcp = tcp(model, data)
    if data.ncon:
        raise RuntimeError("Home begins in contact with the wall or floor")
    if start_tcp[0] <= 0:
        raise RuntimeError("Home flange does not face +X in the base frame")
    target_tcp = start_tcp.copy()
    target_tcp[0] += FORWARD_DISTANCE_M
    target = start.copy()
    jac_pos = np.zeros((3, model.nv))
    jac_rot = np.zeros((3, model.nv))
    body_id = model.body("link_6").id
    for _ in range(100):
        data.qpos[:] = target
        mujoco.mj_forward(model, data)
        error = target_tcp[[0, 2]] - tcp(model, data)[[0, 2]]
        if np.linalg.norm(error) < 1e-7:
            break
        mujoco.mj_jacBody(model, data, jac_pos, jac_rot, body_id)
        step = np.linalg.solve(jac_pos[[0, 2]][:, [1, 2]], error)
        target[1:3] += np.clip(step, -0.05, 0.05)
    else:
        raise RuntimeError("J2/J3 could not reach the forward TCP target")
    data.qpos[:] = target
    mujoco.mj_forward(model, data)
    if data.ncon or np.linalg.norm(tcp(model, data)[[0, 2]] - target_tcp[[0, 2]]) > 1e-4:
        raise RuntimeError("forward endpoint is in contact or misses target")
    return start, target, start_tcp


def build(output: Path) -> dict:
    model = wall_model()
    start, end, start_tcp = solve_forward_end(model)
    data = mujoco.MjData(model)
    jac_pos = np.zeros((3, model.nv))
    jac_rot = np.zeros((3, model.nv))
    q_values = np.zeros((round(DURATION_S * 100) + 1, 6))
    current = start.copy()
    for index in range(len(q_values)):
        t = index / 100
        u = np.clip((t - 1.0) / 4.0, 0.0, 1.0)
        blend = 10*u**3 - 15*u**4 + 6*u**5
        target_x = start_tcp[0] + FORWARD_DISTANCE_M * blend
        for _ in range(30):
            data.qpos[:] = current
            mujoco.mj_forward(model, data)
            error = np.array((target_x - tcp(model, data)[0], start_tcp[2] - tcp(model, data)[2]))
            if np.linalg.norm(error) < 1e-8:
                break
            mujoco.mj_jacBody(model, data, jac_pos, jac_rot, model.body("link_6").id)
            current[1:3] += np.linalg.solve(jac_pos[[0, 2]][:, [1, 2]], error)
        else:
            raise RuntimeError(f"Cartesian forward IK failed at {t:.2f}s")
        q_values[index] = current
    if np.max(np.abs(q_values[-1] - end)) > 1e-5:
        raise RuntimeError("dense path endpoint differs from the verified target")
    dq_values = np.gradient(q_values, 0.01, axis=0, edge_order=2)
    ddq_values = np.gradient(dq_values, 0.01, axis=0, edge_order=2)
    for index in (0, 100, 500, 600):
        dq_values[index] = 0
        ddq_values[index] = 0
    points = [
        {"time_s": index / 100, "q_rad": q_values[index].tolist(),
         "dq_rad_s": dq_values[index].tolist(), "ddq_rad_s2": ddq_values[index].tolist()}
        for index in range(len(q_values))
    ]
    payload = {
        "name": "forward_150mm_j1_fixed_rear_wall",
        "joint_names": [f"joint_{i}" for i in range(1, 7)],
        "units": "rad",
        "description": "Move flange 150 mm forward along DR_BASE +X, with J1 fixed at 0 deg; rear wall at X=-350 mm.",
        "rear_wall_x_m": WALL_X_M,
        "forward_axis": "DR_BASE +X",
        "gripper_type": "none",
        "points": points,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    trajectory = load_trajectory(output)
    tcp_positions = []
    samples = resample_uniform(trajectory)
    for index, sample in enumerate(samples):
        data.qpos[:] = sample.q_rad
        data.qvel[:] = sample.dq_rad_s
        mujoco.mj_forward(model, data)
        if abs(sample.q_rad[0]) > 1e-12:
            raise RuntimeError(f"J1 moved at sample {index}")
        if data.ncon:
            raise RuntimeError(f"MuJoCo contact at sample {index}: {data.ncon} contact(s)")
        tcp_positions.append(tcp(model, data))
    tcp_positions = np.asarray(tcp_positions)
    progress = tcp_positions[:, 0] - start_tcp[0]
    if np.min(np.diff(progress)) < -1e-6:
        raise RuntimeError("TCP reversed toward the wall")
    if abs(progress[-1] - FORWARD_DISTANCE_M) > 1e-4:
        raise RuntimeError("TCP did not finish 150 mm forward")
    report = {
        "trajectory_json": str(output),
        "model": str(WALL_SCENE),
        "wall_x_m": WALL_X_M,
        "wall_front_x_m": WALL_FRONT_X_M,
        "start_joint_deg": np.degrees(start).tolist(),
        "end_joint_deg": np.degrees(end).tolist(),
        "tcp_start_m": start_tcp.tolist(),
        "tcp_end_m": tcp_positions[-1].tolist(),
        "forward_distance_mm": float(progress[-1] * 1000),
        "minimum_forward_step_mm": float(np.min(np.diff(progress)) * 1000),
        "maximum_lateral_deviation_mm": float(np.max(np.abs(tcp_positions[:, 1] - start_tcp[1])) * 1000),
        "maximum_height_deviation_mm": float(np.max(np.abs(tcp_positions[:, 2] - start_tcp[2])) * 1000),
        "minimum_tcp_wall_clearance_mm": float(np.min(tcp_positions[:, 0] - WALL_FRONT_X_M) * 1000),
        "checked_samples": len(samples),
        "wall_contact_samples": 0,
        "simulation_kind": "MuJoCo kinematic replay with contact checks; no actuator tracking validation",
    }
    report_path = ROOT / "results/forward_wall/plan_report.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "src/sim_real_gap_getting/examples/forward_150mm_wall.json")
    args = parser.parse_args()
    print(json.dumps(build(args.output.resolve()), indent=2))


if __name__ == "__main__":
    main()
