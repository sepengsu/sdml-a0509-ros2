"""Build two forward/down right-angle steps with J1 fixed and a rear wall."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

from build_forward_wall_trajectory import ROOT, WALL_FRONT_X_M, WALL_X_M, tcp, wall_model

sys.path.insert(0, str(ROOT / "src/sim_real_gap_getting"))
from sim_real_gap_getting.home_pose import HOME_JOINT_RAD  # noqa: E402
from sim_real_gap_getting.trajectory import load_trajectory, resample_uniform  # noqa: E402


LEG_M = 0.20
LEG_S = 8.0
HOLD_S = 1.0
REPEATS = 2
OFFSETS = ((0.0, 0.0), (LEG_M, 0.0), (LEG_M, -LEG_M),
           (2*LEG_M, -LEG_M), (2*LEG_M, -2*LEG_M))
DURATION_S = 2*HOLD_S + 2*REPEATS*LEG_S


def desired_xz(time_s: float, start_xz: np.ndarray) -> np.ndarray:
    if time_s <= HOLD_S:
        return start_xz.copy()
    if time_s >= DURATION_S - HOLD_S:
        return start_xz + OFFSETS[-1]
    segment = min(int((time_s - HOLD_S) // LEG_S), len(OFFSETS) - 2)
    u = (time_s - HOLD_S - segment*LEG_S) / LEG_S
    blend = 10*u**3 - 15*u**4 + 6*u**5
    first = np.asarray(OFFSETS[segment])
    last = np.asarray(OFFSETS[segment + 1])
    return start_xz + first + blend*(last - first)


def build(output: Path) -> dict:
    model = wall_model()
    data = mujoco.MjData(model)
    home = np.asarray(HOME_JOINT_RAD, dtype=float)
    current = home.copy()
    data.qpos[:] = home
    mujoco.mj_forward(model, data)
    if data.ncon:
        raise RuntimeError("Home has a contact")
    start_tcp = tcp(model, data)
    start_xz = start_tcp[[0, 2]]
    jac_pos = np.zeros((3, model.nv))
    jac_rot = np.zeros((3, model.nv))
    count = round(DURATION_S * 100) + 1
    q_values = np.zeros((count, 6))
    tcp_values = np.zeros((count, 3))
    for index in range(count):
        time_s = index / 100
        target = desired_xz(time_s, start_xz)
        for _ in range(40):
            data.qpos[:] = current
            mujoco.mj_forward(model, data)
            error = target - tcp(model, data)[[0, 2]]
            if np.linalg.norm(error) < 1e-8:
                break
            mujoco.mj_jacBody(model, data, jac_pos, jac_rot, model.body("link_6").id)
            jac_xz = jac_pos[[0, 2]][:, [1, 2]]
            current[1:3] += np.clip(np.linalg.solve(jac_xz, error), -0.06, 0.06)
        else:
            raise RuntimeError(f"Cartesian IK failed at {time_s:.2f}s, target X/Z={target}")
        data.qpos[:] = current
        mujoco.mj_forward(model, data)
        if data.ncon:
            raise RuntimeError(f"MuJoCo contact at {time_s:.2f}s")
        q_values[index] = current
        tcp_values[index] = tcp(model, data)
    dq_values = np.gradient(q_values, 0.01, axis=0, edge_order=2)
    ddq_values = np.gradient(dq_values, 0.01, axis=0, edge_order=2)
    corner_indices = [0] + [round((HOLD_S + leg*LEG_S)*100)
                            for leg in range(2*REPEATS + 1)] + [count - 1]
    for index in corner_indices:
        dq_values[index] = 0
        ddq_values[index] = 0
    points = [
        {"time_s": index / 100, "q_rad": q_values[index].tolist(),
         "dq_rad_s": dq_values[index].tolist(), "ddq_rad_s2": ddq_values[index].tolist()}
        for index in range(count)
    ]
    payload = {
        "name": "right_angle_forward_down_20cm_twice",
        "joint_names": [f"joint_{i}" for i in range(1, 7)],
        "units": "rad",
        "description": "Two staircase repetitions: +X 200 mm, -Z 200 mm. J1 stays at 0 deg; stop at each corner.",
        "rear_wall_x_m": WALL_X_M,
        "forward_axis": "DR_BASE +X",
        "gripper_type": "none",
        "points": points,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    trajectory = load_trajectory(output)
    samples = resample_uniform(trajectory)
    if len(samples) != count:
        raise RuntimeError("100 Hz sample count changed")
    if np.max(np.abs(q_values[:, 0])) > 1e-12:
        raise RuntimeError("J1 must remain fixed")
    boundaries = [round((HOLD_S + leg*LEG_S)*100) for leg in range(2*REPEATS + 1)]
    for leg, (start, finish) in enumerate(zip(boundaries[:-1], boundaries[1:])):
        axis = 0 if leg % 2 == 0 else 2
        direction = 1 if axis == 0 else -1
        if np.min(direction * np.diff(tcp_values[start:finish + 1, axis])) < -1e-7:
            raise RuntimeError(f"leg {start}..{finish} reversed direction")
        stationary_axis = 2 if axis == 0 else 0
        if np.max(np.abs(tcp_values[start:finish + 1, stationary_axis] - tcp_values[start, stationary_axis])) > 1e-4:
            raise RuntimeError(f"leg {start}..{finish} left its right-angle line")
    report = {
        "trajectory_json": str(output),
        "repeat_count": REPEATS,
        "leg_distance_mm": LEG_M * 1000,
        "leg_duration_s": LEG_S,
        "duration_s": DURATION_S,
        "wall_x_m": WALL_X_M,
        "joint_start_deg": np.degrees(q_values[0]).tolist(),
        "joint_end_deg": np.degrees(q_values[-1]).tolist(),
        "tcp_start_m": tcp_values[0].tolist(),
        "tcp_end_m": tcp_values[-1].tolist(),
        "forward_total_mm": float((tcp_values[-1, 0] - tcp_values[0, 0])*1000),
        "down_total_mm": float((tcp_values[0, 2] - tcp_values[-1, 2])*1000),
        "maximum_lateral_drift_mm": float(np.max(np.abs(tcp_values[:, 1] - tcp_values[0, 1]))*1000),
        "minimum_tcp_wall_clearance_mm": float(np.min(tcp_values[:, 0] - WALL_FRONT_X_M)*1000),
        "checked_samples": len(samples),
        "contact_samples": 0,
    }
    report_path = ROOT / "results/right_angle_wall/plan_report.json"
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "src/sim_real_gap_getting/examples/right_angle_20cm_twice.json")
    args = parser.parse_args()
    print(json.dumps(build(args.output.resolve()), indent=2))


if __name__ == "__main__":
    main()
