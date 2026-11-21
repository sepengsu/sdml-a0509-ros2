"""Quantify planned joint excitation and MuJoCo torque-term balance.

This diagnostic uses commanded targets and approximate tool models. It does
not estimate real torque noise or establish dynamic parameter identifiability.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .front_gripper_audit import _default_trajectory_dir
from .front_trajectory import PROFILE_NAMES
from .trajectory import load_trajectory


def audit_front_quality(trajectory_dir: Path, clearance_report: Path) -> dict:
    import numpy as np
    from a0509_dynamics_experiment.simulation_executor import MujocoObserver

    clearance = json.loads(clearance_report.read_text(encoding="utf-8"))
    plans = {name: load_trajectory(trajectory_dir / f"a0509_front_{name}.json")
             for name in PROFILE_NAMES}
    report = {
        "status": "planned_target_diagnostic_only",
        "sample_hz": 100,
        "source": "100 Hz commanded joint trajectories; no measured robot samples",
        "f1_f2_same_geometric_joint_path": True,
        "torque_term_method": ("MuJoCo mass matrix times target ddq; separately report joint "
                               "damping coefficient times target dq"),
        "real_torque_noise_measured": False,
        "physical_tool_dynamics_calibrated": False,
        "full_six_axis_dynamics_identification_validated": False,
        "paths": {}, "models": {},
    }
    for name, plan in plans.items():
        q = np.asarray([point.q_rad for point in plan.points])
        dq = np.asarray([point.dq_rad_s for point in plan.points])
        ddq = np.asarray([point.ddq_rad_s2 for point in plan.points])
        report["paths"][name] = {
            "duration_s": plan.duration_s,
            "sample_count": len(plan.points),
            "joint_position_span_rad": np.ptp(q, axis=0).round(6).tolist(),
            "joint_velocity_peak_rad_s": np.max(np.abs(dq), axis=0).round(6).tolist(),
            "joint_velocity_rms_rad_s": np.sqrt(np.mean(dq * dq, axis=0)).round(6).tolist(),
            "joint_acceleration_peak_rad_s2": np.max(np.abs(ddq), axis=0).round(6).tolist(),
            "joint_acceleration_rms_rad_s2": np.sqrt(np.mean(ddq * ddq, axis=0)).round(6).tolist(),
            "moving_sample_fraction": np.mean(np.abs(dq) > 1e-4, axis=0).round(4).tolist(),
        }
    for tool, item in clearance["tools"].items():
        scene = Path(item["scene"])
        observer = MujocoObserver(scene, tcp_site=("ag145_tcp" if tool == "ag145" else "mdpg_tcp"))
        model = observer.model
        dofs = observer.dof_addr
        model_report = {
            "scene": str(scene),
            "joint_damping_nm_s_rad": model.dof_damping[dofs].round(4).tolist(),
            "joint_frictionloss_nm": model.dof_frictionloss[dofs].round(4).tolist(),
            "tool_model_calibrated": False,
            "paths": {},
        }
        mass = np.zeros((model.nv, model.nv), dtype=float)
        acceleration = np.zeros(model.nv, dtype=float)
        for name, plan in plans.items():
            inertia_square = np.zeros(6, dtype=float)
            damping_square = np.zeros(6, dtype=float)
            inertia_peak = np.zeros(6, dtype=float)
            damping_peak = np.zeros(6, dtype=float)
            for point in plan.points:
                observer._set_state(point.q_rad, point.dq_rad_s, point.ddq_rad_s2)
                observer.mj.mj_fullM(model, observer.data, mass)
                acceleration[:] = 0
                acceleration[dofs] = point.ddq_rad_s2
                inertia = (mass @ acceleration)[dofs]
                damping = model.dof_damping[dofs] * np.asarray(point.dq_rad_s)
                inertia_square += inertia * inertia
                damping_square += damping * damping
                inertia_peak = np.maximum(inertia_peak, np.abs(inertia))
                damping_peak = np.maximum(damping_peak, np.abs(damping))
            count = len(plan.points)
            model_report["paths"][name] = {
                "model_inertia_torque_rms_nm": np.sqrt(inertia_square / count).round(6).tolist(),
                "model_inertia_torque_peak_nm": inertia_peak.round(6).tolist(),
                "model_damping_torque_rms_nm": np.sqrt(damping_square / count).round(6).tolist(),
                "model_damping_torque_peak_nm": damping_peak.round(6).tolist(),
            }
        report["models"][tool] = model_report
    return report


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Audit four planned front paths for excitation")
    parser.add_argument("--trajectory-dir", type=Path, default=_default_trajectory_dir())
    parser.add_argument("--clearance-report", type=Path)
    args = parser.parse_args(argv)
    clearance_report = (args.clearance_report or
                        args.trajectory_dir / "a0509_front_gripper_clearance.json")
    report = audit_front_quality(args.trajectory_dir, clearance_report)
    output = args.trajectory_dir / "a0509_front_quality_audit.json"
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({name: data["joint_position_span_rad"]
                      for name, data in report["paths"].items()}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
