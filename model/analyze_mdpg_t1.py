"""Analyze the recorded arm-only T1 trial used for the mDPG visual replay.

The tool appears only in the video. Torques here belong to the bare A0509
MuJoCo model and must not be interpreted as mDPG/pneumatic measurements.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
JOINTS = tuple(f"J{i}" for i in range(1, 7))
COLORS = plt.get_cmap("tab10").colors[:6]


def _six(row: dict, prefix: str) -> np.ndarray:
    return np.asarray([float(row[f"{prefix}{i}"]) for i in range(1, 7)])


def _rms(values: np.ndarray, axis=0) -> np.ndarray:
    return np.sqrt(np.mean(np.square(values), axis=axis))


def analyze(source: str | Path, video_metadata: str | Path, output: str | Path) -> dict:
    source, video_metadata, output = map(lambda value: Path(value).resolve(),
                                         (source, video_metadata, output))
    output.mkdir(parents=True, exist_ok=True)
    trial = json.loads((source / "metadata.json").read_text(encoding="utf-8"))
    visual = json.loads(video_metadata.read_text(encoding="utf-8"))
    dynamics_file = source / "dynamics.csv"
    digest = hashlib.sha256(dynamics_file.read_bytes()).hexdigest()
    if (trial.get("trajectory_id") != "T1" or trial.get("mode") != "simulation"
            or trial.get("gripper_type") != "none" or not trial.get("execution_success")
            or visual.get("mode") != "visual_replay" or visual.get("gripper_type") != "mdpg_1s1"
            or visual.get("source_dynamics_sha256") != digest):
        raise ValueError("T1 source and mDPG visual replay provenance do not match")
    with dynamics_file.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    with (source / "trajectory.csv").open(newline="", encoding="utf-8") as stream:
        planned = list(csv.DictReader(stream))
    if len(rows) < 2 or len(rows) != len(planned):
        raise ValueError("T1 observation and plan row counts differ")
    t = np.asarray([float(row["ros_time_s"]) for row in rows])
    t -= t[0]
    dt = np.diff(t)
    if not np.isfinite(t).all() or np.any(dt <= 0):
        raise ValueError("ROS simulation time is not finite and strictly increasing")
    q, dq, ddq, target = (np.stack([_six(row, stem) for row in rows])
                          for stem in ("q", "dq", "ddq", "target_q"))
    inverse = np.stack([np.asarray(json.loads(row["qfrc_inverse"]), dtype=float) for row in rows])
    bias = np.stack([np.asarray(json.loads(row["qfrc_bias"]), dtype=float) for row in rows])
    mass = np.stack([np.asarray(json.loads(row["mass_matrix"]), dtype=float) for row in rows])
    damping_coefficient = np.stack([
        np.asarray(json.loads(row["dof_damping"]), dtype=float) for row in rows])
    measured_tcp = np.stack([np.asarray(json.loads(row["tcp_position"]), dtype=float)
                             for row in rows])
    planned_tcp = np.asarray([[float(row[f"{axis}_mm"]) for axis in "xyz"]
                              for row in planned])
    plan_time = np.asarray([float(row["time_s"]) for row in planned])
    arrays = (q, dq, ddq, target, inverse, bias, mass, damping_coefficient,
              measured_tcp, planned_tcp, plan_time)
    if not all(np.isfinite(array).all() for array in arrays):
        raise ValueError("nonfinite required T1 values")
    if any(array.shape != (len(rows), 6) for array in (q, dq, ddq, target, inverse, bias)):
        raise ValueError("joint array has the wrong shape")
    if mass.shape != (len(rows), 6, 6) or np.max(np.abs(plan_time - t)) > .1:
        raise ValueError("mass matrix or plan/observation time mismatch")

    arm_scene = ROOT / "src/doosan-robot2/dsr_description2/mujoco_models/a0509/scene.xml"
    model = mujoco.MjModel.from_xml_path(str(arm_scene))
    if model.nq != 6 or model.nv != 6:
        raise ValueError("expected six-axis A0509 arm-only model")
    data = mujoco.MjData(model)
    gravity = np.empty_like(q)
    contacts = np.empty(len(rows), dtype=int)
    for index, position in enumerate(q):
        data.qpos[:] = position
        data.qvel[:] = 0
        mujoco.mj_forward(model, data)
        gravity[index] = data.qfrc_bias[:6]
        contacts[index] = data.ncon
    inertia = np.einsum("nij,nj->ni", mass, ddq)
    velocity_bias = bias - gravity
    damping = damping_coefficient * dq
    reconstruction = inertia + gravity + velocity_bias + damping
    residual = inverse - reconstruction
    tracking = q - target
    tcp_tracking = measured_tcp - planned_tcp
    missing = {stem: sum(not row[f"{stem}{j}"] for row in rows for j in range(1, 7))
               for stem in ("q", "dq", "ddq", "target_q")}
    missing.update({field: sum(not row[field] for row in rows)
                    for field in ("tcp_position", "qfrc_inverse", "qfrc_actuator")})
    applied_unavailable = sum(not row["qfrc_applied"] or
                              not np.isfinite(np.asarray(json.loads(row["qfrc_applied"]), dtype=float)).any()
                              for row in rows)
    # These screens catch gross recording glitches; they are not certified limits.
    gross = {"position_jump_gt_0p05_rad": int(np.sum(np.any(np.abs(np.diff(q, axis=0)) > .05, axis=1))),
             "velocity_jump_gt_0p5_rad_s": int(np.sum(np.any(np.abs(np.diff(dq, axis=0)) > .5, axis=1))),
             "abs_acceleration_gt_10_rad_s2": int(np.sum(np.any(np.abs(ddq) > 10, axis=1))),
             "abs_inverse_torque_gt_200_nm": int(np.sum(np.any(np.abs(inverse) > 200, axis=1)))}
    median_dt = float(np.median(dt))
    missing_gap_count = int(np.sum(dt > 1.5 * median_dt))
    summary = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "trajectory_id": "T1", "source_trial": str(source),
        "source_trial_relative_to_repo": (str(source.relative_to(ROOT)).replace("\\", "/")
                                          if source.is_relative_to(ROOT) else None),
        "source_ros_domain_id": trial["ros_domain_id"],
        "analysis_mujoco_version": mujoco.__version__,
        "source_dynamics_sha256": digest,
        "visual_video_sha256": visual["video"]["sha256"],
        "scope": "bare A0509 MuJoCo arm states; mDPG is visual-only in the separate video",
        "torque_definition": "reconstructed MuJoCo inverse dynamics, not actuator or physical torque sensing",
        "decomposition": "inverse = M(q)ddq + gravity(q) + [qfrc_bias(q,dq)-gravity(q)] + dof_damping*dq",
        "gravity_method": "same A0509 MuJoCo model with logged q, zero qvel, and mj_forward",
        "sample_count": len(rows), "sim_duration_s": float(t[-1]),
        "sampling": {"nominal_hz": trial["trajectory_config"]["sample_hz"],
                     "observed_joint_state_hz": trial["quality"]["joint_states"]["observed_hz"],
                     "median_row_interval_s": median_dt, "max_row_interval_s": float(np.max(dt)),
                     "max_plan_row_time_deviation_s": float(np.max(np.abs(plan_time - t))),
                     "row_gaps_gt_1p5_median": missing_gap_count,
                     "upstream_estimated_missing_rate": trial["quality"]["joint_states"]["estimated_missing_rate"]},
        "tracking": {"joint_rmse_rad": _rms(tracking).tolist(),
                     "joint_max_abs_rad": np.max(np.abs(tracking), axis=0).tolist(),
                     "tcp_arm_rmse_mm": _rms(tcp_tracking).tolist(),
                     "tcp_arm_max_abs_mm": np.max(np.abs(tcp_tracking), axis=0).tolist(),
                     "planned_tcp_axis_range_mm": np.ptp(planned_tcp, axis=0).tolist(),
                     "measured_arm_tcp_axis_range_mm": np.ptp(measured_tcp, axis=0).tolist()},
        "distribution": {"q_range_rad": np.ptp(q, axis=0).tolist(),
                         "dq_rms_rad_s": _rms(dq).tolist(),
                         "dq_abs_max_rad_s": np.max(np.abs(dq), axis=0).tolist(),
                         "ddq_rms_rad_s2": _rms(ddq).tolist(),
                         "ddq_abs_max_rad_s2": np.max(np.abs(ddq), axis=0).tolist()},
        "torque": {"inverse_rms_nm": _rms(inverse).tolist(),
                   "inverse_abs_max_nm": np.max(np.abs(inverse), axis=0).tolist(),
                   "inertia_rms_nm": _rms(inertia).tolist(),
                   "gravity_rms_nm": _rms(gravity).tolist(),
                   "velocity_bias_rms_nm": _rms(velocity_bias).tolist(),
                   "damping_rms_nm": _rms(damping).tolist()},
        "quality": {"missing_required_or_actuator_cells": missing,
                    "qfrc_applied_unavailable_rows": applied_unavailable,
                    "nonfinite_required_rows": 0,
                    "gross_outlier_candidate_counts": gross,
                    "gross_screen_thresholds_are_heuristic": True,
                    "rows_with_contact_in_arm_model": int(np.sum(contacts > 0)),
                    "max_contact_count_in_arm_model": int(np.max(contacts)),
                    "physics_reconstruction_rmse_nm": float(_rms(residual, axis=None)),
                    "physics_reconstruction_max_abs_nm": float(np.max(np.abs(residual))),
                    "source_physics_actuator_available": missing["qfrc_actuator"] < len(rows)},
        "limits": ["one recorded trial", "mDPG mass/contact/suction not simulated",
                   "acceleration is a five-sample velocity slope in simulation time",
                   "gross anomaly screen does not detect every sensor or model error"],
    }
    _plots(output, t, dt, q, dq, ddq, target, tracking, measured_tcp,
           planned_tcp, inverse, inertia, gravity, velocity_bias, damping,
           residual, missing, gross, contacts, summary)
    with (output / "per_joint_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["joint", "tracking_rmse_rad", "tracking_max_abs_rad", "q_range_rad",
                         "dq_rms_rad_s", "ddq_rms_rad_s2", "inverse_rms_nm", "inverse_abs_max_nm",
                         "inertia_rms_nm", "gravity_rms_nm", "velocity_bias_rms_nm", "damping_rms_nm"])
        for j in range(6):
            writer.writerow([JOINTS[j], *[summary[group][field][j] for group, field in (
                ("tracking", "joint_rmse_rad"), ("tracking", "joint_max_abs_rad"),
                ("distribution", "q_range_rad"), ("distribution", "dq_rms_rad_s"),
                ("distribution", "ddq_rms_rad_s2"), ("torque", "inverse_rms_nm"),
                ("torque", "inverse_abs_max_nm"), ("torque", "inertia_rms_nm"),
                ("torque", "gravity_rms_nm"), ("torque", "velocity_bias_rms_nm"),
                ("torque", "damping_rms_nm"))]])
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def _plots(output, t, dt, q, dq, ddq, target, tracking, measured_tcp,
           planned_tcp, inverse, inertia, gravity, velocity_bias, damping,
           residual, missing, gross, contacts, summary):
    fig, axes = plt.subplots(4, 3, figsize=(15, 12), sharex=True)
    for j in range(6):
        ax = axes[j // 3, j % 3]
        ax.plot(t, target[:, j], "--", color="black", linewidth=1, label="Target")
        ax.plot(t, q[:, j], color=COLORS[j], linewidth=1, label="Actual")
        ax.set_title(f"{JOINTS[j]} | RMSE {summary['tracking']['joint_rmse_rad'][j]:.4f} rad")
        ax.set_ylabel("Position (rad)")
        ax.grid(alpha=.25)
        axes[2 + j // 3, j % 3].plot(t, tracking[:, j], color=COLORS[j], linewidth=1)
        axes[2 + j // 3, j % 3].set_ylabel(f"{JOINTS[j]} error (rad)")
        axes[2 + j // 3, j % 3].set_xlabel("ROS simulation time (s)")
        axes[2 + j // 3, j % 3].grid(alpha=.25)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("T1 joint tracking | arm-only MuJoCo source for mDPG visual replay")
    fig.tight_layout(); fig.savefig(output / "joint_tracking.png", dpi=150); plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for axis, label, unit in zip(range(3), "XYZ", ("X", "Y", "Z")):
        axes[axis].plot(t, planned_tcp[:, axis], "--", color="black", label="Planned")
        axes[axis].plot(t, measured_tcp[:, axis], color=COLORS[axis], label="Measured arm TCP")
        axes[axis].set_ylabel(f"{label} (mm)"); axes[axis].grid(alpha=.25)
    axes[0].legend(); axes[-1].set_xlabel("ROS simulation time (s)")
    fig.suptitle("T1 arm TCP tracking | this is not the uncalibrated mDPG suction TCP")
    fig.tight_layout(); fig.savefig(output / "tcp_tracking.png", dpi=150); plt.close(fig)

    fig, axes = plt.subplots(3, 6, figsize=(17, 9))
    for row, (values, label) in enumerate(((q, "q (rad)"), (dq, "dq (rad/s)"),
                                           (ddq, "ddq (rad/s²)"))):
        for j in range(6):
            ax = axes[row, j]
            ax.hist(values[:, j], bins=35, color=COLORS[j], alpha=.85)
            ax.set_title(f"{JOINTS[j]} {label}", fontsize=9)
            ax.grid(axis="y", alpha=.2)
    fig.suptitle("T1 q–dq–ddq distributions | full 100 Hz arm-only trial")
    fig.tight_layout(); fig.savefig(output / "q_dq_ddq_distribution.png", dpi=150); plt.close(fig)

    fig, axes = plt.subplots(2, 3, figsize=(15, 8))
    for j, ax in enumerate(axes.flat):
        points = ax.scatter(q[:, j], dq[:, j], c=ddq[:, j], s=5, cmap="coolwarm")
        ax.set_title(JOINTS[j]); ax.set_xlabel("q (rad)"); ax.set_ylabel("dq (rad/s)")
        fig.colorbar(points, ax=ax, label="ddq (rad/s²)")
        ax.grid(alpha=.2)
    fig.suptitle("T1 joint phase space | color = estimated acceleration")
    fig.tight_layout(); fig.savefig(output / "joint_phase_space.png", dpi=150); plt.close(fig)

    fig, axes = plt.subplots(2, 3, figsize=(16, 8), sharex=True)
    terms = ((inverse, "Inverse total", "black"), (inertia, "M·ddq", "#d55e00"),
             (gravity, "Gravity", "#2271b2"), (velocity_bias, "Velocity bias", "#009e73"),
             (damping, "Damping", "#aa3377"))
    for j, ax in enumerate(axes.flat):
        for values, label, color in terms:
            ax.plot(t, values[:, j], label=label, color=color,
                    linewidth=1.3 if label == "Inverse total" else .85)
        ax.set_title(JOINTS[j]); ax.set_xlabel("ROS simulation time (s)")
        ax.set_ylabel("Torque (Nm)"); ax.grid(alpha=.25)
    axes[0, 0].legend(fontsize=8, ncol=2)
    fig.suptitle("T1 reconstructed arm torque | no mDPG mass or pneumatic dynamics")
    fig.tight_layout(); fig.savefig(output / "torque_profile.png", dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(11, 5))
    x = np.arange(6)
    for index, (name, values) in enumerate((("Inertia", inertia), ("Gravity", gravity),
                                            ("Velocity bias", velocity_bias), ("Damping", damping))):
        ax.bar(x + (index - 1.5) * .2, _rms(values), .2, label=name)
    ax.set_xticks(x, JOINTS); ax.set_ylabel("Component RMS (Nm)")
    ax.set_title("T1 arm torque component magnitude (RMS)")
    ax.grid(axis="y", alpha=.25); ax.legend()
    fig.tight_layout(); fig.savefig(output / "torque_component_rms.png", dpi=150); plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    axes[0, 0].hist(dt, bins=np.linspace(0, .02, 31), color="#2271b2")
    axes[0, 0].set_title("Recorded ROS time interval")
    axes[0, 0].set_xlabel("Δt (s)"); axes[0, 0].set_ylabel("Rows")
    axes[0, 0].text(.98, .93, f"Max {np.max(dt):.3f} s", transform=axes[0, 0].transAxes,
                    ha="right", va="top")
    axes[0, 1].bar(np.arange(6), np.max(np.abs(residual), axis=0), color="#009e73")
    axes[0, 1].set_xticks(np.arange(6), JOINTS)
    axes[0, 1].set_title("Max physics reconstruction residual")
    axes[0, 1].set_ylabel("Nm")
    keys = list(missing)
    axes[1, 0].barh(keys, [missing[key] for key in keys], color="#aa3377")
    axes[1, 0].set_title("Missing CSV cells (actuator export unavailable)")
    axes[1, 0].set_xlabel("Blank cells")
    axes[1, 1].axis("off")
    message = (f"Rows: {len(t)} | duration: {t[-1]:.2f} s\n"
               f"JointState observed: {summary['sampling']['observed_joint_state_hz']:.2f} Hz\n"
               f"Row gaps > 1.5× median: {summary['sampling']['row_gaps_gt_1p5_median']}\n"
               f"Largest row interval: {np.max(dt):.3f} s\n"
               f"Rows with arm-model contact: {int(np.sum(contacts > 0))}\n"
               f"Physics max |error|: {np.max(np.abs(residual)):.2e} Nm\n"
               f"Gross spike candidates: {sum(gross.values())}\n"
               f"Actuator torque unavailable: {missing['qfrc_actuator']}/{len(t)} rows\n\n"
               "mDPG is visual-only; these are arm-model checks.")
    axes[1, 1].text(.02, .96, message, va="top", fontsize=9, wrap=True)
    fig.suptitle("T1 data quality summary")
    fig.tight_layout(); fig.savefig(output / "quality_summary.png", dpi=150); plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Analyze T1 source for mDPG visual replay")
    parser.add_argument("--source", default=str(ROOT / "results/mdpg_visual_source/simulation/T1/trial_001"))
    parser.add_argument("--video-metadata", default=str(ROOT / "docs/media/mdpg_t1_visual/metadata.json"))
    parser.add_argument("--output", default=str(ROOT / "docs/media/mdpg_t1_visual/analysis"))
    args = parser.parse_args(argv)
    result = analyze(args.source, args.video_metadata, args.output)
    print(json.dumps({"samples": result["sample_count"], "quality": result["quality"],
                      "tracking": result["tracking"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
