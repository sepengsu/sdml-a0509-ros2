"""Per-trajectory analysis of the actual T5-T14 MuJoCo CSV observations."""

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
import numpy as np

from .data import REPO_ROOT
from .dynamics10_data import TRAJECTORIES, _json_array, _six


def _stat(values: np.ndarray) -> dict:
    return {"min": values.min(axis=0).tolist(), "max": values.max(axis=0).tolist(),
            "rms": np.sqrt(np.mean(values**2, axis=0)).tolist(),
            "abs_max": np.max(np.abs(values), axis=0).tolist()}


def analyze(source: str | Path, output: str | Path) -> dict:
    source, output = Path(source).resolve(), Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    figures = output / "figures"
    figures.mkdir(exist_ok=True)
    source_manifest_path = source / "dataset_manifest.json"
    source_manifest = json.loads(source_manifest_path.read_text(encoding="utf-8"))
    report = {"created_utc": datetime.now(timezone.utc).isoformat(),
              "source_manifest_sha256": hashlib.sha256(source_manifest_path.read_bytes()).hexdigest(),
              "torque_definition": "MuJoCo inverse dynamics reconstructed at observed q,dq,ddq; not physical torque sensing",
              "description": "descriptive analysis only; no model selection on T13/T14",
              "trajectories": {}, "coverage": source_manifest["coverage"],
              "similarity": source_manifest["trajectory_similarity"]}
    summary_rows = []
    for trajectory in TRAJECTORIES:
        entry = source_manifest["status"][trajectory]
        folder = source / entry["simulation_relative_path"]
        with (folder / "dynamics.csv").open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        with (folder / "trajectory.csv").open(newline="", encoding="utf-8") as stream:
            planned = list(csv.DictReader(stream))
        if len(rows) != len(planned) or not entry["quality"]["pass"]:
            raise ValueError(f"invalid or mismatched trial: {trajectory}")
        q, dq, ddq, target = (np.stack([_six(row, stem) for row in rows])
                              for stem in ("q", "dq", "ddq", "target_q"))
        torque = np.stack([_json_array(row, "qfrc_inverse", (6,)) for row in rows])
        t = np.asarray([float(row["ros_time_s"]) for row in rows])
        t -= t[0]
        motion = np.asarray([row["phase"].startswith("motion") for row in planned])
        hold = ~motion
        tracking = q - target
        stats = {"position_rad": _stat(q), "velocity_rad_s": _stat(dq),
                 "acceleration_rad_s2": _stat(ddq), "inverse_torque_nm": _stat(torque),
                 "tracking_error_rad": _stat(tracking)}
        stats["position_range_rad"] = (q.max(axis=0) - q.min(axis=0)).tolist()
        stats["motion_inverse_torque_rms_nm"] = np.sqrt(np.mean(torque[motion]**2, axis=0)).tolist()
        stats["hold_inverse_torque_rms_nm"] = np.sqrt(np.mean(torque[hold]**2, axis=0)).tolist()
        joint_state_hz = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))["quality"]["joint_states"]
        record = {"split": entry["split"], "trial_id": folder.name, "full_rate_samples": len(rows),
                  "motion_samples": int(motion.sum()), "hold_samples": int(hold.sum()),
                  "sim_time_duration_s": float(t[-1]),
                  "joint_state_observed_hz": joint_state_hz["observed_hz"],
                  "joint_state_estimated_missing_rate": joint_state_hz["estimated_missing_rate"],
                  "sim_time_monotonic": entry["quality"]["sim_time_strictly_increasing"],
                  "limit_violation_counts": entry["quality"]["limit_violation_counts"],
                  "active_joints": sorted(set(component["joint"] for component in entry["definition"]["components"])),
                  "statistics": stats, "source_folder": entry["simulation_relative_path"]}
        report["trajectories"][trajectory] = record
        for j in range(6):
            summary_rows.append([trajectory, entry["split"], folder.name, j+1, int(j+1 in record["active_joints"]),
                                 len(rows), stats["position_rad"]["min"][j],
                                 stats["position_rad"]["max"][j], stats["position_range_rad"][j],
                                 stats["velocity_rad_s"]["rms"][j],
                                 stats["velocity_rad_s"]["abs_max"][j],
                                 stats["acceleration_rad_s2"]["rms"][j],
                                 stats["acceleration_rad_s2"]["abs_max"][j],
                                 stats["inverse_torque_nm"]["rms"][j],
                                 stats["inverse_torque_nm"]["abs_max"][j],
                                 stats["tracking_error_rad"]["rms"][j],
                                 stats["tracking_error_rad"]["abs_max"][j]])
        fig, axes = plt.subplots(5, 1, figsize=(12, 12), sharex=True)
        panels = ((q, "q (rad)"), (dq, "dq (rad/s)"), (ddq, "ddq (rad/s²)"),
                  (torque, "inverse torque (Nm)"), (tracking, "target error (rad)"))
        for ax, (values, label) in zip(axes, panels):
            for j in range(6):
                ax.plot(t, values[:, j], label=f"J{j+1}", linewidth=.85)
            ax.set_ylabel(label)
            ax.grid(alpha=.25)
            ax.axvspan(t[0], t[np.flatnonzero(motion)[0]], color="gray", alpha=.07)
            ax.axvspan(t[np.flatnonzero(motion)[-1]], t[-1], color="gray", alpha=.07)
        axes[0].legend(ncol=6, fontsize=8)
        axes[-1].set_xlabel("ROS simulation time from trial start (s)")
        fig.suptitle(f"{trajectory} | {entry['definition']['name']} | {entry['split']} | AG145 MuJoCo")
        fig.tight_layout()
        fig.savefig(figures / f"{trajectory}_joint_dynamics.png", dpi=145)
        plt.close(fig)

    with (output / "per_joint_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["trajectory_id", "split", "trial_id", "joint", "active", "samples",
                         "q_min_rad", "q_max_rad", "q_range_rad", "dq_rms_rad_s", "dq_abs_max_rad_s",
                         "ddq_rms_rad_s2", "ddq_abs_max_rad_s2", "inverse_torque_rms_nm",
                         "inverse_torque_abs_max_nm", "tracking_rms_rad", "tracking_abs_max_rad"])
        writer.writerows(summary_rows)
    (output / "analysis.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Analyze each T5-T14 recorded MuJoCo trajectory")
    parser.add_argument("--source", default=str(REPO_ROOT / "results" / "dynamics10"))
    parser.add_argument("--output", default=str(REPO_ROOT / "model" / "residual_runs" / "dynamics10_v1" / "analysis"))
    args = parser.parse_args(argv)
    report = analyze(args.source, args.output)
    for name, row in report["trajectories"].items():
        print(name, row["split"], row["full_rate_samples"], "max inverse Nm",
              round(max(row["statistics"]["inverse_torque_nm"]["abs_max"]), 2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
