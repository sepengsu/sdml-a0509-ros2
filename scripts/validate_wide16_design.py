"""Validate the new wide16 plans against saved MuJoCo condition models."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/a0509_dynamics_experiment"))

from a0509_dynamics_experiment.dynamics10_safety import validate_joint_plan
from a0509_dynamics_experiment.wide16_trajectory import generate_wide
from model.campaign_rbd import DEFAULT_CAMPAIGN, load_scene


OUTPUT = ROOT / "results/wide16_v3_design_preflight.json"
CONDITIONS = ("bare_p0", "ag145_p0", "ag145_p05", "ag145_p10",
              "mdpg_p0", "mdpg_p05", "mdpg_p10")


def main() -> None:
    plans = {f"T{i}": generate_wide(f"T{i}") for i in range(1, 17)}
    records = []
    for condition in CONDITIONS:
        model = load_scene(DEFAULT_CAMPAIGN / "models/conditions" / f"{condition}.xml")
        data = mujoco.MjData(model)
        tcp_id = (model.site("ag145_tcp").id if condition.startswith("ag145") else
                  model.site("mdpg_tcp").id if condition.startswith("mdpg") else -1)
        flange_id = model.body("link_6").id
        for trajectory, plan in plans.items():
            limits = validate_joint_plan(plan)
            points = np.empty((len(plan.points), 3), dtype=float)
            for index, point in enumerate(plan.points):
                data.qpos[:] = point.q_rad
                mujoco.mj_forward(model, data)
                if data.ncon:
                    raise RuntimeError(f"{condition}/{trajectory} contact at sample {index}")
                points[index] = (data.site_xpos[tcp_id] if tcp_id >= 0 else
                                 data.xpos[flange_id])
            span_deg = np.rad2deg(np.subtract(limits["position_max_rad"],
                                             limits["position_min_rad"]))
            record = {
                "condition": condition, "trajectory": trajectory,
                "samples": len(plan.points), "duration_s": plan.points[-1].time_s,
                "max_joint_span_deg": round(float(span_deg.max()), 2),
                "tcp_max_from_start_mm": round(float(np.linalg.norm(points - points[0], axis=1).max()*1000), 1),
                "tcp_xyz_span_mm": (np.ptp(points, axis=0) * 1000).round(1).tolist(),
                "peak_velocity_rad_s": round(float(max(limits["peak_velocity_rad_s"])), 4),
                "peak_acceleration_rad_s2": round(float(max(limits["peak_acceleration_rad_s2"])), 4),
                "contact_samples": 0,
            }
            records.append(record)
            print(f"{condition} {trajectory}: {record['tcp_max_from_start_mm']} mm, no contact",
                  flush=True)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps({"profile": "wide16_v3", "simulation_only": True,
                                  "conditions": CONDITIONS, "records": records},
                                 indent=2) + "\n", encoding="utf-8")
    csv_path = OUTPUT.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)
    print(OUTPUT)


if __name__ == "__main__":
    main()
