"""Classify F1–F4 real readiness without changing trajectory execution flags."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .front_trajectory import PROFILE_NAMES


def review_front_real(clearance_file: Path, validation_file: Path,
                      *, width_limit_mm: float = 400.0) -> dict:
    clearance = json.loads(Path(clearance_file).read_text(encoding="utf-8"))
    validation = json.loads(Path(validation_file).read_text(encoding="utf-8"))
    result = {
        "execution_scope": "simulation_only",
        "width_limit_mm": width_limit_mm,
        "clearance_report": str(Path(clearance_file).resolve()),
        "trajectory_validation": str(Path(validation_file).resolve()),
        "real_robot_executable": False,
        "tools": {},
    }
    for tool, tool_report in clearance["tools"].items():
        paths = {}
        for name in ("setup_10cm", *PROFILE_NAMES):
            path = tool_report["paths"][name]
            segments = ([tool_report["paths"]["setup_10cm"], path]
                        if name != "setup_10cm" else [path])
            trial_width = max(segment["whole_robot_lateral_span_mm"]
                              for segment in segments)
            reasons = []
            if any(segment["model_contact_steps_without_lower_obstacle"] for segment in segments):
                reasons.append("robot contact in MuJoCo scene")
            if any(not segment["tool_mass_simulation_preflight"]["passed"]
                   for segment in segments):
                reasons.append("MuJoCo tracking/torque preflight failed")
            if trial_width > width_limit_mm:
                reasons.append("conservative whole-robot width exceeds available width")
            if any(segment["model_clearance_passed"] is not True for segment in segments):
                reasons.append("lower obstacle top Z is unverified or clearance failed")
            if any(segment["measured_corridor_passed"] is not True for segment in segments):
                reasons.append("measured DR_BASE Y corridor is unverified or failed")
            if not tool_report["physical_mount_verified"]:
                reasons.append("physical gripper and adapter transform is unverified")
            if not validation["real_current_pose_verified"]:
                reasons.append("current real robot start pose is unverified")
            if not validation["real_100hz_interface_verified"]:
                reasons.append("real 100 Hz control interface is unverified")
            paths[name] = {
                "model_collision_free": all(segment["model_contact_steps_without_lower_obstacle"] == 0
                                            for segment in segments),
                "model_screen_passed": (all(segment["model_contact_steps_without_lower_obstacle"] == 0
                                            and segment["tool_mass_simulation_preflight"]["passed"]
                                            for segment in segments)
                                        and trial_width <= width_limit_mm),
                "trial_includes_setup": name != "setup_10cm",
                "model_width_mm": trial_width,
                "width_remaining_mm": round(width_limit_mm - trial_width, 2),
                "real_robot_executable": False,
                "execution_scope": "simulation_only",
                "reasons": reasons,
            }
        result["tools"][tool] = {
            "whole_campaign_width_mm": tool_report["whole_campaign_lateral_span_mm"],
            "whole_campaign_width_passed": (tool_report["whole_campaign_lateral_span_mm"]
                                             <= width_limit_mm),
            "paths": paths,
        }
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clearance-report", type=Path, required=True)
    parser.add_argument("--validation-report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    review = review_front_real(args.clearance_report, args.validation_report)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(review, indent=2) + "\n", encoding="utf-8")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
