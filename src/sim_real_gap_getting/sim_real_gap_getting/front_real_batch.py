"""Prepare and run a supervised real A0509 front-path data collection batch.

Preparation is offline and records measured workspace inputs. Motion happens
only in ``run_prepared_real_batch`` after a separate operator action. Each
segment uses the existing guarded real_execute process and records RT data.
"""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from .front_gripper_audit import audit_front_grippers
from .front_trajectory import PROFILE_NAMES
from .trajectory import load_trajectory


SETUP = "setup_10cm"
LATERAL_MARGIN_MM = 20.0
LOWER_MARGIN_MM = 30.0


def _write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare_front_real_batch(source_dir: Path, arm_scene: Path, output_dir: Path, *,
                             tool: str, profiles: tuple[str, ...], repeats: int,
                             corridor_y_min_mm: float, corridor_y_max_mm: float,
                             lower_obstacle_top_z_mm: float,
                             mdpg_mount_xyz_m=(0.0, 0.0, 0.0),
                             mdpg_mount_rpy_rad=(0.0, 0.0, 0.0),
                             physical_tool_confirmed: bool = False,
                             other_obstacles_clear_confirmed: bool = False) -> Path:
    if tool not in ("ag145", "mdpg_1s1"):
        raise ValueError("select one physically mounted gripper")
    if not profiles or len(set(profiles)) != len(profiles) or any(p not in PROFILE_NAMES for p in profiles):
        raise ValueError("select one or more distinct F1-F4 profiles")
    if not isinstance(repeats, int) or isinstance(repeats, bool) or not 1 <= repeats <= 10:
        raise ValueError("repeats must be 1..10; start with one")
    values = (corridor_y_min_mm, corridor_y_max_mm, lower_obstacle_top_z_mm,
              *mdpg_mount_xyz_m, *mdpg_mount_rpy_rad)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("workspace and mount values must be finite numbers")
    if not 0 < corridor_y_max_mm - corridor_y_min_mm <= 400:
        raise ValueError("measured lateral corridor must be at most 400 mm")
    source_dir, arm_scene = Path(source_dir).resolve(), Path(arm_scene).resolve()
    if not arm_scene.is_file():
        raise FileNotFoundError(arm_scene)
    for name in (SETUP, *profiles):
        if not (source_dir / f"a0509_front_{name}.json").is_file():
            raise FileNotFoundError(source_dir / f"a0509_front_{name}.json")
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    run_dir = Path(output_dir).expanduser().resolve() / f"front_real_trial_{stamp}"
    run_dir.mkdir(parents=True)
    report_path = run_dir / "preflight.json"
    report = {
        "experiment_id": run_dir.name,
        "mode": "real_supervised_trial", "tool": tool,
        "profiles": list(profiles), "repeats_per_profile": repeats,
        "source_dir": str(source_dir), "arm_scene": str(arm_scene),
        "arm_scene_sha256": _sha(arm_scene),
        "corridor_y_mm": [corridor_y_min_mm, corridor_y_max_mm],
        "lower_obstacle_top_z_mm": lower_obstacle_top_z_mm,
        "lateral_margin_each_side_mm": LATERAL_MARGIN_MM,
        "lower_margin_mm": LOWER_MARGIN_MM,
        "mdpg_mount_xyz_m": list(mdpg_mount_xyz_m),
        "mdpg_mount_rpy_rad": list(mdpg_mount_rpy_rad),
        "physical_tool_confirmed_by_operator": bool(physical_tool_confirmed),
        "other_obstacles_clear_confirmed_by_operator": bool(other_obstacles_clear_confirmed),
        "model_checks": {}, "files": {},
        "ready_for_supervised_trial": False,
        "status": "preparing",
        "started_utc": datetime.now(timezone.utc).isoformat(),
    }
    report["required_start_joint_deg"] = [
        round(math.degrees(value), 3) for value in
        load_trajectory(source_dir / f"a0509_front_{SETUP}.json").points[0].q_rad
    ]
    _write_json(report_path, report)
    try:
        audit = audit_front_grippers(
            source_dir, run_dir / "models",
            arm_scene=arm_scene,
            lower_obstacle_top_z_mm=lower_obstacle_top_z_mm,
            planning_margin_mm=LOWER_MARGIN_MM,
            corridor_y_min_mm=corridor_y_min_mm,
            corridor_y_max_mm=corridor_y_max_mm,
            mdpg_mount_xyz_m=mdpg_mount_xyz_m,
            mdpg_mount_rpy_rad=mdpg_mount_rpy_rad)
        _write_json(run_dir / "model_audit.json", audit)
        report["model_audit"] = str(run_dir / "model_audit.json")
        selected = audit["tools"][tool]["paths"]
        for name in (SETUP, *profiles):
            item = selected[name]
            low = item["whole_robot_lateral_y_min_mm"]
            high = item["whole_robot_lateral_y_max_mm"]
            reasons = []
            if item["model_contact_steps_without_lower_obstacle"]:
                reasons.append("moving robot contacts model obstacle")
            if not item["tool_mass_simulation_preflight"]["passed"]:
                reasons.append("model tracking or torque preflight failed")
            if item["model_clearance_passed"] is not True:
                reasons.append("lower obstacle clearance under 30 mm")
            if low < corridor_y_min_mm + LATERAL_MARGIN_MM or high > corridor_y_max_mm - LATERAL_MARGIN_MM:
                reasons.append("moving arm/tool does not fit measured Y corridor with 20 mm side margins")
            report["model_checks"][name] = {
                "model_y_mm": [low, high],
                "minimum_model_z_mm": item["minimum_model_z_mm"],
                "model_contact_steps": item["model_contact_steps_without_lower_obstacle"],
                "passed": not reasons, "reasons": reasons,
            }
        report["ready_for_supervised_trial"] = (
            all(item["passed"] for item in report["model_checks"].values()) and
            physical_tool_confirmed and other_obstacles_clear_confirmed)
        if not physical_tool_confirmed:
            report["operator_blocker"] = "physical tool and adapter not confirmed"
        elif not other_obstacles_clear_confirmed:
            report["operator_blocker"] = "other obstacles not confirmed clear"
        if report["ready_for_supervised_trial"]:
            input_dir = run_dir / "inputs"
            input_dir.mkdir()
            for name in (SETUP, *profiles):
                source = source_dir / f"a0509_front_{name}.json"
                data = json.loads(source.read_text(encoding="utf-8"))
                data.update({
                    "gripper_type": tool,
                    "execution_scope": "supervised_real_trial",
                    "real_motion_validated": False,
                    "physical_preflight_report": str(report_path),
                    "source_sha256": _sha(source),
                })
                target = input_dir / source.name
                _write_json(target, data)
                load_trajectory(target)
                report["files"][name] = {"path": str(target), "sha256": _sha(target)}
        report["status"] = "ready_for_supervised_trial" if report["ready_for_supervised_trial"] else "blocked"
    except Exception as error:
        report["status"] = "error"
        report["reason"] = str(error)
        raise
    finally:
        report["finished_utc"] = datetime.now(timezone.utc).isoformat()
        _write_json(report_path, report)
    return report_path


def run_prepared_real_batch(report_path: Path, *, real_domain_id: int = 0,
                            stop_requested=None, progress=None) -> Path:
    report_path = Path(report_path).resolve()
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("status") != "ready_for_supervised_trial" or not report.get("ready_for_supervised_trial"):
        raise ValueError("physical preflight has not passed")
    if _sha(Path(report["arm_scene"])) != report["arm_scene_sha256"]:
        raise ValueError("collision scene changed after physical preflight")
    for entry in report["files"].values():
        if _sha(Path(entry["path"])) != entry["sha256"]:
            raise ValueError("prepared trajectory changed after preflight")
    run_dir = report_path.parent
    manifest_path = run_dir / "manifest.json"
    manifest = {
        "experiment_id": report["experiment_id"],
        "mode": "real_supervised_trial", "tool": report["tool"],
        "preflight_file": str(report_path),
        "segments": [], "result": "running",
        "started_utc": datetime.now(timezone.utc).isoformat(),
    }
    _write_json(manifest_path, manifest)
    segments = [(SETUP, 0)] + [(p, repeat_index)
                               for p in report["profiles"]
                               for repeat_index in range(1, report["repeats_per_profile"] + 1)]
    try:
        for index, (name, repeat_index) in enumerate(segments):
            if stop_requested is not None and stop_requested():
                raise RuntimeError("operator stopped real batch")
            segment_dir = run_dir / "data" / f"{index:02d}_{name}"
            segment_dir.mkdir(parents=True)
            command = [sys.executable, "-m", "sim_real_gap_getting.real_execute",
                       "--trajectory", report["files"][name]["path"],
                       "--gripper-type", report["tool"],
                       "--output-dir", str(segment_dir),
                       "--experiment-id", report["experiment_id"],
                       "--profile-name", name,
                       "--repeat-index", str(repeat_index),
                       "--operator-confirmed", "--supervised-trial"]
            environment = os.environ.copy()
            environment["ROS_DOMAIN_ID"] = str(real_domain_id)
            if progress is not None:
                progress({"event": "segment_start", "name": name,
                          "repeat_index": repeat_index,
                          "index": index + 1, "total": len(segments)})
            with (segment_dir / "runner.log").open("w", encoding="utf-8") as log:
                process = subprocess.Popen(command, env=environment, stdout=log,
                                           stderr=subprocess.STDOUT)
                signalled = False
                while process.poll() is None:
                    if stop_requested is not None and stop_requested() and not signalled:
                        process.send_signal(signal.SIGINT)
                        signalled = True
                    time.sleep(0.1)
            records = sorted(segment_dir.glob("*_real_execution_*.json"))
            record = json.loads(records[-1].read_text(encoding="utf-8")) if records else None
            entry = {"name": name, "repeat_index": repeat_index,
                     "index": index + 1, "exit_code": process.returncode,
                     "record_file": str(records[-1]) if records else None,
                     "result": record.get("result") if record else "no_record"}
            manifest["segments"].append(entry)
            _write_json(manifest_path, manifest)
            if progress is not None:
                progress({"event": "segment_done", **entry, "total": len(segments)})
            if process.returncode != 0 or record is None or record.get("result") != "completed":
                raise RuntimeError(f"real segment {name} failed; inspect {segment_dir / 'runner.log'}")
        manifest["result"] = "completed"
    except Exception as error:
        manifest["result"] = "aborted"
        manifest["reason"] = str(error)
        raise
    finally:
        manifest["ended_utc"] = datetime.now(timezone.utc).isoformat()
        _write_json(manifest_path, manifest)
    return manifest_path
