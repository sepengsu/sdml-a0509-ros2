"""Run all four front cycles as independent, compressed MuJoCo trials.

Each trial performs the setup once, then repeats one closed cycle. The batch
never connects to ROS or sends commands to a physical A0509.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

from .front_gripper_audit import _default_trajectory_dir, audit_front_grippers
from .front_quality_audit import audit_front_quality
from .front_sim_collect import collect_front_sim
from .front_trajectory import PROFILE_NAMES


INPUT_FILES = ("a0509_front_setup_10cm.json",
               *(f"a0509_front_{profile}.json" for profile in PROFILE_NAMES))
TOOL_CHOICES = ("none", "ag145", "mdpg_1s1", "both")


def _write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def collect_front_batch(source_dir: Path, output_dir: Path, *,
                        tool: str = "ag145", repeats: int = 10,
                        arm_scene: Path | None = None,
                        stop_requested=None, progress=None) -> Path:
    if tool not in TOOL_CHOICES:
        raise ValueError(f"tool must be one of {TOOL_CHOICES}")
    if not isinstance(repeats, int) or isinstance(repeats, bool) or not 1 <= repeats <= 100:
        raise ValueError("repeats must be an integer from 1 to 100")
    source_dir = Path(source_dir).resolve()
    output_dir = Path(output_dir).expanduser().resolve()
    if arm_scene is not None:
        arm_scene = Path(arm_scene).expanduser().resolve()
        if not arm_scene.is_file():
            raise FileNotFoundError(arm_scene)
    for name in INPUT_FILES:
        if not (source_dir / name).is_file():
            raise FileNotFoundError(source_dir / name)
    tools = ("ag145", "mdpg_1s1") if tool == "both" else (tool,)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    run_dir = output_dir / f"front_batch_{stamp}"
    input_dir = run_dir / "inputs"
    input_dir.mkdir(parents=True)
    for name in INPUT_FILES:
        shutil.copy2(source_dir / name, input_dir / name)
    source_config = source_dir / "a0509_front_run_config.json"
    if source_config.is_file():
        config = json.loads(source_config.read_text(encoding="utf-8"))
        config["cycle_repeats"] = repeats
        config["batch_tool_selection"] = list(tools)
        _write_json(input_dir / source_config.name, config)
    manifest_path = run_dir / "manifest.json"
    manifest = {
        "experiment_id": run_dir.name,
        "mode": "simulation", "execution_scope": "simulation_only",
        "source_dir": str(source_dir), "run_dir": str(run_dir),
        "tool_models": list(tools), "profiles": list(PROFILE_NAMES),
        "arm_scene": str(arm_scene) if arm_scene is not None else "default_forward_wall",
        "arm_scene_sha256": (hashlib.sha256(arm_scene.read_bytes()).hexdigest()
                             if arm_scene is not None else None),
        "repeats_per_profile": repeats,
        "setup_policy": "once per independent tool/profile trial",
        "expected_trials": len(tools) * len(PROFILE_NAMES),
        "completed_trials": [],
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "result": "running",
    }
    _write_json(manifest_path, manifest)

    def emit(event: str, **fields):
        if progress is not None:
            progress({"event": event, "run_dir": str(run_dir), **fields})

    try:
        if stop_requested is not None and stop_requested():
            raise RuntimeError("batch stopped by user")
        emit("audit_start", message="Checking selected tool models and all four paths")
        clearance = audit_front_grippers(input_dir, run_dir / "models",
                                         arm_scene=arm_scene, tools=tools)
        audit_file = input_dir / "a0509_front_gripper_clearance.json"
        _write_json(audit_file, clearance)
        if clearance["status"] != "model_preflight_passed_physical_unverified":
            raise RuntimeError("tool and whole-robot model preflight failed")
        if stop_requested is not None and stop_requested():
            raise RuntimeError("batch stopped by user")
        quality = audit_front_quality(input_dir, audit_file)
        _write_json(input_dir / "a0509_front_quality_audit.json", quality)
        manifest["model_preflight"] = clearance["status"]
        manifest["lateral_width_screen"] = {
            name: {"span_mm": clearance["tools"][name]["whole_campaign_lateral_span_mm"],
                   "fits_400_mm": clearance["tools"][name]["whole_campaign_lateral_span_passed"]}
            for name in tools
        }
        emit("audit_done", message="Model contact/tracking preflight completed; real geometry unverified",
             lateral_width_screen=manifest["lateral_width_screen"])
        trial_number = 0
        for active_tool in tools:
            for profile in PROFILE_NAMES:
                if stop_requested is not None and stop_requested():
                    raise RuntimeError("batch stopped by user")
                emit("trial_start", tool=active_tool, profile=profile,
                     trial_number=trial_number + 1,
                     total_trials=manifest["expected_trials"])

                def on_sample(done: int, total: int):
                    emit("trial_progress", tool=active_tool, profile=profile,
                         trial_number=trial_number + 1,
                         total_trials=manifest["expected_trials"],
                         samples_done=done, samples_total=total)

                summary_path = collect_front_sim(
                    input_dir, run_dir / "data" / active_tool,
                    tool=active_tool, profile=profile, repeats=repeats,
                    stop_requested=stop_requested, progress=on_sample)
                trial = json.loads(summary_path.read_text(encoding="utf-8"))
                if trial["result"] != "completed":
                    raise RuntimeError(f"{active_tool}/{profile} did not complete")
                manifest["completed_trials"].append({
                    "tool": active_tool, "profile": profile,
                    "summary_file": str(summary_path),
                    "samples_file": trial["samples_file"],
                    "rows": trial["written_rows"],
                    "compressed_bytes": Path(trial["samples_file"]).stat().st_size,
                    "maximum_tracking_error_rad": trial["maximum_tracking_error_rad"],
                })
                trial_number += 1
                _write_json(manifest_path, manifest)
                emit("trial_done", tool=active_tool, profile=profile,
                     trial_number=trial_number,
                     total_trials=manifest["expected_trials"],
                     summary_file=str(summary_path))
        manifest["result"] = "completed"
        emit("complete", manifest=str(manifest_path))
    except Exception as error:
        manifest["result"] = "aborted"
        manifest["reason"] = str(error)
        emit("aborted", message=str(error))
        raise
    finally:
        manifest["ended_utc"] = datetime.now(timezone.utc).isoformat()
        _write_json(manifest_path, manifest)
    return manifest_path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Collect all four simulation-only front paths")
    parser.add_argument("--trajectory-dir", type=Path, default=_default_trajectory_dir())
    parser.add_argument("--output-dir", type=Path, default=Path.cwd() / "results/front_sim_batch")
    parser.add_argument("--tool", choices=TOOL_CHOICES, default="ag145")
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--arm-scene", type=Path,
                        help="Alternative A0509 MuJoCo scene, e.g. 60th + rear wall")
    args = parser.parse_args(argv)
    manifest = collect_front_batch(args.trajectory_dir, args.output_dir,
                                   tool=args.tool, repeats=args.repeats,
                                   arm_scene=args.arm_scene,
                                   progress=lambda event: print(json.dumps(event), flush=True))
    print(manifest)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
