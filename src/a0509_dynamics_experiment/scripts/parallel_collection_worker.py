"""Run one isolated shard of the seven-condition MuJoCo collection campaign.

The PowerShell launcher assigns each (condition, repeat) to exactly one worker.
This process runs inside a simulation-only Docker container and never connects
to a real Doosan controller.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import traceback


SPEC = Path("/ros2_ws/src/a0509_dynamics_experiment/config/payload_conditions.json")
RESULTS = Path("/ros2_ws/results")


def timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_status(path: Path, **values) -> None:
    current = json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}
    current.update(values)
    current["updated_utc"] = timestamp()
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(current, indent=2) + "\n", encoding="utf-8")
    temporary.replace(path)


def completed(job: dict, campaign: dict, spec_sha256: str) -> bool:
    parent = (RESULTS / campaign["results_name"] / job["condition"] /
              f"repeat_{job['repeat']:03d}" / "simulation" / job["trajectory"])
    for trial in sorted(parent.glob("trial_*")) if parent.is_dir() else ():
        if not all((trial / name).is_file() for name in
                   ("metadata.json", "dynamics.csv", "rosbag/metadata.yaml")):
            continue
        try:
            metadata = json.loads((trial / "metadata.json").read_text(encoding="utf-8"))
            if not (metadata.get("execution_success") is True
                    and int(metadata.get("sample_count", 0)) > 0
                    and int(metadata.get("quality", {}).get("rows_written", 0))
                    == int(metadata["sample_count"])
                    and metadata.get("trajectory_id") == job["trajectory"]
                    and metadata.get("gripper_type") == job["gripper"]
                    and math.isclose(float(metadata.get("payload_mass_kg", -1)),
                                     float(job["payload_kg"]), abs_tol=1e-12)):
                continue
            if job["payload_kg"] > 0 and (metadata.get("payload_model") or {}).get("spec_sha256") != spec_sha256:
                continue
            if campaign.get("record_video") and not (trial / "simulation.mp4").is_file():
                continue
            if job["gripper"] == "ag145" and job["payload_kg"] > 0:
                gripper = metadata.get("gripper_model") or {}
                if (gripper.get("configuration") != "fixed grasp pose"
                        or not math.isclose(float(gripper.get("grasp_angle_rad", -1)),
                                            0.60, abs_tol=1e-12)):
                    continue
            if job["gripper"] == "mdpg_1s1":
                gripper = metadata.get("gripper_model") or {}
                mount = campaign["mount"]
                if (gripper.get("dynamics_model") != "box_approx"
                        or any(not math.isclose(float(a), float(b), abs_tol=1e-12)
                               for key, saved in (("xyz_m", "mount_xyz_m"),
                                                  ("rpy_rad", "mount_rpy_rad"))
                               for a, b in zip(mount[key], gripper.get(saved, [])))
                        or len(gripper.get("mount_xyz_m", [])) != 3
                        or len(gripper.get("mount_rpy_rad", [])) != 3):
                    continue
            return True
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
            continue
    return False


def command(job: dict, campaign: dict, domain: int) -> list[str]:
    root = (RESULTS / campaign["results_name"] / job["condition"] /
            f"repeat_{job['repeat']:03d}")
    argv = ["ros2", "run", "a0509_dynamics_experiment", "run_experiment",
            "--simulation", "--trajectory", job["trajectory"],
            "--gripper-type", job["gripper"], "--payload-mass", f"{job['payload_kg']:.1f}",
            "--results-dir", str(root), "--sim-domain-id", str(domain),
            "--sample-hz", "100", "--defer-summary"]
    if job["gripper"] == "mdpg_1s1":
        argv += ["--mdpg-dynamics-model", "box_approx", "--mdpg-mount-xyz"]
        argv += [str(value) for value in campaign["mount"]["xyz_m"]]
        argv += ["--mdpg-mount-rpy"]
        argv += [str(value) for value in campaign["mount"]["rpy_rad"]]
    if job["payload_kg"] > 0:
        argv += ["--payload-spec-file", str(SPEC)]
    if campaign.get("record_video"):
        argv.append("--record-video")
    return argv


def run(manifest_path: Path, index: int) -> int:
    campaign = json.loads(manifest_path.read_text(encoding="utf-8"))
    if campaign.get("schema_version") != 1:
        raise ValueError("unsupported campaign manifest")
    if not campaign["results_name"].replace("_", "").replace("-", "").isalnum():
        raise ValueError("invalid results name")
    if not 0 <= index < campaign["robot_count"]:
        raise ValueError("invalid worker index")
    domain = campaign["base_domain_id"] + index
    if not 0 <= domain <= 232:
        raise ValueError("ROS domain out of range")
    if not SPEC.is_file() or hashlib.sha256(SPEC.read_bytes()).hexdigest() != campaign["payload_spec_sha256"]:
        raise ValueError("payload specification differs from campaign manifest")
    jobs = [job for job in campaign["jobs"] if job["worker"] == index]
    status = manifest_path.parent / f"worker_{index+1:02d}.status.json"
    log_path = manifest_path.parent / f"worker_{index+1:02d}.log"
    write_status(status, state="running", assigned=len(jobs), completed=0, skipped=0,
                 ran=0, domain_id=domain, container=campaign["containers"][index],
                 started_utc=timestamp(), current_job=None, error=None)
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(domain)
    done = skipped = ran = 0
    with log_path.open("a", encoding="utf-8", buffering=1) as log:
        print(f"[{timestamp()}] worker {index+1} domain {domain}; {len(jobs)} assigned", file=log)
        try:
            shards = sorted({(job["condition"], job["repeat"]) for job in jobs})
            for condition, repeat in shards:
                shard_jobs = [job for job in jobs if (job["condition"], job["repeat"]) == (condition, repeat)]
                for job in shard_jobs:
                    name = f"{condition}/{job['trajectory']}/repeat_{repeat:03d}"
                    write_status(status, current_job=name, completed=done, skipped=skipped, ran=ran)
                    if hashlib.sha256(SPEC.read_bytes()).hexdigest() != campaign["payload_spec_sha256"]:
                        raise RuntimeError("payload specification changed during collection")
                    if completed(job, campaign, campaign["payload_spec_sha256"]):
                        skipped += 1
                        print(f"[{timestamp()}] SKIP {name}", file=log)
                    else:
                        argv = command(job, campaign, domain)
                        print(f"[{timestamp()}] RUN {name}: {' '.join(argv)}", file=log)
                        result = subprocess.run(argv, cwd="/ros2_ws", env=environment,
                                                stdout=log, stderr=subprocess.STDOUT, check=False)
                        if result.returncode != 0 or not completed(job, campaign, campaign["payload_spec_sha256"]):
                            raise RuntimeError(f"{name} failed (exit {result.returncode}); see {log_path}")
                        ran += 1
                    done += 1
                    write_status(status, completed=done, skipped=skipped, ran=ran)
                # T5-T14 quality and coverage are audited once per shard.
                from a0509_dynamics_experiment.dynamics10_dataset import summarize
                shard_root = RESULTS / campaign["results_name"] / condition / f"repeat_{repeat:03d}"
                print(f"[{timestamp()}] AUDIT {shard_root}", file=log)
                summarize(shard_root)
            write_status(status, state="complete", completed=done, skipped=skipped,
                         ran=ran, current_job=None, finished_utc=timestamp())
            print(f"[{timestamp()}] COMPLETE {done}/{len(jobs)}", file=log)
            return 0
        except Exception as error:
            traceback.print_exc(file=log)
            write_status(status, state="failed", completed=done, skipped=skipped,
                         ran=ran, error=str(error), finished_utc=timestamp())
            return 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--index", type=int, required=True)
    args = parser.parse_args()
    sys.exit(run(args.manifest, args.index))
