"""Stable CSV/JSON layout shared by preview, simulation and real recording."""

from __future__ import annotations

import csv
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any, Iterable

from .trajectory_generator import TrajectoryPlan

JOINT_COLUMNS = [f"{stem}{i}" for stem in ("q", "dq", "ddq", "target_q") for i in range(1, 7)]
COMMON_COLUMNS = ["timestamp", "ros_time_s", *JOINT_COLUMNS, "tcp_position", "trajectory_id",
                  "trial_id", "gripper_type", "payload_mass", "mode"]
SIM_COLUMNS = ["qpos", "qvel", "qacc", "qfrc_inverse", "qfrc_actuator", "qfrc_applied", "qfrc_bias",
               "body_mass", "body_inertia", "dof_frictionloss", "dof_damping"]
REAL_COLUMNS = ["actual_joint_position", "actual_joint_velocity", "actual_motor_torque",
                "actual_joint_torque", "external_joint_torque", "actual_tcp_position",
                "target_joint_position", "joint_temperature", "rt_time_stamp"]
DYNAMICS_COLUMNS = COMMON_COLUMNS + SIM_COLUMNS + REAL_COLUMNS
TRAJECTORY_COLUMNS = ["time_s", "phase", "waypoint", "x_mm", "y_mm", "z_mm",
                      "vx_mm_s", "vy_mm_s", "vz_mm_s", "ax_mm_s2", "ay_mm_s2", "az_mm_s2",
                      *[f"q{i}_rad" for i in range(1, 7)],
                      *[f"dq{i}_rad_s" for i in range(1, 7)],
                      *[f"ddq{i}_rad_s2" for i in range(1, 7)]]


def create_trial(root: str | Path, mode: str, trajectory_id: str) -> tuple[Path, str]:
    if mode not in ("preview", "simulation", "real"):
        raise ValueError("invalid mode")
    parent = Path(root).expanduser().resolve() / mode / trajectory_id
    parent.mkdir(parents=True, exist_ok=True)
    for number in range(1, 10000):
        trial_id = f"trial_{number:03d}"
        path = parent / trial_id
        try:
            path.mkdir()
            return path, trial_id
        except FileExistsError:
            continue
    raise RuntimeError("trial directory limit reached")


def _sequence(row: dict[str, Any], prefix: str, values: tuple[float, ...] | None) -> None:
    if values is not None:
        for i, value in enumerate(values, 1):
            row[f"{prefix}{i}_rad" if prefix == "q" else f"{prefix}{i}_rad_s" if prefix == "dq" else f"{prefix}{i}_rad_s2"] = value


def save_trajectory(plan: TrajectoryPlan, folder: Path) -> None:
    with (folder / "trajectory.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=TRAJECTORY_COLUMNS)
        writer.writeheader()
        for point in plan.points:
            row: dict[str, Any] = {"time_s": point.time_s, "phase": point.phase, "waypoint": int(point.waypoint)}
            for values, suffix in ((point.xyz_mm, ("x_mm", "y_mm", "z_mm")),
                                   (point.velocity_mm_s, ("vx_mm_s", "vy_mm_s", "vz_mm_s")),
                                   (point.acceleration_mm_s2, ("ax_mm_s2", "ay_mm_s2", "az_mm_s2"))):
                if values is not None:
                    row.update(zip(suffix, values))
            _sequence(row, "q", point.q_rad)
            _sequence(row, "dq", point.dq_rad_s)
            _sequence(row, "ddq", point.ddq_rad_s2)
            writer.writerow(row)
    (folder / "trajectory.json").write_text(json.dumps(plan.as_dict(), indent=2) + "\n", encoding="utf-8")
    with (folder / "waypoints.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=TRAJECTORY_COLUMNS)
        writer.writeheader()
        with (folder / "trajectory.csv").open(newline="", encoding="utf-8") as source:
            for row in csv.DictReader(source):
                if row["waypoint"] == "1":
                    writer.writerow(row)


def _csv_value(value: Any) -> Any:
    if value is None:
        return ""
    if isinstance(value, (tuple, list, dict)):
        return json.dumps(value, separators=(",", ":"))
    return value


def save_dynamics(rows: Iterable[dict[str, Any]], folder: Path) -> int:
    count = 0
    with (folder / "dynamics.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=DYNAMICS_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _csv_value(row.get(key)) for key in DYNAMICS_COLUMNS})
            count += 1
    return count


def save_metadata(folder: Path, plan: TrajectoryPlan, mode: str, trial_id: str,
                  extra: dict[str, Any] | None = None) -> None:
    metadata: dict[str, Any] = {
        "schema_version": "1.0", "created_utc": datetime.now(timezone.utc).isoformat(),
        "trajectory_id": plan.config.trajectory_id, "trial_id": trial_id, "mode": mode,
        "trajectory_config": asdict(plan.config), "joint_order": [f"joint_{i}" for i in range(1, 7)],
        "units": {"q": "rad", "dq": "rad/s", "ddq": "rad/s^2", "target_q": "rad",
                  "tcp_position": "mm", "torque": "Nm", "timestamp": "UTC Unix seconds",
                  "ros_time_s": "ROS clock seconds"},
        "time_sync": "ROS node time is stored as ros_time_s. Topic freshness and RT joins use local monotonic arrival time; RT topics have no header.",
        "filtering": "none", "missing_data": "Blank CSV cells mean unavailable or stale measurements.",
        "safety": "Trajectory dimensions are unvalidated research examples and are never sufficient for real motion approval.",
    }
    if extra:
        metadata.update(extra)
    (folder / "metadata.json").write_text(json.dumps(metadata, indent=2, default=str) + "\n", encoding="utf-8")
