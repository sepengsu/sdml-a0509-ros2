"""Prepare one joint seven-condition dataset for M0-M4.

Only the first complete deterministic repeat of each trajectory is used. The
script requires all seven conditions and all sixteen trajectories before it
creates an output. It reads simulator inverse-dynamics labels, not motor torque.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import tempfile

import numpy as np

from .campaign_rbd import load_scene
from .rbd import RBDModel


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / "data/grid_16_wide_v3"
DEFAULT_OUTPUT = ROOT / "data/wide16_v3_joint_dataset"
CONDITIONS = ("bare_p0", "ag145_p0", "ag145_p05", "ag145_p10",
              "mdpg_p0", "mdpg_p05", "mdpg_p10")
SPLITS = {**{f"T{i}": "train" for i in range(1, 13)},
          "T13": "validation", "T14": "validation",
          "T15": "test", "T16": "test"}
LEGACY_SPLITS = {**{f"T{i}": "diagnostic" for i in range(1, 5)},
                 **{f"T{i}": "train" for i in range(5, 12)},
                 "T12": "validation", "T13": "test", "T14": "test",
                 "T15": "train", "T16": "test"}
PROFILES = {"wide16_v3": SPLITS, "trajectory16_legacy": LEGACY_SPLITS}
GRIPPER = {"bare": 0, "ag145": 1, "mdpg": 2}


def _selected_trial(source: Path, condition: str, trajectory: str,
                    profile: str, split_policy: dict[str, str]) -> tuple[Path, dict]:
    base = source / "simulation" / trajectory / condition
    for folder in sorted(base.glob("trial_[0-9][0-9][0-9]")):
        meta_path = folder / "metadata.json"
        csv_path = folder / "dynamics.csv"
        if not meta_path.is_file() or not csv_path.is_file():
            continue
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        if (meta.get("mode") != "simulation" or meta.get("condition_id") != condition
                or meta.get("trajectory_id") != trajectory or meta.get("trial_id") != folder.name
                or meta.get("dataset_split") != split_policy[trajectory]
                or (meta.get("trajectory_config", {}).get("profile")
                    != (profile if profile == "wide16_v3" else None))
                or (profile == "trajectory16_legacy" and meta.get("campaign") is not None)):
            raise ValueError(f"campaign identity/split/profile mismatch: {folder}")
        if meta.get("joint_order") != [f"joint_{j}" for j in range(1, 7)]:
            raise ValueError(f"joint order mismatch: {folder}")
        units = meta.get("units", {})
        if tuple(units.get(k) for k in ("q", "dq", "ddq", "torque")) != ("rad", "rad/s", "rad/s^2", "Nm"):
            raise ValueError(f"units mismatch: {folder}")
        quality = meta.get("quality", {})
        report = meta.get("report", {})
        if (quality.get("rows_written", 0) < 2 or
                quality.get("joint_states", {}).get("estimated_missing_rate", 1) > 0.01 or
                report.get("max_tracking_error_rad", 1) > 0.05 or
                report.get("torque_limit_samples") != 0 or
                report.get("cross_robot_contacts") != 0):
            raise ValueError(f"trial failed quality gate: {folder}")
        bag = meta.get("rosbag_relative_path")
        if not bag or not (source / bag / "metadata.yaml").is_file():
            raise ValueError(f"missing rosbag completion marker: {folder}")
        return folder, meta
    raise FileNotFoundError(f"complete {profile} trial missing: {base}")


def _numbers(row: dict, stem: str) -> np.ndarray:
    result = np.asarray([float(row[f"{stem}{j}"]) for j in range(1, 7)], dtype=np.float64)
    if not np.isfinite(result).all():
        raise ValueError(f"nonfinite {stem}")
    return result


def _reference(row: dict, q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> np.ndarray:
    tau = np.asarray(json.loads(row["qfrc_inverse"]), dtype=np.float64)
    bias = np.asarray(json.loads(row["qfrc_bias"]), dtype=np.float64)
    damping = np.asarray(json.loads(row["dof_damping"]), dtype=np.float64)
    mass = np.asarray(json.loads(row["mass_matrix"]), dtype=np.float64)
    if (tau.shape != (6,) or bias.shape != (6,) or damping.shape != (6,)
            or mass.shape != (6, 6) or not all(np.isfinite(x).all() for x in (tau, bias, damping, mass))):
        raise ValueError("invalid inverse dynamics fields")
    if np.max(np.abs(mass @ ddq + bias + damping * dq - tau)) > 2e-5:
        raise ValueError("inverse dynamics identity mismatch")
    return tau


def prepare(source: Path, output: Path, *, stride: int = 25, nominal_scale: float = 0.8,
            profile: str = "wide16_v3") -> dict:
    source, output = source.resolve(), output.resolve()
    if profile not in PROFILES:
        raise ValueError(f"profile must be one of {tuple(PROFILES)}")
    split_policy = PROFILES[profile]
    if stride < 1 or not 0 < nominal_scale <= 1:
        raise ValueError("stride must be positive and nominal scale must be in (0, 1]")
    if output.exists():
        raise FileExistsError(f"choose a new --output path: {output}")
    if source in output.parents or source == output:
        raise ValueError("output must be outside the source campaign")
    selected = {(condition, trajectory): _selected_trial(source, condition, trajectory,
                                                          profile, split_policy)
                for condition in CONDITIONS for trajectory in split_policy}
    scenes = {condition: source / "models/conditions" / f"{condition}.xml" for condition in CONDITIONS}
    for scene in scenes.values():
        if not scene.is_file():
            raise FileNotFoundError(scene)
    all_features, all_reference, all_nominal = [], [], []
    condition_codes, trajectory_codes, split_codes = [], [], []
    trials = []
    for ci, condition in enumerate(CONDITIONS):
        nominal = RBDModel(load_scene(scenes[condition], nominal_scale))
        exact = RBDModel(load_scene(scenes[condition]))
        for trajectory in split_policy:
            folder, meta = selected[condition, trajectory]
            q_rows, dq_rows, ddq_rows, tau_rows = [], [], [], []
            count = 0
            with (folder / "dynamics.csv").open(encoding="utf-8", newline="") as stream:
                for count, row in enumerate(csv.DictReader(stream), start=1):
                    if (row["trajectory_id"] != trajectory or row["trial_id"] != folder.name
                            or row["mode"] != "simulation"):
                        raise ValueError(f"CSV identity mismatch: {folder}, row {count}")
                    if (count - 1) % stride:
                        continue
                    q, dq, ddq = (_numbers(row, stem) for stem in ("q", "dq", "ddq"))
                    tau = _reference(row, q, dq, ddq)
                    q_rows.append(q)
                    dq_rows.append(dq)
                    ddq_rows.append(ddq)
                    tau_rows.append(tau)
            if count != meta["quality"]["rows_written"] or not tau_rows:
                raise ValueError(f"CSV row count mismatch: {folder}")
            q, dq, ddq, tau = (np.stack(values) for values in (q_rows, dq_rows, ddq_rows, tau_rows))
            for index in (0, len(q) - 1):
                if np.max(np.abs(exact.evaluate(q[index], dq[index], ddq[index])["torque"] - tau[index])) > 2e-5:
                    raise ValueError(f"saved scene does not reproduce label: {folder}")
            # Bare has no tool or payload inertial to scale, so its nominal
            # RBD is exactly the reference scene verified above.
            tau_nominal = tau.copy() if condition == "bare_p0" else nominal.evaluate_many(q, dq, ddq)["torque"]
            gripper_name = "bare" if condition == "bare_p0" else condition.split("_")[0]
            code = np.zeros((len(q), 4), dtype=np.float32)
            code[:, GRIPPER[gripper_name]] = 1
            code[:, 3] = float(meta["payload_mass_kg"])
            if not np.isfinite(code).all() or (code[:, 3] < 0).any():
                raise ValueError(f"invalid payload mass: {folder}")
            features = np.concatenate((q, dq, ddq, code), axis=1).astype(np.float32)
            all_features.append(features)
            all_reference.append(tau.astype(np.float32))
            all_nominal.append(tau_nominal.astype(np.float32))
            condition_codes.append(np.full(len(q), ci, dtype=np.uint8))
            trajectory_codes.append(np.full(len(q), int(trajectory[1:]), dtype=np.uint8))
            split_codes.append(np.full(len(q), ("train", "validation", "test", "diagnostic").index(split_policy[trajectory]),
                                       dtype=np.uint8))
            trials.append({"condition": condition, "trajectory": trajectory, "trial": folder.name,
                           "source_rows": count, "selected_rows": len(q), "split": split_policy[trajectory],
                           "metadata_sha256": hashlib.sha256((folder / "metadata.json").read_bytes()).hexdigest()})
            print(f"{condition}/{trajectory}: {len(q)} rows", flush=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=".joint_", dir=output.parent))
    try:
        arrays = {"features": np.concatenate(all_features), "tau_reference": np.concatenate(all_reference),
                  "tau_nominal": np.concatenate(all_nominal), "condition": np.concatenate(condition_codes),
                  "trajectory": np.concatenate(trajectory_codes), "split": np.concatenate(split_codes)}
        np.savez_compressed(temporary / "dataset.npz", **arrays)
        report = {"schema_version": 1, "created_utc": datetime.now(timezone.utc).isoformat(),
                  "source": str(source), "profile": profile, "stride": stride,
                  "selected_repeat_policy": "first complete trial only", "conditions": CONDITIONS,
                  "split_codes": {name: code for code, name in enumerate(
                      ("train", "validation", "test", "diagnostic")) if name in split_policy.values()},
                  "split_policy": {split: [t for t, name in split_policy.items() if name == split]
                                   for split in ("train", "validation", "test", "diagnostic")
                                   if split in split_policy.values()},
                  "feature_order": "q1..6, dq1..6, ddq1..6, bare/ag145/mdpg one-hot, payload_mass_kg",
                  "nominal_inertial_scale": nominal_scale, "torque_label": "MuJoCo qfrc_inverse, not motor torque",
                  "nominal_definition": "saved scene with tool/payload mass and inertia scaled; arm unchanged",
                  "scene_sha256": {name: hashlib.sha256(path.read_bytes()).hexdigest()
                                   for name, path in scenes.items()},
                  "dataset_sha256": hashlib.sha256((temporary / "dataset.npz").read_bytes()).hexdigest(),
                  "rows": {split: int((arrays["split"] == code).sum())
                           for split, code in (("train", 0), ("validation", 1),
                                               ("test", 2), ("diagnostic", 3))
                           if split in split_policy.values()},
                  "trials": trials}
        (temporary / "manifest.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                                   encoding="utf-8")
        os.rename(temporary, output)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--stride", type=int, default=25)
    parser.add_argument("--nominal-scale", type=float, default=0.8)
    parser.add_argument("--profile", choices=PROFILES, default="wide16_v3")
    args = parser.parse_args()
    report = prepare(args.source, args.output, stride=args.stride,
                     nominal_scale=args.nominal_scale, profile=args.profile)
    print(json.dumps({"output": str(args.output.resolve()), "rows": report["rows"]}, indent=2))


if __name__ == "__main__":
    main()
