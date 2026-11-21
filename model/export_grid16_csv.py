"""Export completed 16-trajectory MuJoCo trials as PKAN-ready flat CSVs.

The existing PhysicsKAN has no tool/payload input, so each condition is kept in
its own directory. Repetitions are deterministic; trial_001 is selected by
default and no random train/test split is performed.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import shutil
import tempfile


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / "data/grid_16_trajectories"
DEFAULT_OUTPUT = ROOT / "data/grid16_pkan_csv"
SPLITS = {
    **{f"T{i}": "diagnostic" for i in range(1, 5)},
    **{f"T{i}": "train" for i in range(5, 12)},
    "T12": "validation", "T13": "test", "T14": "test",
    "T15": "train", "T16": "test",
}
SPLIT_ORDER = ("train", "validation", "test", "diagnostic")
SCALARS = ("condition_id", "gripper_type", "payload_mass_kg", "trajectory_id",
           "trial_id", "time_s")
VECTORS = ("q", "dq", "ddq", "tau_inverse", "bias", "damping")
FIELDNAMES = list(SCALARS) + [f"{name}{i}" for name in VECTORS for i in range(1, 7)] \
    + [f"M{i}{j}" for i in range(1, 7) for j in range(1, 7)]


def _vector(raw: str, name: str) -> list[float]:
    value = json.loads(raw)
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError(f"{name} must have six values")
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} has nonfinite values")
    return result


def _matrix(raw: str) -> list[list[float]]:
    value = json.loads(raw)
    if not isinstance(value, list) or len(value) != 6:
        raise ValueError("mass_matrix must be 6x6")
    result = [_vector(json.dumps(row), "mass_matrix row") for row in value]
    if any(abs(result[i][j] - result[j][i]) > 1e-6 for i in range(6) for j in range(6)):
        raise ValueError("mass_matrix is not symmetric")
    return result


def _metadata(folder: Path, condition: str, trajectory: str) -> dict | None:
    path = folder / "metadata.json"
    if not path.is_file() or not (folder / "dynamics.csv").is_file():
        return None
    meta = json.loads(path.read_text(encoding="utf-8"))
    if (meta.get("mode") != "simulation" or meta.get("condition_id") != condition
            or meta.get("trajectory_id") != trajectory or meta.get("trial_id") != folder.name
            or meta.get("dataset_split") != SPLITS[trajectory]):
        raise ValueError(f"metadata identity or split mismatch: {folder}")
    if meta.get("joint_order") != [f"joint_{i}" for i in range(1, 7)]:
        raise ValueError(f"joint order mismatch: {folder}")
    units = meta.get("units", {})
    if tuple(units.get(name) for name in ("q", "dq", "ddq", "torque")) != ("rad", "rad/s", "rad/s^2", "Nm"):
        raise ValueError(f"unit mismatch: {folder}")
    report = meta.get("report", {})
    if (report.get("max_tracking_error_rad", 1) > 0.05
            or report.get("torque_limit_samples") != 0
            or report.get("cross_robot_contacts") != 0):
        raise ValueError(f"failed simulation quality check: {folder}")
    bag = meta.get("rosbag_relative_path")
    if not bag or not (folder.parents[3] / bag / "metadata.yaml").is_file():
        raise ValueError(f"ROS Bag completion marker missing: {folder}")
    if meta.get("quality", {}).get("joint_states", {}).get("estimated_missing_rate", 1) > 0.01:
        raise ValueError(f"joint-state gaps exceed 1%: {folder}")
    return meta


def _select_trials(source: Path, condition: str, all_repeats: bool) -> list[tuple[Path, dict]] | None:
    selected: list[tuple[Path, dict]] = []
    for trajectory in SPLITS:
        base = source / "simulation" / trajectory / condition
        folders = sorted(p for p in base.glob("trial_[0-9][0-9][0-9]") if p.is_dir())
        valid = [(folder, meta) for folder in folders
                 if (meta := _metadata(folder, condition, trajectory)) is not None]
        if not valid:
            return None  # A condition enters the export only with all 16 paths complete.
        selected.extend(valid if all_repeats else valid[:1])
    return selected


def _flat_row(row: dict, meta: dict) -> list:
    q, dq, ddq = ([float(row[f"{stem}{i}"]) for i in range(1, 7)]
                  for stem in ("q", "dq", "ddq"))
    tau = _vector(row["qfrc_inverse"], "qfrc_inverse")
    bias = _vector(row["qfrc_bias"], "qfrc_bias")
    damping = _vector(row["dof_damping"], "dof_damping")
    mass = _matrix(row["mass_matrix"])
    stamp = float(row["ros_time_s"])
    if not all(math.isfinite(v) for v in (*q, *dq, *ddq, stamp)):
        raise ValueError("nonfinite state or time")
    error = max(abs(sum(mass[i][j] * ddq[j] for j in range(6))
                    + bias[i] + damping[i] * dq[i] - tau[i]) for i in range(6))
    if error > 2e-5:
        raise ValueError(f"inverse dynamics identity mismatch: {error:.6g} Nm")
    return [meta["condition_id"], meta["gripper_type"], meta["payload_mass_kg"],
            meta["trajectory_id"], meta["trial_id"], stamp,
            *q, *dq, *ddq, *tau, *bias, *damping,
            *(mass[i][j] for i in range(6) for j in range(6))]


def export(source: Path, output: Path, stride: int = 5,
           conditions: list[str] | None = None, all_repeats: bool = False) -> dict:
    source, output = source.resolve(), output.resolve()
    if stride < 1:
        raise ValueError("stride must be positive")
    if output.exists():
        raise FileExistsError(f"output already exists; choose a fresh --output path: {output}")
    if source == output or source in output.parents:
        raise ValueError("output must be outside the source campaign")
    available = sorted(p.name for p in (source / "simulation" / "T1").iterdir() if p.is_dir())
    chosen = conditions or available
    if not chosen or any(item not in available for item in chosen):
        raise ValueError(f"condition must be one of: {available}")
    selected = {condition: _select_trials(source, condition, all_repeats) for condition in chosen}
    selected = {condition: files for condition, files in selected.items() if files is not None}
    if not selected:
        raise ValueError("no condition has a complete T1–T16 set")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix=".grid16_pkan_", dir=output.parent))
    report = {
        "schema_version": "1.0", "created_utc": datetime.now(timezone.utc).isoformat(),
        "source": str(source), "stride": stride, "source_hz": 100,
        "selected_repeat_policy": "all" if all_repeats else "first completed trial per trajectory",
        "split_policy": {split: [t for t, assigned in SPLITS.items() if assigned == split]
                         for split in SPLIT_ORDER},
        "columns": FIELDNAMES, "conditions": {},
        "torque_label": "MuJoCo qfrc_inverse at observed q,dq,ddq; reconstructed, not a sensor measurement",
        "identity": "tau_inverse = M(q) @ ddq + bias(q,dq) + damping * dq",
        "units": {"q": "rad", "dq": "rad/s", "ddq": "rad/s^2", "tau_inverse": "Nm",
                  "bias": "Nm", "damping": "Nm s/rad", "M": "kg m^2", "time_s": "simulation seconds"},
        "caveats": ["Conditions require separate models unless tool/payload features are added to PhysicsKAN",
                    "three repeats use identical simulation setup and are not independent state coverage",
                    "T16 may be a legacy chirp recording while its source design is changing"]
    }
    try:
        for condition, trials in selected.items():
            directory = temporary / condition
            directory.mkdir()
            handles = {split: (directory / f"{split}.csv").open("w", encoding="utf-8", newline="")
                       for split in SPLIT_ORDER}
            try:
                writers = {split: csv.writer(stream) for split, stream in handles.items()}
                for writer in writers.values():
                    writer.writerow(FIELDNAMES)
                condition_report = {"rows": {split: 0 for split in SPLIT_ORDER}, "trials": []}
                for folder, meta in trials:
                    trajectory = meta["trajectory_id"]
                    writer = writers[SPLITS[trajectory]]
                    source_rows = used_rows = 0
                    previous_time = -math.inf
                    with (folder / "dynamics.csv").open(encoding="utf-8", newline="") as stream:
                        for source_rows, row in enumerate(csv.DictReader(stream), start=1):
                            if (row["trajectory_id"] != trajectory or row["trial_id"] != meta["trial_id"]
                                    or row["mode"] != "simulation" or row["gripper_type"] != meta["gripper_type"]):
                                raise ValueError(f"CSV identity mismatch: {folder}, row {source_rows}")
                            if (source_rows - 1) % stride:
                                continue
                            flat = _flat_row(row, meta)
                            if flat[5] <= previous_time:
                                raise ValueError(f"nonmonotonic simulation time: {folder}")
                            previous_time = flat[5]
                            writer.writerow(flat)
                            used_rows += 1
                    if source_rows != meta["quality"]["rows_written"] or not used_rows:
                        raise ValueError(f"row count mismatch: {folder}")
                    condition_report["rows"][SPLITS[trajectory]] += used_rows
                    condition_report["trials"].append({
                        "trajectory_id": trajectory, "trial_id": meta["trial_id"],
                        "source": str(folder.relative_to(source)).replace("\\", "/"),
                        "source_rows": source_rows, "selected_rows": used_rows,
                        "time_scale": meta.get("grid_time_scale"),
                    })
                report["conditions"][condition] = condition_report
            finally:
                for stream in handles.values():
                    stream.close()
        (temporary / "manifest.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        os.rename(temporary, output)
    except BaseException:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--stride", type=int, default=5, help="every fifth 100 Hz row = 20 Hz")
    parser.add_argument("--condition", action="append", help="restrict to one or more completed conditions")
    parser.add_argument("--all-repeats", action="store_true", help="include deterministic repeats in the same split")
    args = parser.parse_args()
    result = export(args.source, args.output, args.stride, args.condition, args.all_repeats)
    print(json.dumps({"output": str(args.output.resolve()),
                      "conditions": {key: value["rows"] for key, value in result["conditions"].items()}},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
