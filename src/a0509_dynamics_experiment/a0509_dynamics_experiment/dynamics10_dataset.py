"""Audit and visualize T5-T14 data without changing any learning model."""

from __future__ import annotations

import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import subprocess
import xml.etree.ElementTree as ET

from .dynamics10_safety import ACCELERATION_LIMITS, POSITION_LIMITS, VELOCITY_LIMITS
from .trajectory_generator import DYNAMICS10_IDS, EXCITATIONS, excitation_metadata


def _read(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _vector(row: dict, stem: str) -> list[float] | None:
    try:
        if stem == "qfrc_inverse":
            values = json.loads(row[stem])
        else:
            values = [float(row[f"{stem}{i}"]) for i in range(1, 7)]
        if len(values) != 6 or not all(math.isfinite(float(v)) for v in values):
            return None
        return [float(v) for v in values]
    except (ValueError, TypeError, KeyError, json.JSONDecodeError):
        return None


def _stats(vectors: list[list[float]]) -> dict | None:
    if not vectors:
        return None
    return {"min": [min(row[i] for row in vectors) for i in range(6)],
            "max": [max(row[i] for row in vectors) for i in range(6)],
            "rms": [math.sqrt(sum(row[i]**2 for row in vectors) / len(vectors)) for i in range(6)],
            "abs_max": [max(abs(row[i]) for row in vectors) for i in range(6)]}


def _trial_quality(folder: Path) -> dict:
    rows = _read(folder / "dynamics.csv")
    planned = _read(folder / "trajectory.csv")
    vectors = {stem: [_vector(row, stem) for row in rows]
               for stem in ("q", "dq", "ddq", "target_q", "qfrc_inverse")}
    valid = {stem: [v for v in values if v is not None] for stem, values in vectors.items()}
    stamps = [float(row["ros_time_s"]) for row in rows]
    monotonic = all(b > a for a, b in zip(stamps, stamps[1:]))
    intervals = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
    missing = {stem: len(rows) - len(values) for stem, values in valid.items()}
    tracking = [max(abs(q[i] - target[i]) for i in range(6))
                for q, target in zip(vectors["q"], vectors["target_q"])
                if q is not None and target is not None]
    violations = {"position": 0, "velocity": 0, "acceleration": 0}
    for stem, limits in (("q", POSITION_LIMITS), ("dq", VELOCITY_LIMITS),
                         ("ddq", ACCELERATION_LIMITS)):
        for vector in valid[stem]:
            for i, value in enumerate(vector):
                violated = (not limits[i][0] <= value <= limits[i][1] if stem == "q"
                            else abs(value) > limits[i])
                violations[{"q": "position", "dq": "velocity", "ddq": "acceleration"}[stem]] += int(violated)
    phase_count = {"pre_hold": 0, "motion": 0, "post_hold": 0}
    for row in planned:
        phase = row["phase"]
        phase_count[phase if phase in phase_count else "motion"] += 1
    quality = {"samples": len(rows), "planned_samples": len(planned),
               "complete_joint_and_torque_samples": sum(all(vectors[key][i] is not None
                  for key in ("q", "dq", "ddq", "qfrc_inverse")) for i in range(len(rows))),
               "missing_or_nonfinite_counts": missing,
               "sim_time_strictly_increasing": monotonic,
               "sim_time_dt_median_s": statistics.median(intervals) if intervals else None,
               "sim_time_dt_max_s": max(intervals) if intervals else None,
               "observed_hz": (len(rows)-1)/(stamps[-1]-stamps[0]) if len(rows)>1 and stamps[-1]>stamps[0] else None,
               "target_tracking_abs_max_rad": max(tracking) if tracking else None,
               "target_tracking_rmse_rad": math.sqrt(sum(v*v for v in tracking)/len(tracking)) if tracking else None,
               "position_rad": _stats(valid["q"]), "velocity_rad_s": _stats(valid["dq"]),
               "acceleration_rad_s2": _stats(valid["ddq"]),
               "inverse_torque_nm": _stats(valid["qfrc_inverse"]),
               "limit_violation_counts": violations, "planned_phase_samples": phase_count,
               "qfrc_inverse_semantics": "reconstructed MuJoCo inverse dynamics, not a physical torque sensor"}
    quality["pass"] = (len(rows) >= .9 * len(planned) and monotonic and
                       len(valid["q"]) >= .98*len(rows) and
                       len(valid["dq"]) >= .98*len(rows) and
                       len(valid["ddq"]) >= .95*len(rows) and
                       len(valid["qfrc_inverse"]) >= .95*len(rows) and
                       not any(violations.values()) and
                       quality["target_tracking_abs_max_rad"] is not None and
                       quality["target_tracking_abs_max_rad"] < .15)
    return quality


def _latest(root: Path, mode: str, trajectory_id: str) -> Path | None:
    parent = root / mode / trajectory_id
    if not parent.exists():
        return None
    folders = sorted(p for p in parent.iterdir() if p.is_dir() and (p / "metadata.json").is_file())
    return folders[-1] if folders else None


def _legacy_ranges(root: Path) -> dict:
    repo = root.resolve()
    while repo != repo.parent and not (repo / "results" / "ag145" / "simulation").is_dir():
        repo = repo.parent
    vectors = {key: [] for key in ("q", "dq", "ddq")}
    for trajectory, trial in (("T1", "trial_005"), ("T2", "trial_001"), ("T3", "trial_001")):
        path = repo / "results" / "ag145" / "simulation" / trajectory / trial / "dynamics.csv"
        if path.is_file():
            for row in _read(path)[::5]:
                for stem in vectors:
                    value = _vector(row, stem)
                    if value is not None:
                        vectors[stem].append(value)
    return {stem: _stats(items) for stem, items in vectors.items()}


def _coverage(root: Path, entries: dict) -> dict:
    import numpy as np
    legacy = _legacy_ranges(root)
    current = {stem: [] for stem in ("q", "dq", "ddq")}
    for entry in entries.values():
        path = entry.get("simulation_path")
        if path and entry.get("quality", {}).get("pass"):
            for row in _read(Path(path) / "dynamics.csv"):
                for stem in current:
                    value = _vector(row, stem)
                    if value is not None:
                        current[stem].append(value)
    new = {stem: _stats(items) for stem, items in current.items()}
    added = {}
    for stem in ("q", "dq"):
        if legacy[stem] and new[stem]:
            added[stem] = [{"below_legacy": max(0, legacy[stem]["min"][i]-new[stem]["min"][i]),
                            "above_legacy": max(0, new[stem]["max"][i]-legacy[stem]["max"][i])}
                           for i in range(6)]
    # A 12D q/dq distance supplements per-joint ranges. Downsampling avoids
    # counting adjacent 100 Hz samples as independent new states.
    old_states = []
    repo = root.resolve()
    while repo != repo.parent and not (repo / "results" / "ag145" / "simulation").is_dir():
        repo = repo.parent
    for trajectory, trial in (("T1", "trial_005"), ("T2", "trial_001"), ("T3", "trial_001")):
        path = repo / "results" / "ag145" / "simulation" / trajectory / trial / "dynamics.csv"
        if path.is_file():
            for row in _read(path)[::5]:
                q, dq = _vector(row, "q"), _vector(row, "dq")
                if q is not None and dq is not None:
                    old_states.append(q+dq)
    new_states = []
    for entry in entries.values():
        path = entry.get("simulation_path")
        if path and entry.get("quality", {}).get("pass"):
            for row in _read(Path(path) / "dynamics.csv")[::5]:
                q, dq = _vector(row, "q"), _vector(row, "dq")
                if q is not None and dq is not None:
                    new_states.append(q+dq)
    distance = None
    if old_states and new_states:
        old = np.asarray(old_states)
        recent = np.asarray(new_states)
        scale = np.maximum(old.std(axis=0), np.asarray([.05]*12))
        old_scaled = old / scale
        nearest = []
        for start in range(0, len(recent), 64):
            chunk = recent[start:start+64] / scale
            sq = np.sum((chunk[:, None, :] - old_scaled[None, :, :])**2, axis=2)
            nearest.extend(np.sqrt(np.min(sq, axis=1)).tolist())
        distance = {"old_samples": len(old), "new_samples": len(recent),
                    "median": float(np.median(nearest)), "p95": float(np.percentile(nearest, 95)),
                    "scale_floor_per_dimension": .05,
                    "definition": "Euclidean nearest distance in q(rad)+dq(rad/s), each dimension scaled by max(legacy std,0.05); 5-row stride"}
    return {"legacy_T1_T3": legacy, "new_T5_T14": new, "range_extension_by_joint": added,
            "nearest_legacy_distance_12d": distance,
            "interpretation": "q/dq 1D bounds and standardized 12D proximity only; not full 18D coverage. Legacy ddq used wall-time differencing and new ddq uses filtered simulation time, so acceleration ranges are not directly comparable."}


def _trajectory_similarity(root: Path, entries: dict) -> dict:
    """Pairwise 6D planned position RMS after normalizing motion duration."""
    import numpy as np
    curves = {}
    for trajectory, entry in entries.items():
        path = entry.get("preview_path") or entry.get("simulation_path")
        if not path:
            continue
        rows = [row for row in _read(Path(path) / "trajectory.csv")
                if row["phase"].startswith("motion")]
        if not rows:
            continue
        q = np.asarray([[float(row[f"q{i}_rad"]) for i in range(1, 7)] for row in rows])
        x = np.linspace(0.0, 1.0, len(q))
        curves[trajectory] = np.stack([np.interp(np.linspace(0, 1, 201), x, q[:, i])
                                       for i in range(6)], axis=1)
    pairs = []
    for i, a in enumerate(curves):
        for b in list(curves)[i+1:]:
            rms = float(np.sqrt(np.mean((curves[a]-curves[b])**2)))
            pairs.append({"a": a, "b": b, "joint_position_rms_rad": rms})
    pairs.sort(key=lambda item: item["joint_position_rms_rad"])
    return {"closest_pairs": pairs[:10],
            "near_duplicate_pairs_under_0p01_rad_rms": [p for p in pairs if p["joint_position_rms_rad"] < .01],
            "metric": "6-joint planned position RMS after resampling motion to 201 normalized times"}


def _model_parameters(scene: Path) -> dict | None:
    if not scene.is_file():
        return None
    root = ET.parse(scene).getroot()
    body = root.find(".//body[@name='ag145_base_link']")
    if body is None:
        return None
    inertial = body.find("inertial")
    tcp = body.find("site[@name='ag145_tcp']")
    return {"mount_pos_m": [float(x) for x in body.attrib["pos"].split()],
            "mount_quat_wxyz": [float(x) for x in body.attrib["quat"].split()],
            "mass_kg": float(inertial.attrib["mass"]),
            "com_local_m": [float(x) for x in inertial.attrib["pos"].split()],
            "diagonal_inertia_local_kg_m2": [float(x) for x in inertial.attrib["diaginertia"].split()],
            "tcp_local_m": [float(x) for x in tcp.attrib["pos"].split()] if tcp is not None else None,
            "calibrated": False}


def _figures(root: Path, entries: dict, coverage: dict) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    figures = root / "figures"
    figures.mkdir(exist_ok=True)
    fig, axes = plt.subplots(5, 2, figsize=(14, 16))
    for ax, trajectory in zip(axes.flat, DYNAMICS10_IDS):
        path = entries[trajectory].get("simulation_path") or entries[trajectory].get("preview_path")
        if path:
            rows = _read(Path(path) / "trajectory.csv")
            t = [float(row["time_s"]) for row in rows]
            for i in range(1, 7):
                ax.plot(t, [math.degrees(float(row[f"q{i}_rad"])) for row in rows],
                        label=f"J{i}", linewidth=.85)
        ax.set_title(f"{trajectory}: {EXCITATIONS[trajectory]['name']}")
        ax.set_xlabel("Time (s)"); ax.set_ylabel("Joint angle (deg)"); ax.grid(alpha=.25)
    axes[0, 0].legend(ncol=6, fontsize=7)
    fig.tight_layout(); fig.savefig(figures / "trajectory_overview.png", dpi=130); plt.close(fig)

    if not coverage["new_T5_T14"]["q"] or not coverage["legacy_T1_T3"]["q"]:
        return
    fig, axes = plt.subplots(3, 1, figsize=(12, 9))
    for ax, stem, unit in zip(axes, ("q", "dq", "ddq"), ("rad", "rad/s", "rad/s²")):
        old, new = coverage["legacy_T1_T3"][stem], coverage["new_T5_T14"][stem]
        x = np.arange(6)
        ax.vlines(x-.12, old["min"], old["max"], color="gray", linewidth=7, label="Legacy T1-T3")
        ax.vlines(x+.12, new["min"], new["max"], color="tab:blue", linewidth=7, label="New T5-T14")
        ax.set_xticks(x, [f"J{i}" for i in range(1,7)]); ax.set_ylabel(unit)
        ax.set_title(f"{stem} range" + (" (different estimators)" if stem == "ddq" else ""))
        ax.grid(alpha=.2); ax.legend()
    fig.tight_layout(); fig.savefig(figures / "joint_excitation_coverage.png", dpi=140); plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    for ax, stem, unit in zip(axes, ("dq", "ddq"), ("rad/s", "rad/s²")):
        old, new = coverage["legacy_T1_T3"][stem], coverage["new_T5_T14"][stem]
        x = np.arange(6)
        ax.bar(x-.18, old["rms"], .36, label="Legacy T1-T3", color="gray")
        ax.bar(x+.18, new["rms"], .36, label="New T5-T14", color="tab:blue")
        ax.set_xticks(x, [f"J{i}" for i in range(1,7)]); ax.set_ylabel(f"RMS ({unit})")
        if stem == "ddq":
            ax.set_title("Acceleration estimators differ: legacy wall time / new sim time")
        ax.grid(axis="y", alpha=.25); ax.legend()
    fig.tight_layout(); fig.savefig(figures / "velocity_acceleration_distribution.png", dpi=140); plt.close(fig)

    fig, axes = plt.subplots(5, 2, figsize=(14, 15))
    for ax, trajectory in zip(axes.flat, DYNAMICS10_IDS):
        path = entries[trajectory].get("simulation_path")
        if path and entries[trajectory].get("quality", {}).get("pass"):
            rows = _read(Path(path) / "dynamics.csv")
            t0 = float(rows[0]["ros_time_s"])
            for i in range(6):
                series = [(float(row["ros_time_s"])-t0, _vector(row,"qfrc_inverse")) for row in rows]
                ax.plot([t for t,v in series if v is not None], [v[i] for t,v in series if v is not None],
                        linewidth=.8, label=f"J{i+1}")
        ax.set_title(trajectory); ax.set_xlabel("Simulation time (s)"); ax.set_ylabel("Inverse torque (Nm)")
        ax.grid(alpha=.25)
    axes[0,0].legend(ncol=6, fontsize=7)
    fig.tight_layout(); fig.savefig(figures / "torque_profiles.png", dpi=130); plt.close(fig)


def summarize(root: str | Path, outcomes: dict | None = None) -> dict:
    root = Path(root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    outcomes = outcomes or {}
    try:
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root,
                                         stderr=subprocess.DEVNULL, text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        commit = None
    entries = {}
    for trajectory in DYNAMICS10_IDS:
        entry = {"split": EXCITATIONS[trajectory]["split"],
                 "definition": excitation_metadata(trajectory),
                 "latest_attempt": outcomes.get(trajectory)}
        for mode in ("preview", "simulation"):
            folder = _latest(root, mode, trajectory)
            if folder:
                entry[f"{mode}_path"] = str(folder)
                entry[f"{mode}_relative_path"] = folder.relative_to(root).as_posix()
                if mode == "simulation" and (folder / "dynamics.csv").is_file():
                    quality = _trial_quality(folder)
                    entry["quality"] = quality
                    metadata_path = folder / "metadata.json"
                    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
                    metadata["data_quality"] = quality
                    metadata["git_commit_at_audit"] = commit
                    metadata["ag145_model_parameters"] = _model_parameters(folder / "ag145_scene.xml")
                    metadata["dataset_split"] = entry["split"]
                    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
        entries[trajectory] = entry
    coverage = _coverage(root, entries)
    similarity = _trajectory_similarity(root, entries)
    report = {"created_utc": datetime.now(timezone.utc).isoformat(),
              "git_commit_at_audit": commit, "dataset_name": "dynamics10",
              "splits": {"train": list(DYNAMICS10_IDS[:7]), "validation": ["T12"],
                         "test": ["T13", "T14"], "legacy_diagnostic": ["T4"]},
              "status": entries, "coverage": coverage,
              "trajectory_similarity": similarity,
              "note": "Only quality-pass ROS/MuJoCo simulation trials count as collected data; preview is planned motion only."}
    (root / "dataset_manifest.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    with (root / "dataset_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["trajectory_id", "split", "preview", "simulation", "quality_pass", "samples",
                         "complete_samples", "observed_hz", "max_tracking_error_rad", "missing_ddq",
                         "position_min_rad", "position_max_rad", "velocity_rms_rad_s",
                         "velocity_abs_max_rad_s", "acceleration_rms_rad_s2", "acceleration_abs_max_rad_s2",
                         "inverse_torque_rms_nm", "inverse_torque_abs_max_nm", "failure"])
        for trajectory, entry in entries.items():
            q = entry.get("quality", {})
            writer.writerow([trajectory, entry["split"], bool(entry.get("preview_path")),
                             bool(entry.get("simulation_path")), q.get("pass", False), q.get("samples",0),
                             q.get("complete_joint_and_torque_samples",0), q.get("observed_hz"),
                             q.get("target_tracking_abs_max_rad"), q.get("missing_or_nonfinite_counts",{}).get("ddq"),
                             json.dumps(q.get("position_rad",{}).get("min")),
                             json.dumps(q.get("position_rad",{}).get("max")),
                             json.dumps(q.get("velocity_rad_s",{}).get("rms")),
                             json.dumps(q.get("velocity_rad_s",{}).get("abs_max")),
                             json.dumps(q.get("acceleration_rad_s2",{}).get("rms")),
                             json.dumps(q.get("acceleration_rad_s2",{}).get("abs_max")),
                             json.dumps(q.get("inverse_torque_nm",{}).get("rms")),
                             json.dumps(q.get("inverse_torque_nm",{}).get("abs_max")),
                             (entry.get("latest_attempt") or {}).get("error")])
    _figures(root, entries, coverage)
    return report
