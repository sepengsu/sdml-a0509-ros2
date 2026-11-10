"""Build a trajectory-disjoint residual-RBD dataset from recorded AG145 trials.

Only ROS/MuJoCo simulation observations are used. T12 is validation, T13/T14
are untouched tests, and legacy T4 is a diagnostic rather than a final test.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np

from .data import REPO_ROOT, load_trials
from .rbd import RBDModel, flange_parameters, make_model, tool_specs
from .residual_data import _write_csv


TRAJECTORIES = tuple(f"T{i}" for i in range(5, 15))
SPLIT = {**{f"T{i}": "train" for i in range(5, 12)},
         "T12": "validation", "T13": "test", "T14": "test", "T4": "diagnostic"}
NUMERIC_FIELDS = ("q", "dq", "ddq", "tau_reference", "tau_aware")


def _six(row: dict, stem: str) -> np.ndarray:
    result = np.asarray([float(row[f"{stem}{i}"]) for i in range(1, 7)], dtype=np.float64)
    if not np.isfinite(result).all():
        raise ValueError(f"nonfinite {stem}")
    return result


def _json_array(row: dict, name: str, shape: tuple[int, ...]) -> np.ndarray:
    result = np.asarray(json.loads(row[name]), dtype=np.float64)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"invalid {name}: expected {shape}")
    return result


def load_recorded_trial(folder: Path, trajectory: str, stride: int = 5) -> dict:
    if stride < 1:
        raise ValueError("stride must be positive")
    meta = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
    if (meta["trajectory_id"] != trajectory or meta["trial_id"] != folder.name or
            meta["mode"] != "simulation" or meta["gripper_type"] != "ag145" or
            not meta["data_quality"]["pass"]):
        raise ValueError(f"trial is not a validated AG145 simulation: {folder}")
    if meta["joint_order"] != [f"joint_{i}" for i in range(1, 7)]:
        raise ValueError("joint order changed")
    if (meta["units"]["q"], meta["units"]["dq"], meta["units"]["ddq"],
            meta["units"]["torque"]) != ("rad", "rad/s", "rad/s^2", "Nm"):
        raise ValueError("incompatible units")
    tool = meta["ag145_model_parameters"]
    if (tool["mass_kg"] != 1.3 or tool["diagonal_inertia_local_kg_m2"] !=
            [0.0073, 0.005, 0.0033]):
        raise ValueError("AG145 model parameters changed")
    values: dict[str, list] = {key: [] for key in (*NUMERIC_FIELDS, "time_s", "target_q")}
    source_rows = 0
    with (folder / "dynamics.csv").open(newline="", encoding="utf-8") as stream:
        for source_rows, row in enumerate(csv.DictReader(stream), start=1):
            if (row["trajectory_id"] != trajectory or row["trial_id"] != folder.name or
                    row["mode"] != "simulation" or row["gripper_type"] != "ag145"):
                raise ValueError(f"CSV identity mismatch in {folder} row {source_rows}")
            if (source_rows - 1) % stride:
                continue
            q, dq, ddq = (_six(row, stem) for stem in ("q", "dq", "ddq"))
            target = _six(row, "target_q")
            reference = _json_array(row, "qfrc_inverse", (6,))
            mass = _json_array(row, "mass_matrix", (6, 6))
            bias = _json_array(row, "qfrc_bias", (6,))
            damping = _json_array(row, "dof_damping", (6,))
            aware = mass @ ddq + bias + damping * dq
            if not np.allclose(aware, reference, atol=2e-5, rtol=1e-6):
                raise ValueError(f"stored RBD does not reconstruct inverse torque: {folder}")
            stamp = float(row["ros_time_s"])
            if not np.isfinite(stamp) or (values["time_s"] and stamp <= values["time_s"][-1]):
                raise ValueError(f"nonmonotonic simulation time: {folder}")
            for key, value in (("q", q), ("dq", dq), ("ddq", ddq),
                               ("target_q", target), ("tau_reference", reference),
                               ("tau_aware", aware), ("time_s", stamp)):
                values[key].append(value)
    if source_rows != meta["data_quality"]["samples"] or not values["q"]:
        raise ValueError("source sample count changed")
    arrays = {key: np.asarray(value) for key, value in values.items()}
    arrays["time_s"] -= arrays["time_s"][0]
    return {"arrays": arrays, "metadata": meta, "source_rows": source_rows}


def generate_dataset(source_root: str | Path, output: str | Path, stride: int = 5) -> dict:
    source_root = Path(source_root).resolve()
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    source_manifest_path = source_root / "dataset_manifest.json"
    source_manifest = json.loads(source_manifest_path.read_text(encoding="utf-8"))
    if source_manifest["splits"] != {"train": list(TRAJECTORIES[:7]),
                                    "validation": ["T12"], "test": ["T13", "T14"],
                                    "legacy_diagnostic": ["T4"]}:
        raise ValueError("trajectory split policy changed")
    nominal_spec = tool_specs()["ag145"].scaled_nominal()
    nominal_model = RBDModel(make_model(nominal_spec))
    nominal_tool = flange_parameters(nominal_spec)
    true_tool = flange_parameters(tool_specs()["ag145"])
    parts: dict[str, list[np.ndarray]] = {}
    provenance = {}

    def add(trajectory: str, trial_id: str, arrays: dict, folder: Path, source_rows: int):
        count = len(arrays["q"])
        nominal = nominal_model.evaluate_many(arrays["q"], arrays["dq"], arrays["ddq"])["torque"]
        values = {"q": arrays["q"], "dq": arrays["dq"], "ddq": arrays["ddq"],
                  "tool_nominal": np.tile(nominal_tool, (count, 1)),
                  "tool_true": np.tile(true_tool, (count, 1)),
                  "tau_reference": arrays["tau_reference"], "tau_nominal": nominal,
                  "tau_aware": arrays["tau_aware"],
                  "delta_tau_target": arrays["tau_reference"] - nominal,
                  "time_s": arrays["time_s"],
                  "trajectory_id": np.repeat(trajectory, count),
                  "trial_id": np.repeat(trial_id, count),
                  "tool_condition": np.repeat("ag145", count),
                  "split": np.repeat(SPLIT[trajectory], count),
                  "nominal_mass_factor": np.repeat(.8, count)}
        for key, value in values.items():
            parts.setdefault(key, []).append(np.asarray(value))
        provenance[trajectory] = {"folder": str(folder.relative_to(REPO_ROOT)).replace("\\", "/"),
                                  "dynamics_sha256": hashlib.sha256((folder / "dynamics.csv").read_bytes()).hexdigest(),
                                  "source_rows": source_rows, "selected_rows": count,
                                  "split": SPLIT[trajectory], "trial_id": trial_id,
                                  "max_tool_aware_difference_nm": float(np.max(np.abs(arrays["tau_aware"]-arrays["tau_reference"])))}

    for trajectory in TRAJECTORIES:
        entry = source_manifest["status"][trajectory]
        if not entry["quality"]["pass"]:
            raise ValueError(f"source quality failed for {trajectory}")
        folder = source_root / entry["simulation_relative_path"]
        loaded = load_recorded_trial(folder, trajectory, stride)
        add(trajectory, folder.name, loaded["arrays"], folder, loaded["source_rows"])

    # Earlier T4 has already influenced model selection in the prior study.
    # Keep it exclusively as a legacy diagnostic and retain its provenance.
    legacy = load_trials(stride=stride)["T4"]
    reference_model = RBDModel(make_model(tool_specs()["ag145"]))
    aware = reference_model.evaluate_many(legacy.q, legacy.dq, legacy.ddq)
    if not np.allclose(aware["torque"], legacy.tau_inverse, atol=2e-5):
        raise ValueError("legacy T4 reference changed")
    legacy_arrays = {"q": legacy.q, "dq": legacy.dq, "ddq": legacy.ddq,
                     "tau_reference": legacy.tau_inverse,
                     "tau_aware": np.einsum("nij,nj->ni", aware["mass_matrix"], legacy.ddq)
                     + aware["bias"] + aware["damping"],
                     "time_s": legacy.time_s - legacy.time_s[0]}
    add("T4", legacy.folder.name, legacy_arrays, legacy.folder, legacy.source_rows)

    data = {key: np.concatenate(values) for key, values in parts.items()}
    np.savez_compressed(output / "dataset.npz", **data)
    _write_csv(output / "dataset.csv", data)
    counts = {name: int(np.count_nonzero(data["split"] == name))
              for name in ("train", "validation", "test", "diagnostic")}
    report = {"created_utc": datetime.now(timezone.utc).isoformat(),
              "dataset_version": "dynamics10_residual_v1", "stride": stride,
              "counts": counts, "source_manifest_sha256": hashlib.sha256(source_manifest_path.read_bytes()).hexdigest(),
              "sources": provenance, "reference_definition": "recorded AG145 MuJoCo qfrc_inverse reconstructed at observed q,dq,ddq",
              "nominal_definition": "same q,dq,ddq with AG145 0.8x mass, 0.75x principal inertia and (+.005,-.005,-.01) m COM offset",
              "tool_parameter_policy": "all rows are fixed AG145; nominal tool estimate only is a model input; true parameters are audit labels",
              "tool_nominal_flange": nominal_tool.tolist(), "tool_true_flange": true_tool.tolist(),
              "units": {"q": "rad", "dq": "rad/s", "ddq": "rad/s^2", "torque": "Nm", "time_s": "ROS simulation seconds relative to trial start"},
              "split_policy": "T5-T11 train; T12 validation/early stopping; T13-T14 untouched test; older T4 diagnostic only",
              "caveats": ["single AG145 tool model and one latest trial per trajectory",
                          "subsampling every fifth 100 Hz row reduces adjacent correlation but not trial dependence",
                          "reference torque is reconstructed MuJoCo inverse dynamics, not a real sensor",
                          "legacy T4 acceleration used a different estimator and is not comparable as an independent test"]}
    (output / "manifest.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build dynamics10 AG145 residual training data")
    parser.add_argument("--source", default=str(REPO_ROOT / "results" / "dynamics10"))
    parser.add_argument("--output", default=str(REPO_ROOT / "model" / "residual_data" / "dynamics10_v1"))
    parser.add_argument("--stride", type=int, default=5)
    args = parser.parse_args(argv)
    report = generate_dataset(args.source, args.output, args.stride)
    print(json.dumps({"counts": report["counts"], "source_trials": list(report["sources"])}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
