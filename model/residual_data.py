"""Generate paired nominal/reference MuJoCo inverse-dynamics samples offline."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np

from .data import REPO_ROOT, TRIAL_IDS, load_trials, time_split
from .rbd import AG145_SCENE, ARM_SCENE, RBDModel, flange_parameters, make_model, tool_specs


ARRAY_FIELDS = ("q", "dq", "ddq", "tool_nominal", "tool_true", "tau_reference",
                "tau_nominal", "tau_aware", "delta_tau_target")
SPLITS = ("train", "validation", "test_new_trial", "test_unseen_trajectory",
          "test_unseen_tool", "test_mismatch_severity")
TRAIN_TOOLS = ("bare", "ag145", "virtual_light", "virtual_heavy")
SEVERITY_FACTORS = (0.6, 0.7, 0.8, 0.9, 1.0)


def new_joint_trial(home: np.ndarray, samples: int = 240, duration_s: float = 8.0):
    """Independent analytic joint motion; no ROS controller/trajectory replay."""
    time = np.linspace(0.0, duration_s, samples)
    envelope = np.sin(np.pi * time / duration_s) ** 2
    d_envelope = (np.pi / duration_s) * np.sin(2 * np.pi * time / duration_s)
    dd_envelope = (2 * np.pi**2 / duration_s**2) * np.cos(2 * np.pi * time / duration_s)
    amplitude = np.asarray((0.07, 0.05, 0.06, 0.08, 0.05, 0.10))
    frequency = np.asarray((0.23, 0.31, 0.19, 0.37, 0.29, 0.41))
    phase = np.asarray((0.0, 0.6, 1.1, 0.3, 1.7, 0.9))
    angle = time[:, None] * (2 * np.pi * frequency) + phase
    omega = 2 * np.pi * frequency
    sine, cosine = np.sin(angle), np.cos(angle)
    q = home + amplitude * envelope[:, None] * sine
    dq = amplitude * (d_envelope[:, None] * sine + envelope[:, None] * omega * cosine)
    ddq = amplitude * (dd_envelope[:, None] * sine + 2 * d_envelope[:, None] * omega * cosine
                       - envelope[:, None] * omega**2 * sine)
    return time, q, dq, ddq


def _add(chunks: dict, source: tuple, tool_name: str, split: str, nominal_factor: float,
         q: np.ndarray, dq: np.ndarray, ddq: np.ndarray, time: np.ndarray,
         reference: dict, nominal: dict, true_parameters: np.ndarray,
         nominal_parameters: np.ndarray) -> None:
    count = len(q)
    trajectory, trial = source
    values = {
        "q": q, "dq": dq, "ddq": ddq,
        "tool_nominal": np.repeat(nominal_parameters[None, :], count, axis=0),
        "tool_true": np.repeat(true_parameters[None, :], count, axis=0),
        "tau_reference": reference["torque"], "tau_nominal": nominal["torque"],
        # Tool-aware RBD uses the exact reference parameters but reconstructs
        # torque from the RBD terms independently of mj_inverse.
        "tau_aware": np.einsum("nij,nj->ni", reference["mass_matrix"], ddq)
        + reference["bias"] + reference["damping"],
        "delta_tau_target": reference["torque"] - nominal["torque"],
        "time_s": time,
        "trajectory_id": np.repeat(trajectory, count),
        "trial_id": np.repeat(trial, count),
        "tool_condition": np.repeat(tool_name, count),
        "split": np.repeat(split, count),
        "nominal_mass_factor": np.repeat(nominal_factor, count),
    }
    for key, value in values.items():
        chunks.setdefault(key, []).append(np.asarray(value))


def _csv_columns() -> list[str]:
    cols = ["time_s", "trajectory_id", "trial_id", "tool_condition", "split", "nominal_mass_factor"]
    for stem in ("q", "dq", "ddq", "tau_reference", "tau_nominal", "tau_aware", "delta_tau_target"):
        cols.extend(f"{stem}_{i}" for i in range(1, 7))
    for stem in ("tool_true", "tool_nominal"):
        cols.extend(f"{stem}_{part}" for part in
                    ("mass_kg", "com_x_m", "com_y_m", "com_z_m", "ixx_kg_m2", "iyy_kg_m2",
                     "izz_kg_m2", "ixy_kg_m2", "ixz_kg_m2", "iyz_kg_m2"))
    return cols


def _write_csv(path: Path, data: dict[str, np.ndarray]) -> None:
    columns = _csv_columns()
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        for index in range(len(data["q"])):
            writer.writerow([
                data["time_s"][index], data["trajectory_id"][index],
                data["trial_id"][index], data["tool_condition"][index],
                data["split"][index], data["nominal_mass_factor"][index],
                *(v for key in ("q", "dq", "ddq", "tau_reference", "tau_nominal",
                              "tau_aware", "delta_tau_target") for v in data[key][index]),
                *data["tool_true"][index], *data["tool_nominal"][index],
            ])


def generate_dataset(output: str | Path, stride: int = 5) -> dict:
    if stride != 5:
        raise ValueError("this study pins stride=5 to the validated AG145 training data")
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    trials = load_trials(stride=stride)
    specs = tool_specs()
    chunks: dict[str, list[np.ndarray]] = {}
    source_check = {}
    for tool_name, spec in specs.items():
        reference_model = RBDModel(make_model(spec))
        nominal_spec = spec.scaled_nominal()
        nominal_model = RBDModel(make_model(nominal_spec))
        true_parameters = flange_parameters(spec)
        nominal_parameters = flange_parameters(nominal_spec)
        for trajectory, trial in trials.items():
            reference = reference_model.evaluate_many(trial.q, trial.dq, trial.ddq)
            nominal = nominal_model.evaluate_many(trial.q, trial.dq, trial.ddq)
            if tool_name == "ag145":
                discrepancy = float(np.max(np.abs(reference["torque"] - trial.tau_inverse)))
                source_check[trajectory] = discrepancy
                if discrepancy > 1e-3:
                    raise RuntimeError(f"recorded AG145 qfrc_inverse mismatch for {trajectory}: {discrepancy}")
            if tool_name == "virtual_unseen":
                selections = [("test_unseen_tool", np.arange(trial.size))] if trajectory != "T4" else []
            elif trajectory == "T4":
                selections = [("test_unseen_trajectory", np.arange(trial.size))]
            else:
                train, validation = time_split(trial)
                selections = [("train", train), ("validation", validation)]
            for split, indices in selections:
                selected = {key: value[indices] for key, value in reference.items()}
                selected_nominal = {key: value[indices] for key, value in nominal.items()}
                _add(chunks, (trajectory, trial.folder.name), tool_name, split, 0.8,
                     trial.q[indices], trial.dq[indices], trial.ddq[indices], trial.time_s[indices],
                     selected, selected_nominal, true_parameters, nominal_parameters)
        if tool_name != "virtual_unseen":
            time, q, dq, ddq = new_joint_trial(trials["T1"].q[0])
            reference = reference_model.evaluate_many(q, dq, ddq)
            nominal = nominal_model.evaluate_many(q, dq, ddq)
            _add(chunks, ("NEW_JOINT", "synthetic_trial_001"), tool_name, "test_new_trial", 0.8,
                 q, dq, ddq, time, reference, nominal, true_parameters, nominal_parameters)

    # Test D changes only the nominal model on one held-out T4 interval.
    spec = specs["ag145"]
    trial = trials["T4"]
    indices = np.arange(0, trial.size, 5)
    reference = RBDModel(make_model(spec)).evaluate_many(trial.q[indices], trial.dq[indices], trial.ddq[indices])
    for factor in SEVERITY_FACTORS:
        nominal_spec = spec.nominal_at(factor)
        nominal = RBDModel(make_model(nominal_spec)).evaluate_many(
            trial.q[indices], trial.dq[indices], trial.ddq[indices])
        _add(chunks, ("T4", "trial_001_severity"), "ag145", "test_mismatch_severity", factor,
             trial.q[indices], trial.dq[indices], trial.ddq[indices], trial.time_s[indices],
             reference, nominal, flange_parameters(spec), flange_parameters(nominal_spec))

    data = {key: np.concatenate(values) for key, values in chunks.items()}
    if not np.allclose(data["tau_reference"] - data["tau_nominal"], data["delta_tau_target"], atol=1e-10):
        raise RuntimeError("residual target is inconsistent")
    np.savez_compressed(output / "dataset.npz", **data)
    _write_csv(output / "dataset.csv", data)
    counts = {split: int(np.count_nonzero(data["split"] == split)) for split in SPLITS}
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(), "dataset_version": "rbd_residual_v1",
        "source": "offline MuJoCo inverse dynamics at recorded AG145 states plus analytic NEW_JOINT states",
        "source_trials": {task: {"path": str(trial.folder.relative_to(REPO_ROOT)).replace("\\", "/"),
                                 "dynamics_sha256": hashlib.sha256((trial.folder / "dynamics.csv").read_bytes()).hexdigest(),
                                 "rows": trial.size} for task, trial in trials.items()},
        "scene_sha256": {"arm": hashlib.sha256(ARM_SCENE.read_bytes()).hexdigest(),
                         "ag145": hashlib.sha256(AG145_SCENE.read_bytes()).hexdigest()},
        "recorded_ag145_max_abs_inverse_discrepancy_nm": source_check,
        "mujoco_version": __import__("mujoco").__version__, "stride": stride,
        "splits": counts, "split_policy": {
            "train_validation": "T1-T3 first 80% vs final 20% of each recorded trial; T4 wholly excluded",
            "test_new_trial": "independently generated 8 s analytic six-joint trial; not ROS execution",
            "test_unseen_trajectory": "all T4 recorded states, excluded from training and early stopping",
            "test_unseen_tool": "virtual_unseen parameter combination on T1-T3 states; no training rows",
            "test_mismatch_severity": "AG145 T4 subsample, nominal mass factors 0.6..1.0",
        },
        "tool_specs": {name: {"reference": spec.metadata(), "nominal_0p8": spec.scaled_nominal().metadata(),
                              "reference_flange_10": flange_parameters(spec).tolist(),
                              "nominal_flange_10": flange_parameters(spec.scaled_nominal()).tolist()}
                       for name, spec in specs.items()},
        "training_feature_policy": "KAN sees only q,dq,ddq and nominal flange parameters; true tool parameters and reference torque are labels/audit only",
        "nominal_mismatch": "mass factor f; COM local offset (0.005,-0.005,-0.01)*(1-f)/0.2 m; inertia factor 1.25*f-0.25",
        "units": {"q": "rad", "dq": "rad/s", "ddq": "rad/s^2", "torque": "Nm",
                  "tool_mass": "kg", "tool_com": "m, link_6 frame", "tool_inertia": "kg m^2, link_6 frame",
                  "time_s": "source MuJoCo simulation time or analytic trial time"},
        "torque_definition": "MuJoCo qfrc_inverse = M(q)ddq + qfrc_bias(q,dq) + dof_damping*dq; not measured sensor torque",
        "warning": "virtual tools are synthetic inertial perturbations of the AG145 visual scene; no pneumatic gripper dynamics",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def load_dataset(folder: str | Path) -> dict[str, np.ndarray]:
    with np.load(Path(folder) / "dataset.npz") as archive:
        return {key: archive[key] for key in archive.files}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build paired MuJoCo RBD residual dataset")
    parser.add_argument("--output", default=str(Path(__file__).resolve().parent / "residual_data" / "rbd_v1"))
    parser.add_argument("--stride", type=int, default=5)
    args = parser.parse_args(argv)
    report = generate_dataset(args.output, args.stride)
    print(json.dumps({"splits": report["splits"],
                      "recorded_ag145_max_abs_inverse_discrepancy_nm": report[
                          "recorded_ag145_max_abs_inverse_discrepancy_nm"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
