"""Evaluate nominal RBD, residual KAN, tool-aware RBD and optional MLP."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from .residual_data import SPLITS, load_dataset
from .train_residual import load_trained


def _infer(model, data: dict[str, np.ndarray]) -> np.ndarray:
    result = []
    with torch.no_grad():
        for start in range(0, len(data["q"]), 256):
            part = slice(start, start + 256)
            tensors = [torch.as_tensor(data[key][part], dtype=torch.float32)
                       for key in ("q", "dq", "ddq", "tool_nominal")]
            result.append(model(*tensors).numpy().astype(np.float64))
    return np.concatenate(result)


def _metrics(reference: np.ndarray, predicted: np.ndarray) -> dict:
    error = predicted - reference
    return {"rmse_nm": float(np.sqrt(np.mean(error**2))),
            "mae_nm": float(np.mean(np.abs(error))),
            "max_abs_error_nm": float(np.max(np.abs(error))),
            "joint_rmse_nm": np.sqrt(np.mean(error**2, axis=0)).tolist()}


def evaluate(dataset_folder: str | Path, run_folder: str | Path) -> dict:
    dataset_folder = Path(dataset_folder).resolve()
    run_folder = Path(run_folder).resolve()
    data = load_dataset(dataset_folder)
    kan, payload = load_trained(run_folder / "best_pkan.pt", dataset_folder)
    mlp_path = run_folder / "best_mlp.pt"
    mlp = load_trained(mlp_path, dataset_folder)[0] if mlp_path.is_file() else None
    residual_kan = _infer(kan, data)
    predictions = {"nominal_rbd": data["tau_nominal"],
                   "residual_pkan": data["tau_nominal"] + residual_kan,
                   "tool_aware_rbd": data["tau_aware"]}
    if mlp is not None:
        predictions["residual_mlp"] = data["tau_nominal"] + _infer(mlp, data)
    reference = data["tau_reference"]
    report = {"created_utc": datetime.now(timezone.utc).isoformat(),
              "model_version": "residual_pkan_v1",
              "dataset_sha256": hashlib.sha256((dataset_folder / "dataset.npz").read_bytes()).hexdigest(),
              "model_checkpoint": "best_pkan.pt", "best_epoch": payload["best_epoch"],
              "torque_unit": "Nm", "reference_definition": "MuJoCo simulated qfrc_inverse",
              "model_definitions": {
                  "nominal_rbd": "MuJoCo RBD with 0.8x mass, 0.75x inertia and specified CoM offset",
                  "residual_pkan": "nominal RBD plus learned 34-input KAN residual",
                  "tool_aware_rbd": "M(q)ddq + bias(q,dq) + damping with exact reference tool parameters; numerical error expected",
                  "residual_mlp": "optional nominal RBD plus MLP residual with the same inputs",
              }, "splits": {}}
    for split in ("validation", *SPLITS[2:]):
        split_index = np.flatnonzero(data["split"] == split)
        if len(split_index) == 0:
            continue
        groups = {"all": split_index}
        for tool in np.unique(data["tool_condition"][split_index]):
            groups[str(tool)] = split_index[data["tool_condition"][split_index] == tool]
        if split == "test_mismatch_severity":
            for factor in np.unique(data["nominal_mass_factor"][split_index]):
                groups[f"factor_{factor:g}"] = split_index[
                    data["nominal_mass_factor"][split_index] == factor]
        split_report = {}
        for group, indices in groups.items():
            measured = {name: _metrics(reference[indices], values[indices])
                        for name, values in predictions.items()}
            baseline = measured["nominal_rbd"]["rmse_nm"]
            measured["rbd_error_reduction_pct"] = (
                100 * (baseline - measured["residual_pkan"]["rmse_nm"]) / baseline
                if baseline > 1e-9 else None)
            split_report[group] = {"samples": len(indices), "models": measured}
        report["splits"][split] = split_report

    output_csv = run_folder / "predictions.csv"
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        columns = ["split", "trajectory_id", "trial_id", "tool_condition", "nominal_mass_factor", "time_s"]
        for stem in ("tau_reference", "tau_nominal", "delta_tau_target", "delta_tau_pkan",
                     "tau_pkan", "tau_aware", "tau_mlp"):
            columns.extend(f"{stem}_{i}_nm" for i in range(1, 7))
        writer.writerow(columns)
        for index in range(len(reference)):
            writer.writerow([data["split"][index], data["trajectory_id"][index],
                             data["trial_id"][index], data["tool_condition"][index],
                             data["nominal_mass_factor"][index], data["time_s"][index],
                             *reference[index], *data["tau_nominal"][index],
                             *data["delta_tau_target"][index], *residual_kan[index],
                             *predictions["residual_pkan"][index], *data["tau_aware"][index],
                             *(predictions["residual_mlp"][index] if mlp is not None else [""] * 6)])
    (run_folder / "metrics.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main(argv: list[str] | None = None) -> int:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Evaluate A0509 residual torque models")
    parser.add_argument("--dataset", default=str(root / "residual_data" / "rbd_v1"))
    parser.add_argument("--run", default=str(root / "residual_runs" / "rbd_v1"))
    args = parser.parse_args(argv)
    report = evaluate(args.dataset, args.run)
    for split, groups in report["splits"].items():
        models = groups["all"]["models"]
        print(split, groups["all"]["samples"],
              "nominal", round(models["nominal_rbd"]["rmse_nm"], 4),
              "pkan", round(models["residual_pkan"]["rmse_nm"], 4),
              "aware", round(models["tool_aware_rbd"]["rmse_nm"], 4))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
