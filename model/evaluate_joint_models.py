"""Evaluate M0-M4 on the wide16 v3 validation and held-out test splits."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path

import numpy as np
import torch

from .train_joint_models import DEFAULT_DATA, DEFAULT_OUTPUT as DEFAULT_MODELS, MODELS, load_dataset


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "data/wide16_v3_joint_evaluation"
NAMES = {"M0": "Nominal RBD", "M1": "Direct MLP", "M2": "Direct KAN",
         "M3": "RBD + MLP", "M4": "RBD + KAN"}


def _metrics(prediction: np.ndarray, reference: np.ndarray) -> dict:
    error = prediction.astype(np.float64) - reference.astype(np.float64)
    return {"samples": len(reference), "torque_rmse_nm": float(np.sqrt(np.mean(error ** 2))),
            "joint_rmse_nm": np.sqrt(np.mean(error ** 2, axis=0)).tolist(),
            "torque_mae_nm": float(np.mean(np.abs(error)))}


def evaluate(data_root: Path, model_root: Path, output: Path, *, batch_size: int = 4096,
             threads: int = 1) -> dict:
    if batch_size < 1 or threads < 1:
        raise ValueError("batch size and threads must be positive")
    if output.exists():
        raise FileExistsError(f"choose a new --output: {output}")
    torch.set_num_threads(threads)
    manifest, arrays = load_dataset(data_root)
    predictions = {"M0": arrays["tau_nominal"]}
    for model_id, (model_type, expected_name, target_kind) in MODELS.items():
        path = model_root / model_id / "best.pt"
        checkpoint = torch.load(path, map_location="cpu", weights_only=True)
        if (checkpoint.get("model_id") != model_id or checkpoint.get("model_name") != expected_name
                or checkpoint.get("target_kind") != target_kind
                or checkpoint.get("dataset_sha256") != manifest["dataset_sha256"]
                or checkpoint.get("profile") != manifest["profile"]
                or checkpoint.get("source") != manifest["source"]
                or checkpoint.get("nominal_inertial_scale") != manifest["nominal_inertial_scale"]):
            raise ValueError(f"checkpoint provenance mismatch: {path}")
        model = model_type()
        model.load_state_dict(checkpoint["model_state"])
        model.eval()
        chunks = []
        with torch.no_grad():
            for start in range(0, len(arrays["features"]), batch_size):
                features = torch.from_numpy(arrays["features"][start:start + batch_size])
                chunks.append(model(features).numpy())
        prediction = np.concatenate(chunks)
        if target_kind == "residual":
            prediction = arrays["tau_nominal"] + prediction
        if not np.isfinite(prediction).all():
            raise ValueError(f"nonfinite prediction: {model_id}")
        predictions[model_id] = prediction
    scores = {}
    rows = []
    for split in ("validation", "test"):
        split_mask = arrays["split"] == manifest["split_codes"][split]
        scores[split] = {"all": {}, "by_condition": {}, "by_trajectory": {}}
        for model_id, prediction in predictions.items():
            scores[split]["all"][model_id] = _metrics(prediction[split_mask], arrays["tau_reference"][split_mask])
        for ci, condition in enumerate(manifest["conditions"]):
            mask = split_mask & (arrays["condition"] == ci)
            scores[split]["by_condition"][condition] = {}
            for model_id, prediction in predictions.items():
                item = _metrics(prediction[mask], arrays["tau_reference"][mask])
                scores[split]["by_condition"][condition][model_id] = item
                rows.append({"split": split, "condition": condition, "model_id": model_id,
                             "model_name": NAMES[model_id], "samples": item["samples"],
                             "torque_rmse_nm": item["torque_rmse_nm"],
                             "torque_mae_nm": item["torque_mae_nm"]})
        for trajectory in manifest["split_policy"][split]:
            mask = split_mask & (arrays["trajectory"] == int(trajectory[1:]))
            scores[split]["by_trajectory"][trajectory] = {
                model_id: _metrics(prediction[mask], arrays["tau_reference"][mask])
                for model_id, prediction in predictions.items()}
    output.mkdir(parents=True, exist_ok=False)
    report = {"created_utc": datetime.now(timezone.utc).isoformat(),
              "data_root": str(data_root.resolve()), "model_root": str(model_root.resolve()),
              "dataset_sha256": manifest["dataset_sha256"], "profile": manifest["profile"],
              "model_names": NAMES, "scores": scores,
              "interpretation": "MuJoCo inverse-dynamics labels; no measured motor torque or real-robot validation"}
    (output / "metrics.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                         encoding="utf-8")
    with (output / "comparison.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    test_mask = arrays["split"] == manifest["split_codes"]["test"]
    np.savez_compressed(output / "test_predictions.npz",
                        features=arrays["features"][test_mask],
                        tau_reference=arrays["tau_reference"][test_mask],
                        condition=arrays["condition"][test_mask],
                        trajectory=arrays["trajectory"][test_mask],
                        **{model_id: prediction[test_mask] for model_id, prediction in predictions.items()})
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--models", type=Path, default=DEFAULT_MODELS)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--batch-size", type=int, default=4096)
    parser.add_argument("--threads", type=int, default=1)
    args = parser.parse_args()
    report = evaluate(args.data, args.models, args.output, batch_size=args.batch_size, threads=args.threads)
    print(json.dumps({model: item["torque_rmse_nm"]
                      for model, item in report["scores"]["test"]["all"].items()}, indent=2))


if __name__ == "__main__":
    main()
