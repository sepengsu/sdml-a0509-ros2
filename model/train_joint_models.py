"""Train one of M1-M4 jointly across all seven campaign conditions."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import random

import numpy as np
import torch

from .prepare_joint_dataset import CONDITIONS, DEFAULT_OUTPUT as DEFAULT_DATA, PROFILES
from .residual_models import DirectKAN, DirectMLP, ResidualKAN, ResidualMLP


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "data/wide16_v3_joint_models"
MODELS = {"M1": (DirectMLP, "direct_mlp", "tau_reference"),
          "M2": (DirectKAN, "direct_kan", "tau_reference"),
          "M3": (ResidualMLP, "rbd_plus_mlp", "residual"),
          "M4": (ResidualKAN, "rbd_plus_kan", "residual")}


def load_dataset(data_root: Path) -> tuple[dict, dict]:
    data_root = data_root.resolve()
    manifest_path = data_root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("profile") not in PROFILES or tuple(manifest.get("conditions", ())) != CONDITIONS:
        raise ValueError("expected a supported complete seven-condition dataset")
    path = data_root / "dataset.npz"
    if hashlib.sha256(path.read_bytes()).hexdigest() != manifest["dataset_sha256"]:
        raise ValueError("dataset digest mismatch")
    with np.load(path, allow_pickle=False) as raw:
        arrays = {key: raw[key] for key in ("features", "tau_reference", "tau_nominal",
                                           "condition", "trajectory", "split")}
    n = len(arrays["features"])
    if (arrays["features"].shape != (n, 22) or arrays["tau_reference"].shape != (n, 6)
            or arrays["tau_nominal"].shape != (n, 6) or any(len(v) != n for v in arrays.values())
            or not all(np.isfinite(arrays[k]).all() for k in ("features", "tau_reference", "tau_nominal"))):
        raise ValueError("invalid dataset shape or values")
    for split, code in manifest["split_codes"].items():
        if int((arrays["split"] == code).sum()) != manifest["rows"][split] or manifest["rows"][split] < 1:
            raise ValueError(f"split count mismatch: {split}")
    return manifest, arrays


def train(model_id: str, data_root: Path, output_root: Path, *, epochs: int = 60,
          batch_size: int = 1024, patience: int = 8, learning_rate: float = 0.002,
          min_delta: float = 1e-8, seed: int = 42, threads: int = 1) -> dict:
    if model_id not in MODELS:
        raise ValueError(f"model must be one of {tuple(MODELS)}")
    if (epochs < 1 or batch_size < 1 or patience < 1 or learning_rate <= 0
            or min_delta < 0 or threads < 1):
        raise ValueError("invalid training parameters")
    manifest, arrays = load_dataset(data_root)
    output = output_root.resolve() / model_id
    if output.exists():
        raise FileExistsError(f"model output already exists; choose a fresh --output: {output}")
    torch.set_num_threads(threads)
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    model_type, name, target_kind = MODELS[model_id]
    x = torch.from_numpy(arrays["features"].astype(np.float32, copy=False))
    reference = torch.from_numpy(arrays["tau_reference"].astype(np.float32, copy=False))
    nominal = torch.from_numpy(arrays["tau_nominal"].astype(np.float32, copy=False))
    target = reference if target_kind == "tau_reference" else reference - nominal
    train_indices = torch.from_numpy(np.flatnonzero(arrays["split"] == manifest["split_codes"]["train"]))
    val_indices = torch.from_numpy(np.flatnonzero(arrays["split"] == manifest["split_codes"]["validation"]))
    input_mean = x[train_indices].mean(dim=0)
    input_scale = x[train_indices].std(dim=0, unbiased=False).clamp_min(0.25)
    output_mean = target[train_indices].mean(dim=0)
    output_scale = target[train_indices].std(dim=0, unbiased=False).clamp_min(0.05)
    model = model_type(input_mean, input_scale, output_mean, output_scale)
    optimizer = torch.optim.AdamW(model.parameters(), lr=learning_rate, weight_decay=1e-5)
    best_loss = float("inf")
    best_state = None
    best_epoch = 0
    stop_reason = "max_epochs"
    history = []
    for epoch in range(1, epochs + 1):
        model.train()
        order = train_indices[torch.randperm(len(train_indices))]
        train_weighted = 0.0
        for indices in order.split(batch_size):
            optimizer.zero_grad(set_to_none=True)
            prediction = model(x[indices])
            loss = (((prediction - target[indices]) / output_scale) ** 2).mean()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            optimizer.step()
            train_weighted += float(loss.detach()) * len(indices)
        model.eval()
        val_sse = 0.0
        val_nm_sse = 0.0
        with torch.no_grad():
            for indices in val_indices.split(batch_size):
                error = model(x[indices]) - target[indices]
                val_sse += float(((error / output_scale) ** 2).sum())
                val_nm_sse += float((error ** 2).sum())
        val_loss = val_sse / (len(val_indices) * 6)
        val_rmse = (val_nm_sse / (len(val_indices) * 6)) ** 0.5
        history.append({"epoch": epoch, "train_normalized_mse": train_weighted / len(train_indices),
                        "validation_normalized_mse": val_loss, "validation_target_rmse_nm": val_rmse})
        print(f"{model_id} epoch {epoch}: val loss {val_loss:.6f}, target RMSE {val_rmse:.6f} Nm", flush=True)
        if not np.isfinite(val_loss):
            raise ValueError(f"nonfinite validation loss at epoch {epoch}")
        if val_loss < best_loss - min_delta:
            best_loss = val_loss
            best_epoch = epoch
            best_state = {key: value.detach().clone() for key, value in model.state_dict().items()}
        if epoch - best_epoch >= patience:
            stop_reason = "early_stopping"
            print(f"{model_id}: early stopping at epoch {epoch}; best epoch {best_epoch}", flush=True)
            break
    assert best_state is not None
    output.mkdir(parents=True, exist_ok=False)
    checkpoint = {"model_id": model_id, "model_name": name, "target_kind": target_kind,
                  "model_state": best_state, "dataset_sha256": manifest["dataset_sha256"],
                  "source": manifest["source"], "profile": manifest["profile"],
                  "nominal_inertial_scale": manifest["nominal_inertial_scale"],
                  "best_epoch": best_epoch, "seed": seed}
    torch.save(checkpoint, output / "best.pt")
    report = {"created_utc": datetime.now(timezone.utc).isoformat(),
              "model_id": model_id, "model_name": name, "target_kind": target_kind,
              "dataset_sha256": manifest["dataset_sha256"], "data_root": str(data_root.resolve()),
              "parameters": {"epochs_requested": epochs, "epochs_run": len(history),
                             "batch_size": batch_size, "patience": patience,
                             "learning_rate": learning_rate, "min_delta": min_delta,
                             "seed": seed, "threads": threads,
                             "trainable_count": sum(p.numel() for p in model.parameters())},
              "best_epoch": best_epoch, "stop_reason": stop_reason,
              "best_validation_normalized_mse": best_loss,
              "history": history}
    (output / "training.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                          encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=MODELS, required=True)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch-size", type=int, default=1024)
    parser.add_argument("--patience", type=int, default=8)
    parser.add_argument("--learning-rate", type=float, default=0.002)
    parser.add_argument("--min-delta", type=float, default=1e-8)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--threads", type=int, default=1)
    args = parser.parse_args()
    report = train(args.model, args.data, args.output, epochs=args.epochs,
                   batch_size=args.batch_size, patience=args.patience,
                   learning_rate=args.learning_rate, min_delta=args.min_delta,
                   seed=args.seed, threads=args.threads)
    print(json.dumps({"model": args.model, "best_epoch": report["best_epoch"],
                      "output": str((args.output / args.model).resolve())}, indent=2))


if __name__ == "__main__":
    main()
