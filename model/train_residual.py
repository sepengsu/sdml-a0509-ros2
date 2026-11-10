"""Train residual PKAN (and optional MLP) on paired offline MuJoCo RBD data."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import random

import numpy as np
import torch

from .residual_data import load_dataset
from .residual_pkan import ResidualMLP, ResidualPKAN, normalization


@dataclass
class Config:
    dataset: str
    output: str
    max_epochs: int = 100
    patience: int = 15
    batch_size: int = 128
    learning_rate: float = 0.002
    residual_lambda: float = 0.001
    seed: int = 42
    train_mlp: bool = True


def _tensors(data: dict[str, np.ndarray], index: np.ndarray) -> dict[str, torch.Tensor]:
    return {key: torch.as_tensor(data[key][index], dtype=torch.float32)
            for key in ("q", "dq", "ddq", "tool_nominal", "tau_nominal",
                        "tau_reference", "delta_tau_target")}


def _loss(model, batch: dict[str, torch.Tensor], weight: float):
    result = model.corrected(batch["q"], batch["dq"], batch["ddq"],
                             batch["tool_nominal"], batch["tau_nominal"])
    normalized_error = (result["torque"] - batch["tau_reference"]) / model.target_scale
    normalized_residual = result["residual"] / model.target_scale
    torque_loss = normalized_error.square().mean()
    residual_penalty = normalized_residual.square().mean()
    return torque_loss + weight * residual_penalty, float(torque_loss.detach()), float(residual_penalty.detach())


def _train_one(kind: str, norms: dict[str, torch.Tensor], train_data: dict,
               val_data: dict, config: Config, output: Path, dataset_hash: str) -> dict:
    torch.manual_seed(config.seed)
    model = (ResidualPKAN(**norms) if kind == "pkan" else ResidualMLP(**norms))
    optimizer = torch.optim.AdamW(model.parameters(), lr=config.learning_rate, weight_decay=1e-5)
    best_loss = float("inf")
    best_epoch = 0
    history = []
    no_improvement = 0
    batch_count = len(train_data["q"])
    for epoch in range(1, config.max_epochs + 1):
        model.train()
        order = torch.randperm(batch_count)
        weighted = 0.0
        for indices in order.split(config.batch_size):
            batch = {key: value[indices] for key, value in train_data.items()}
            optimizer.zero_grad(set_to_none=True)
            loss, _, _ = _loss(model, batch, config.residual_lambda)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
            optimizer.step()
            weighted += float(loss.detach()) * len(indices)
        model.eval()
        with torch.no_grad():
            val_loss, val_torque, val_penalty = _loss(model, val_data, config.residual_lambda)
        record = {"epoch": epoch, "train_loss": weighted / batch_count,
                  "validation_loss": float(val_loss), "validation_torque_loss": val_torque,
                  "validation_residual_penalty": val_penalty}
        history.append(record)
        if record["validation_loss"] < best_loss - 1e-6:
            best_loss = record["validation_loss"]
            best_epoch = epoch
            no_improvement = 0
            torch.save({"model_kind": kind, "model_state": model.state_dict(),
                        "config": asdict(config), "dataset_sha256": dataset_hash,
                        "best_epoch": best_epoch}, output / f"best_{kind}.pt")
        else:
            no_improvement += 1
        if no_improvement >= config.patience:
            break
    return {"kind": kind, "best_epoch": best_epoch, "best_validation_loss": best_loss,
            "epochs_run": len(history), "history": history,
            "parameters": sum(param.numel() for param in model.parameters())}


def train(config: Config) -> dict:
    if (config.max_epochs < 1 or config.patience < 1 or config.batch_size < 1
            or config.learning_rate <= 0 or config.residual_lambda < 0):
        raise ValueError("invalid training hyperparameters")
    torch.set_num_threads(1)
    torch.manual_seed(config.seed)
    np.random.seed(config.seed)
    random.seed(config.seed)
    dataset = Path(config.dataset).resolve()
    output = Path(config.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    data = load_dataset(dataset)
    train_index = np.flatnonzero(data["split"] == "train")
    val_index = np.flatnonzero(data["split"] == "validation")
    if len(train_index) == 0 or len(val_index) == 0:
        raise ValueError("training or validation split is empty")
    norms = normalization(*(data[key][train_index] for key in
                            ("q", "dq", "ddq", "tool_nominal", "delta_tau_target")))
    train_data = _tensors(data, train_index)
    val_data = _tensors(data, val_index)
    dataset_hash = hashlib.sha256((dataset / "dataset.npz").read_bytes()).hexdigest()
    reports = {}
    for kind in (("pkan", "mlp") if config.train_mlp else ("pkan",)):
        reports[kind] = _train_one(kind, norms, train_data, val_data, config, output, dataset_hash)
        print(f"{kind}: best epoch {reports[kind]['best_epoch']}, "
              f"validation loss {reports[kind]['best_validation_loss']:.6f}", flush=True)
    report = {"created_utc": datetime.now(timezone.utc).isoformat(),
              "model": "nominal MuJoCo RBD + direct residual KAN",
              "config": asdict(config), "dataset_sha256": dataset_hash,
              "normalization_source": "train split only", "runs": reports}
    (output / "training.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    (output / "config.json").write_text(json.dumps(asdict(config), indent=2) + "\n", encoding="utf-8")
    return report


def load_trained(checkpoint: str | Path, dataset: str | Path | None = None):
    payload = torch.load(checkpoint, map_location="cpu", weights_only=True)
    if dataset is not None:
        digest = hashlib.sha256((Path(dataset) / "dataset.npz").read_bytes()).hexdigest()
        if digest != payload["dataset_sha256"]:
            raise ValueError("checkpoint and dataset do not match")
    state = payload["model_state"]
    norms = {key: state[key] for key in
             ("feature_mean", "feature_scale", "target_mean", "target_scale")}
    kind = payload["model_kind"]
    if kind not in ("pkan", "mlp"):
        raise ValueError("unsupported residual checkpoint")
    model = ResidualPKAN(**norms) if kind == "pkan" else ResidualMLP(**norms)
    model.load_state_dict(state)
    model.eval()
    return model, payload


def main(argv: list[str] | None = None) -> int:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Train A0509 RBD residual PKAN")
    parser.add_argument("--dataset", default=str(root / "residual_data" / "rbd_v1"))
    parser.add_argument("--output", default=str(root / "residual_runs" / "rbd_v1"))
    parser.add_argument("--max-epochs", type=int, default=100)
    parser.add_argument("--patience", type=int, default=15)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--learning-rate", type=float, default=0.002)
    parser.add_argument("--residual-lambda", type=float, default=0.001)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--no-mlp", action="store_true")
    args = parser.parse_args(argv)
    train(Config(args.dataset, args.output, args.max_epochs, args.patience,
                 args.batch_size, args.learning_rate, args.residual_lambda,
                 args.seed, not args.no_mlp))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
