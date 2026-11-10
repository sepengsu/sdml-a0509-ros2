"""Sequential T1→T4 PKAN training with rehearsal and diagonal EWC."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import random
import sys

import numpy as np
import torch

from .data import REPO_ROOT, TRIAL_IDS, Trial, data_summary, load_trials, time_split
from .physics_check import check as check_physics
from .pkan import PhysicsKAN


TASKS = tuple(TRIAL_IDS)


@dataclass
class Config:
    teacher: str
    output: str
    stride: int = 5
    epochs_per_task: int = 40
    batch_size: int = 128
    learning_rate: float = 0.002
    hidden_dim: int = 16
    grid_intervals: int = 8
    replay_per_task: int = 64
    ewc_lambda: float = 0.2
    fisher_batches: int = 4
    seed: int = 42
    threads: int = 1


def _load_teacher(path: Path, trials: dict[str, Trial], stride: int) -> dict[str, np.ndarray]:
    manifest = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
    if manifest["stride"] != stride:
        raise ValueError("teacher stride does not match training stride")
    scene = trials["T1"].folder / "ag145_scene.xml"
    if hashlib.sha256(scene.read_bytes()).hexdigest() != manifest["scene_sha256"]:
        raise ValueError("AG145 MuJoCo scene changed since teacher generation")
    matrices = {}
    with np.load(path) as archive:
        for task, trial in trials.items():
            source_hash = hashlib.sha256((trial.folder / "dynamics.csv").read_bytes()).hexdigest()
            if manifest["trials"][task]["dynamics_csv_sha256"] != source_hash:
                raise ValueError(f"{task} CSV changed since teacher generation")
            if not np.allclose(archive[f"{task}_q"], trial.q, atol=1e-6):
                raise ValueError(f"{task} teacher states do not match CSV")
            mass = archive[f"{task}_mass_matrix"].copy()
            if mass.shape != (trial.size, 6, 6):
                raise ValueError(f"{task} mass matrices have wrong shape")
            matrices[task] = mass
    return matrices


def _tensor_data(trial: Trial, mass: np.ndarray) -> dict[str, torch.Tensor]:
    return {"q": torch.from_numpy(trial.q.astype(np.float32)),
            "dq": torch.from_numpy(trial.dq.astype(np.float32)),
            "ddq": torch.from_numpy(trial.ddq.astype(np.float32)),
            "tau": torch.from_numpy(trial.tau_inverse.astype(np.float32)),
            "bias": torch.from_numpy(trial.bias.astype(np.float32)),
            "mass": torch.from_numpy(mass.astype(np.float32))}


def _batch(data: dict[str, torch.Tensor], indices: torch.Tensor) -> dict[str, torch.Tensor]:
    return {key: value[indices] for key, value in data.items()}


def _loss(model: PhysicsKAN, batch: dict[str, torch.Tensor],
          scales: dict[str, torch.Tensor]) -> tuple[torch.Tensor, dict[str, float]]:
    prediction = model(batch["q"], batch["dq"], batch["ddq"])
    mass = torch.mean(((prediction["mass_matrix"] - batch["mass"]) / scales["mass"]) ** 2)
    bias = torch.mean(((prediction["bias"] - batch["bias"]) / scales["bias"]) ** 2)
    torque = torch.mean(((prediction["tau"] - batch["tau"]) / scales["tau"]) ** 2)
    return mass + bias + torque, {"mass": mass.item(), "bias": bias.item(), "torque": torque.item()}


def _evaluate(model: PhysicsKAN, data: dict[str, torch.Tensor],
              indices: np.ndarray) -> tuple[dict, dict[str, np.ndarray]]:
    predictions = {key: [] for key in ("tau", "bias", "mass_matrix")}
    with torch.no_grad():
        for part in np.array_split(indices, max(1, int(np.ceil(len(indices) / 256)))):
            selected = torch.as_tensor(part, dtype=torch.long)
            out = model(data["q"][selected], data["dq"][selected], data["ddq"][selected])
            for key in predictions:
                predictions[key].append(out[key].cpu().numpy())
    predicted = {key: np.concatenate(values) for key, values in predictions.items()}
    tau_true = data["tau"][indices].numpy()
    bias_true = data["bias"][indices].numpy()
    mass_true = data["mass"][indices].numpy()
    torque_error = predicted["tau"] - tau_true
    bias_error = predicted["bias"] - bias_true
    mass_error = predicted["mass_matrix"] - mass_true
    metrics = {
        "samples": len(indices),
        "torque_rmse_nm": float(np.sqrt(np.mean(torque_error**2))),
        "torque_mae_nm": float(np.mean(np.abs(torque_error))),
        "joint_torque_rmse_nm": np.sqrt(np.mean(torque_error**2, axis=0)).tolist(),
        "bias_rmse_nm": float(np.sqrt(np.mean(bias_error**2))),
        "mass_relative_frobenius": float(np.linalg.norm(mass_error) / np.linalg.norm(mass_true)),
        "minimum_mass_eigenvalue": float(np.linalg.eigvalsh(predicted["mass_matrix"]).min()),
    }
    return metrics, predicted


def _fisher(model: PhysicsKAN, data: dict[str, torch.Tensor], train_indices: np.ndarray,
            scales: dict[str, torch.Tensor], batches: int, batch_size: int,
            rng: np.random.Generator) -> dict[str, torch.Tensor]:
    fisher = {name: torch.zeros_like(param) for name, param in model.named_parameters()}
    for _ in range(batches):
        selected = rng.choice(train_indices, size=min(batch_size, len(train_indices)), replace=False)
        model.zero_grad(set_to_none=True)
        loss, _ = _loss(model, _batch(data, torch.as_tensor(selected, dtype=torch.long)), scales)
        loss.backward()
        for name, param in model.named_parameters():
            if param.grad is not None:
                fisher[name] += param.grad.detach().square() / batches
    model.zero_grad(set_to_none=True)
    return fisher


def _ewc_penalty(model: PhysicsKAN, memories: list[tuple[dict, dict]]) -> torch.Tensor:
    penalty = torch.zeros((), dtype=next(model.parameters()).dtype)
    for reference, fisher in memories:
        for name, param in model.named_parameters():
            penalty = penalty + torch.sum(fisher[name] * (param - reference[name]).square())
    return penalty


def _export_predictions(path: Path, trial: Trial, indices: np.ndarray,
                        predicted: dict[str, np.ndarray]) -> None:
    columns = ["ros_time_s", *[f"tau_true_{i}" for i in range(1, 7)],
               *[f"tau_pred_{i}" for i in range(1, 7)],
               *[f"bias_true_{i}" for i in range(1, 7)],
               *[f"bias_pred_{i}" for i in range(1, 7)]]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(columns)
        for position, index in enumerate(indices):
            writer.writerow([trial.time_s[index], *trial.tau_inverse[index],
                             *predicted["tau"][position], *trial.bias[index],
                             *predicted["bias"][position]])


def train(config: Config) -> dict:
    if config.epochs_per_task < 1 or config.batch_size < 1 or config.fisher_batches < 1:
        raise ValueError("epochs, batch size and Fisher batches must be positive")
    if config.replay_per_task < 0 or config.ewc_lambda < 0:
        raise ValueError("replay and EWC strength must be nonnegative")
    torch.set_num_threads(config.threads)
    torch.manual_seed(config.seed)
    random.seed(config.seed)
    rng = np.random.default_rng(config.seed)
    output = Path(config.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    teacher_path = Path(config.teacher).resolve()
    trials = load_trials(stride=config.stride)
    matrices = _load_teacher(teacher_path, trials, config.stride)
    identity = check_physics(teacher_path, config.stride)
    if max(item["equation_rmse_nm"] for item in identity.values()) > 1e-3:
        raise ValueError("MuJoCo inverse-dynamics identity fails; refusing to train")
    data = {task: _tensor_data(trial, matrices[task]) for task, trial in trials.items()}
    splits = {task: time_split(trial) for task, trial in trials.items()}
    first_train = splits["T1"][0]
    first = data["T1"]
    initial_mass = first["mass"][first_train].mean(dim=0)
    initial_bias = first["bias"][first_train].mean(dim=0)
    baseline = {}
    for task, trial in trials.items():
        _, validation_indices = splits[task]
        torque = (np.einsum("ij,bj->bi", initial_mass.numpy(), trial.ddq[validation_indices])
                  + initial_bias.numpy() + trial.damping * trial.dq[validation_indices])
        baseline[task] = {"torque_rmse_nm": float(np.sqrt(np.mean(
            (torque - trial.tau_inverse[validation_indices]) ** 2))),
                          "definition": "T1 mean M and bias plus fixed joint damping"}
    model = PhysicsKAN(initial_mass, initial_bias, torch.from_numpy(trials["T1"].damping.astype(np.float32)),
                       hidden_dim=config.hidden_dim, grid_intervals=config.grid_intervals)
    scales = {
        "mass": torch.sqrt(torch.mean(first["mass"][first_train].square(), dim=0)).clamp_min(0.05),
        "bias": torch.sqrt(torch.mean(first["bias"][first_train].square(), dim=0)).clamp_min(2.0),
        "tau": torch.sqrt(torch.mean(first["tau"][first_train].square(), dim=0)).clamp_min(10.0),
    }
    optimizer = torch.optim.AdamW(model.parameters(), lr=config.learning_rate, weight_decay=1e-5)
    memories: list[tuple[dict, dict]] = []
    rehearsal: list[dict[str, torch.Tensor]] = []
    history = []
    stages = {}
    for stage, task in enumerate(TASKS, start=1):
        train_indices, _ = splits[task]
        current = _batch(data[task], torch.as_tensor(train_indices, dtype=torch.long))
        pieces = [current, *rehearsal]
        combined = {key: torch.cat([piece[key] for piece in pieces]) for key in current}
        for epoch in range(1, config.epochs_per_task + 1):
            model.train()
            order = torch.randperm(len(combined["q"]))
            running = 0.0
            for indices in order.split(config.batch_size):
                optimizer.zero_grad(set_to_none=True)
                data_loss, _ = _loss(model, _batch(combined, indices), scales)
                regularizer = config.ewc_lambda * _ewc_penalty(model, memories)
                loss = data_loss + regularizer
                loss.backward()
                torch.nn.utils.clip_grad_norm_(model.parameters(), 5.0)
                optimizer.step()
                running += float(data_loss.item()) * len(indices)
            history.append({"stage": task, "epoch": epoch,
                            "data_loss": running / len(combined["q"])})
        model.eval()
        stages[task] = {}
        for evaluated in TASKS:
            _, validation_indices = splits[evaluated]
            stages[task][evaluated], _ = _evaluate(model, data[evaluated], validation_indices)
        print(f"after {task}: " + ", ".join(
            f"{evaluated} {stages[task][evaluated]['torque_rmse_nm']:.3f} Nm"
            for evaluated in TASKS), flush=True)
        fisher = _fisher(model, data[task], train_indices, scales, config.fisher_batches,
                         config.batch_size, rng)
        reference = {name: param.detach().clone() for name, param in model.named_parameters()}
        memories.append((reference, fisher))
        if config.replay_per_task:
            selected = rng.choice(train_indices, size=min(config.replay_per_task, len(train_indices)),
                                  replace=False)
            rehearsal.append(_batch(data[task], torch.as_tensor(selected, dtype=torch.long)))
        torch.save({"model_state": model.state_dict(), "config": asdict(config),
                    "completed_task": task, "normalization": scales}, output / f"after_{task}.pt")
    final_predictions = {}
    for task in TASKS:
        _, validation_indices = splits[task]
        _, predicted = _evaluate(model, data[task], validation_indices)
        _export_predictions(output / f"{task}_validation_predictions.csv",
                            trials[task], validation_indices, predicted)
        final_predictions[task] = stages["T4"][task]
    checkpoint = output / "final.pt"
    torch.save({"model_state": model.state_dict(), "config": asdict(config),
                "completed_task": "T4", "normalization": scales}, checkpoint)
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "software_versions": {"python": sys.version.split()[0],
                              "numpy": np.__version__, "torch": torch.__version__},
        "model": "PhysicsKAN: cubic spline edge functions; SPD M(q), anchored b(q,dq), fixed D dq",
        "purpose": "MuJoCo AG145 inverse-dynamics surrogate; no independent real torque labels",
        "config": asdict(config), "teacher_sha256": hashlib.sha256(teacher_path.read_bytes()).hexdigest(),
        "data": data_summary(trials), "physics_identity": identity,
        "fixed_T1_baseline": baseline,
        "splits": "first 80% of each subsampled trial for training, final 20% for temporal validation",
        "stages": stages, "final_validation": final_predictions,
        "history": history, "checkpoint": checkpoint.name,
    }
    (output / "metrics.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main(argv: list[str] | None = None) -> int:
    folder = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Train a sequential AG145 physics-informed KAN")
    parser.add_argument("--teacher", default=str(folder / "data" / "ag145_mass_matrices.npz"))
    parser.add_argument("--output", default=str(folder / "runs" / "ag145_pkan"))
    parser.add_argument("--stride", type=int, default=5)
    parser.add_argument("--epochs-per-task", type=int, default=40)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--learning-rate", type=float, default=0.002)
    parser.add_argument("--hidden-dim", type=int, default=16)
    parser.add_argument("--grid-intervals", type=int, default=8)
    parser.add_argument("--replay-per-task", type=int, default=64)
    parser.add_argument("--ewc-lambda", type=float, default=0.2)
    parser.add_argument("--fisher-batches", type=int, default=4)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--threads", type=int, default=1)
    args = parser.parse_args(argv)
    train(Config(**vars(args)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
