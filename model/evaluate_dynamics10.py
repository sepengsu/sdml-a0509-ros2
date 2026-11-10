"""Evaluate old/new residual PKAN on trajectory-disjoint AG145 observations."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .data import REPO_ROOT
from .dynamics10_data import SPLIT, TRAJECTORIES
from .evaluate_residual import _infer, _metrics
from .residual_data import load_dataset
from .train_residual import load_trained


MODEL_ORDER = ("nominal_rbd", "previous_pkan", "dynamics10_pkan", "dynamics10_mlp", "tool_aware_rbd")
COLORS = {"nominal_rbd": "#d55e00", "previous_pkan": "#999999",
          "dynamics10_pkan": "#2271b2", "dynamics10_mlp": "#aa3377",
          "tool_aware_rbd": "#009e73"}
LABELS = {"nominal_rbd": "Nominal RBD", "previous_pkan": "Previous PKAN (T1-T3)",
          "dynamics10_pkan": "New PKAN (T5-T11)", "dynamics10_mlp": "New MLP (T5-T11)",
          "tool_aware_rbd": "Tool-aware RBD"}


def evaluate(dataset: str | Path, run: str | Path, previous_checkpoint: str | Path) -> dict:
    dataset, run = Path(dataset).resolve(), Path(run).resolve()
    run.mkdir(parents=True, exist_ok=True)
    data = load_dataset(dataset)
    new_model, new_checkpoint = load_trained(run / "best_pkan.pt", dataset)
    mlp_model, mlp_checkpoint = load_trained(run / "best_mlp.pt", dataset)
    old_model, old_checkpoint = load_trained(previous_checkpoint)
    reference = data["tau_reference"]
    predictions = {"nominal_rbd": data["tau_nominal"],
                   "previous_pkan": data["tau_nominal"] + _infer(old_model, data),
                   "dynamics10_pkan": data["tau_nominal"] + _infer(new_model, data),
                   "dynamics10_mlp": data["tau_nominal"] + _infer(mlp_model, data),
                   "tool_aware_rbd": data["tau_aware"]}
    report = {"created_utc": datetime.now(timezone.utc).isoformat(),
              "version": "dynamics10_residual_evaluation_v1",
              "dataset_sha256": hashlib.sha256((dataset / "dataset.npz").read_bytes()).hexdigest(),
              "new_pkan_best_epoch": new_checkpoint["best_epoch"],
              "new_mlp_best_epoch": mlp_checkpoint["best_epoch"],
              "previous_pkan_best_epoch": old_checkpoint["best_epoch"],
              "previous_checkpoint": str(Path(previous_checkpoint).resolve().relative_to(REPO_ROOT)).replace("\\", "/"),
              "reference": "recorded AG145 MuJoCo qfrc_inverse; reconstructed, not sensed torque",
              "splits": {"train": list(TRAJECTORIES[:7]), "validation": ["T12"],
                         "test": ["T13", "T14"], "diagnostic": ["T4"]},
              "metrics": {}, "trajectory_metrics": {}}
    train = data["split"] == "train"
    train_bounds = {key: {"min": data[key][train].min(axis=0).tolist(),
                          "max": data[key][train].max(axis=0).tolist()}
                    for key in ("q", "dq", "ddq")}
    report["train_feature_bounds"] = train_bounds
    for split in ("train", "validation", "test", "diagnostic"):
        mask = data["split"] == split
        if not np.any(mask):
            raise ValueError(f"missing {split} data")
        metrics = {name: _metrics(reference[mask], values[mask])
                   for name, values in predictions.items()}
        baseline = metrics["nominal_rbd"]["rmse_nm"]
        metrics["new_pkan_error_reduction_pct"] = 100 * (baseline - metrics["dynamics10_pkan"]["rmse_nm"]) / baseline
        report["metrics"][split] = {"samples": int(mask.sum()), "models": metrics}
    for trajectory in (*TRAJECTORIES, "T4"):
        mask = data["trajectory_id"] == trajectory
        metrics = {name: _metrics(reference[mask], values[mask])
                   for name, values in predictions.items()}
        baseline = metrics["nominal_rbd"]["rmse_nm"]
        metrics["new_pkan_error_reduction_pct"] = 100 * (baseline - metrics["dynamics10_pkan"]["rmse_nm"]) / baseline
        report["trajectory_metrics"][trajectory] = {
            "split": SPLIT[trajectory], "samples": int(mask.sum()), "models": metrics,
            "outside_train_bounds_fraction": {
                key: float(np.mean(np.any(
                    (data[key][mask] < train_bounds[key]["min"]) |
                    (data[key][mask] > train_bounds[key]["max"]), axis=1)))
                for key in ("q", "dq", "ddq")}}

    with (run / "predictions.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        columns = ["trajectory_id", "trial_id", "split", "time_s"]
        for name in ("tau_reference", *MODEL_ORDER):
            columns += [f"{name}_j{i}_nm" for i in range(1, 7)]
        writer.writerow(columns)
        for i in range(len(reference)):
            writer.writerow([data["trajectory_id"][i], data["trial_id"][i],
                             data["split"][i], data["time_s"][i], *reference[i],
                             *(value for name in MODEL_ORDER for value in predictions[name][i])])
    (run / "metrics.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    _plot(report, data, predictions, run)
    return report


def _plot(report: dict, data: dict, predictions: dict, run: Path) -> None:
    figures = run / "figures"
    figures.mkdir(exist_ok=True)
    trajectories = TRAJECTORIES
    fig, ax = plt.subplots(figsize=(13, 5))
    x = np.arange(len(trajectories))
    names = MODEL_ORDER[:4]
    width = .2
    for j, name in enumerate(names):
        values = [report["trajectory_metrics"][t]["models"][name]["rmse_nm"] for t in trajectories]
        ax.bar(x+(j-1.5)*width, values, width, label=LABELS[name], color=COLORS[name])
    ax.axvline(6.5, linestyle=":", color="black")
    ax.axvline(7.5, linestyle=":", color="black")
    ax.set_xticks(x, trajectories); ax.set_ylabel("Six-joint torque RMSE (Nm)")
    ax.set_title("AG145 MuJoCo | Train T5-T11 · Validation T12 · Test T13-T14")
    ax.legend(ncol=2, fontsize=8); ax.grid(axis="y", alpha=.25)
    fig.tight_layout(); fig.savefig(figures / "trajectory_rmse.png", dpi=155); plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(14, 4), sharey=True)
    for ax, trajectory in zip(axes, ("T12", "T13", "T14")):
        metrics = report["trajectory_metrics"][trajectory]["models"]
        x = np.arange(6)
        for j, name in enumerate(names):
            ax.bar(x+(j-1.5)*width, metrics[name]["joint_rmse_nm"], width,
                   color=COLORS[name], label=LABELS[name])
        ax.set_xticks(x, [f"J{i}" for i in range(1, 7)])
        ax.set_title(f"{trajectory} ({report['trajectory_metrics'][trajectory]['split']})")
        ax.set_xlabel("Joint"); ax.grid(axis="y", alpha=.25)
    axes[0].set_ylabel("Torque RMSE (Nm)")
    axes[-1].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(figures / "heldout_joint_rmse.png", dpi=155); plt.close(fig)

    for trajectory in ("T12", "T13", "T14"):
        indices = np.flatnonzero(data["trajectory_id"] == trajectory)
        time = data["time_s"][indices]
        reference = data["tau_reference"][indices]
        # Show the joint with the largest nominal mismatch, selected by the
        # physics baseline before looking at learned model errors.
        joint = int(np.argmax(np.sqrt(np.mean((predictions["nominal_rbd"][indices]-reference)**2, axis=0))))
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        for name, values in (("reference", reference), *[(n, predictions[n][indices]) for n in names]):
            color = "black" if name == "reference" else COLORS[name]
            axes[0].plot(time, values[:, joint], label="MuJoCo reference" if name == "reference" else LABELS[name],
                         color=color, linewidth=1.3)
        for name in names:
            axes[1].plot(time, predictions[name][indices, joint]-reference[:, joint],
                         label=LABELS[name], color=COLORS[name], linewidth=1.2)
        axes[0].set_ylabel(f"J{joint+1} torque (Nm)")
        axes[1].set_ylabel(f"J{joint+1} prediction error (Nm)")
        axes[1].set_xlabel("ROS simulation time from trial start (s)")
        axes[0].legend(ncol=2, fontsize=8)
        for ax in axes: ax.grid(alpha=.25)
        fig.suptitle(f"{trajectory} | AG145 | largest nominal-error joint J{joint+1}")
        fig.tight_layout(); fig.savefig(figures / f"{trajectory}_torque_prediction.png", dpi=155); plt.close(fig)

    history = json.loads((run / "training.json").read_text(encoding="utf-8"))
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    for ax, name in zip(axes, ("pkan", "mlp")):
        item = history["runs"][name]
        ax.plot([row["epoch"] for row in item["history"]],
                [row["train_loss"] for row in item["history"]], label="Train")
        ax.plot([row["epoch"] for row in item["history"]],
                [row["validation_loss"] for row in item["history"]], label="T12 validation")
        ax.axvline(item["best_epoch"], color="gray", linestyle=":", label="Best")
        ax.set_title(name.upper()); ax.set_xlabel("Epoch"); ax.set_ylabel("Normalized loss")
        ax.set_yscale("log"); ax.grid(alpha=.25); ax.legend()
    fig.tight_layout(); fig.savefig(figures / "training_loss.png", dpi=155); plt.close(fig)


def main(argv: list[str] | None = None) -> int:
    root = REPO_ROOT / "model"
    parser = argparse.ArgumentParser(description="Evaluate dynamics10 residual PKAN")
    parser.add_argument("--dataset", default=str(root / "residual_data" / "dynamics10_v1"))
    parser.add_argument("--run", default=str(root / "residual_runs" / "dynamics10_v1"))
    parser.add_argument("--previous", default=str(root / "residual_runs" / "rbd_v1" / "best_pkan.pt"))
    args = parser.parse_args(argv)
    report = evaluate(args.dataset, args.run, args.previous)
    for trajectory, row in report["trajectory_metrics"].items():
        models = row["models"]
        print(trajectory, row["split"], row["samples"],
              "nominal", round(models["nominal_rbd"]["rmse_nm"], 4),
              "previous", round(models["previous_pkan"]["rmse_nm"], 4),
              "new", round(models["dynamics10_pkan"]["rmse_nm"], 4))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
