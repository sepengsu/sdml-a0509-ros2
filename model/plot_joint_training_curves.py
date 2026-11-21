"""Render one loss figure and one held-out-test RMSE bar figure."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .evaluate_joint_models import NAMES


COLORS = {"M1": "#1967AD", "M2": "#DE8F05", "M3": "#158F78", "M4": "#8D5AAF"}
MODEL_IDS = ("M1", "M2", "M3", "M4")


def render(model_root: Path, evaluation_root: Path) -> tuple[Path, Path]:
    report = json.loads((evaluation_root / "metrics.json").read_text(encoding="utf-8"))
    if Path(report["model_root"]).resolve() != model_root.resolve():
        raise ValueError("evaluation belongs to a different model run")
    histories = {}
    for model_id in MODEL_IDS:
        training = json.loads((model_root / model_id / "training.json").read_text(encoding="utf-8"))
        if training["dataset_sha256"] != report["dataset_sha256"]:
            raise ValueError(f"training/evaluation dataset mismatch: {model_id}")
        histories[model_id] = training

    output = evaluation_root / "figures"
    output.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False,
                         "axes.spines.right": False, "savefig.facecolor": "white"})

    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5), sharex=True)
    for ax, model_id in zip(axes.flat, MODEL_IDS):
        record = histories[model_id]
        history = record["history"]
        epochs = np.asarray([item["epoch"] for item in history])
        train_loss = np.asarray([item["train_normalized_mse"] for item in history])
        val_loss = np.asarray([item["validation_normalized_mse"] for item in history])
        best_epoch = record["best_epoch"]
        ax.plot(epochs, train_loss, color=COLORS[model_id], lw=1.8, label="Train")
        ax.plot(epochs, val_loss, color=COLORS[model_id], lw=1.8, ls="--", label="Validation")
        ax.scatter([best_epoch], [val_loss[best_epoch - 1]], s=52, facecolors="white",
                   edgecolors=COLORS[model_id], linewidths=1.7, zorder=4)
        if record.get("stop_reason") == "early_stopping":
            ax.axvline(epochs[-1], color="#626B73", ls=":", lw=1.2)
        ax.set_yscale("log")
        ax.set_title(f"{model_id}  {NAMES[model_id]}  |  best {best_epoch}, end {epochs[-1]}")
        ax.set_ylabel("Normalized MSE")
        ax.grid(True, which="both", color="#D9DFE3", lw=0.5, alpha=0.7)
        ax.legend(loc="upper right", frameon=False)
    for ax in axes[-1]:
        ax.set_xlabel("Epoch")
    params = histories["M1"]["parameters"]
    fig.suptitle(f"{report['profile']} | training and validation loss | "
                 f"{params['epochs_requested']} max epochs, batch {params['batch_size']}, "
                 f"patience {params['patience']}",
                 fontsize=15, weight="bold")
    fig.text(0.5, 0.015, "Loss is normalized using each model's training-target scale; compare trends within each panel.",
             ha="center", color="#58636C", fontsize=9)
    fig.tight_layout(rect=(0, 0.035, 1, 0.94))
    loss_path = output / "training_loss.png"
    fig.savefig(loss_path, dpi=180)
    plt.close(fig)

    ids = ("M0", *MODEL_IDS)
    scores = [report["scores"]["test"]["all"][model_id]["torque_rmse_nm"] for model_id in ids]
    colors = ["#59636E", *(COLORS[model_id] for model_id in MODEL_IDS)]
    fig, ax = plt.subplots(figsize=(11, 8.5))
    positions = np.arange(len(ids))
    bars = ax.bar(positions, scores, color=colors, width=0.66, edgecolor="white", linewidth=1)
    ax.set_xticks(positions, [f"{model_id}\n{NAMES[model_id]}" for model_id in ids])
    ax.set_yscale("log")
    ax.set_ylim(min(scores) * 0.5, max(scores) * 2.0)
    ax.set_ylabel("Held-out test torque RMSE [Nm]  ·  log scale")
    ax.set_title(f"{report['profile']} | M0–M4 held-out test RMSE", fontsize=15, weight="bold")
    ax.grid(True, axis="y", which="both", color="#D9DFE3", lw=0.6, alpha=0.8)
    ax.set_axisbelow(True)
    for bar, score in zip(bars, scores):
        ax.text(bar.get_x() + bar.get_width() / 2, score * 1.12,
                f"{score:.4f}", va="bottom", ha="center", fontsize=10, weight="bold")
    test_rows = report["scores"]["test"]["all"]["M0"]["samples"]
    fig.text(0.5, 0.015, f"{test_rows:,} held-out rows × 6 joints; MuJoCo inverse-dynamics torque labels.",
             ha="center", color="#58636C", fontsize=9)
    fig.tight_layout(rect=(0, 0.035, 1, 1))
    rmse_path = output / "test_rmse_vertical_bar.png"
    fig.savefig(rmse_path, dpi=180)
    plt.close(fig)
    return loss_path, rmse_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--evaluation", type=Path, required=True)
    args = parser.parse_args()
    paths = render(args.models, args.evaluation)
    for path in paths:
        print(path.resolve())


if __name__ == "__main__":
    main()
