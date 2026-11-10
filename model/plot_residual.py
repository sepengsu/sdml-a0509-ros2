"""Publication-style figures for the measured offline residual-RBD study."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


COLORS = {"nominal_rbd": "#d55e00", "residual_pkan": "#2271b2",
          "tool_aware_rbd": "#009e73", "residual_mlp": "#aa3377"}
LABELS = {"nominal_rbd": "Nominal RBD", "residual_pkan": "RBD + Residual KAN",
          "tool_aware_rbd": "Tool-aware RBD", "residual_mlp": "RBD + Residual MLP"}


def _rows(path: Path, split: str, tool: str) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [row for row in csv.DictReader(stream)
                if row["split"] == split and row["tool_condition"] == tool]


def _series(rows: list[dict], stem: str, joint: int) -> np.ndarray:
    return np.asarray([float(row[f"{stem}_{joint}_nm"]) for row in rows])


def plot(run: str | Path) -> list[Path]:
    run = Path(run)
    metrics = json.loads((run / "metrics.json").read_text(encoding="utf-8"))
    training = json.loads((run / "training.json").read_text(encoding="utf-8"))
    predictions = run / "predictions.csv"
    figures = run / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    output = []

    # Figure 1: torque time histories on genuinely new analytic states.
    rows = _rows(predictions, "test_new_trial", "ag145")
    time = np.asarray([float(row["time_s"]) for row in rows])
    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for ax, joint in zip(axes, (2, 3)):
        for stem, label, color, style in (
            ("tau_reference", "MuJoCo reference", "black", "-"),
            ("tau_nominal", "Nominal RBD", COLORS["nominal_rbd"], "--"),
            ("tau_pkan", "RBD + Residual KAN", COLORS["residual_pkan"], ":"),
        ):
            ax.plot(time, _series(rows, stem, joint), style, color=color, label=label, linewidth=1.6)
        ax.set_ylabel(f"J{joint} torque (Nm)")
        ax.grid(alpha=0.25)
    axes[0].legend(ncol=3)
    axes[1].set_xlabel("Analytic independent trial time (s)")
    fig.suptitle("Tool AG145: reference, nominal RBD and residual KAN")
    fig.tight_layout()
    path = figures / "torque_comparison.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    output.append(path)

    # Figure 2: all six joints, including the held-out T4 failure case.
    compared = [name for name in ("nominal_rbd", "residual_pkan", "residual_mlp")
                if name in metrics["splits"]["test_new_trial"]["ag145"]["models"]]
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.7), sharey=True)
    for ax, split, title in zip(axes, ("test_new_trial", "test_unseen_trajectory"),
                                ("New joint trial / AG145", "Unseen T4 / AG145")):
        models = metrics["splits"][split]["ag145"]["models"]
        x = np.arange(6)
        width = 0.72 / len(compared)
        for offset, name in enumerate(compared):
            ax.bar(x + (offset - (len(compared) - 1) / 2) * width,
                   models[name]["joint_rmse_nm"], width,
                   label=LABELS[name], color=COLORS[name])
        ax.set_xticks(x, [f"J{i}" for i in range(1, 7)])
        ax.set_title(title)
        ax.set_xlabel("Joint")
        ax.grid(axis="y", alpha=0.25)
    axes[0].set_ylabel("Torque RMSE (Nm)")
    axes[1].legend(fontsize=8)
    fig.tight_layout()
    path = figures / "joint_rmse.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    output.append(path)

    # Figure 3: learned and target residual separately.
    fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    for ax, joint in zip(axes, (2, 3)):
        ax.plot(time, _series(rows, "delta_tau_target", joint), color="black",
                label="Reference - Nominal RBD")
        ax.plot(time, _series(rows, "delta_tau_pkan", joint), "--",
                color=COLORS["residual_pkan"], label="KAN residual")
        ax.set_ylabel(f"J{joint} residual (Nm)")
        ax.grid(alpha=0.25)
    axes[0].legend()
    axes[1].set_xlabel("Analytic independent trial time (s)")
    fig.tight_layout()
    path = figures / "residual_torque.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    output.append(path)

    # Figure 4: the generalization boundary and model mismatch severity.
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.7))
    groups = (("test_new_trial", "New trial"), ("test_unseen_trajectory", "Unseen T4"),
              ("test_unseen_tool", "Unseen tool"))
    x = np.arange(len(groups))
    for index, name in enumerate(compared):
        values = [metrics["splits"][split]["all"]["models"][name]["rmse_nm"] for split, _ in groups]
        width = 0.72 / len(compared)
        axes[0].bar(x + (index - (len(compared) - 1) / 2) * width, values, width,
                    label=LABELS[name], color=COLORS[name])
    axes[0].set_xticks(x, [label for _, label in groups])
    axes[0].set_ylabel("Torque RMSE (Nm)")
    axes[0].set_title("Generalization by held-out condition")
    axes[0].legend(fontsize=8)
    axes[0].grid(axis="y", alpha=0.25)
    factors = (0.6, 0.7, 0.8, 0.9, 1.0)
    for name in compared:
        values = [metrics["splits"]["test_mismatch_severity"][f"factor_{f:g}"]["models"][name]["rmse_nm"]
                  for f in factors]
        axes[1].plot(factors, values, marker="o", label=LABELS[name], color=COLORS[name])
    axes[1].set_xlabel("Nominal tool mass / reference mass")
    axes[1].set_ylabel("T4 torque RMSE (Nm)")
    axes[1].set_title("Unseen T4: nominal parameter error")
    axes[1].grid(alpha=0.25)
    fig.tight_layout()
    path = figures / "generalization.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    output.append(path)

    # Figure 5: training and temporal-validation losses.
    run_names = list(training["runs"])
    fig, axes = plt.subplots(1, len(run_names), figsize=(6 * len(run_names), 4.5))
    for ax, name in zip(np.atleast_1d(axes), run_names):
        history = training["runs"][name]["history"]
        ax.plot([row["epoch"] for row in history], [row["train_loss"] for row in history], label="Train")
        ax.plot([row["epoch"] for row in history], [row["validation_loss"] for row in history], label="Validation")
        ax.axvline(training["runs"][name]["best_epoch"], color="gray", linestyle=":", label="Best")
        ax.set_title(name.upper())
        ax.set_xlabel("Epoch")
        ax.set_ylabel("Normalized loss")
        ax.set_yscale("log")
        ax.grid(alpha=0.25)
        ax.legend()
    fig.tight_layout()
    path = figures / "loss_curve.png"
    fig.savefig(path, dpi=170)
    plt.close(fig)
    output.append(path)
    return output


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Plot A0509 residual RBD study")
    parser.add_argument("--run", default=str(Path(__file__).resolve().parent / "residual_runs/rbd_v1"))
    args = parser.parse_args(argv)
    for path in plot(args.run):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
