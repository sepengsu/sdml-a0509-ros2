"""Render held-out M0-M4 prediction figures and performance tables."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .evaluate_joint_models import NAMES


MODEL_IDS = tuple(NAMES)
COLORS = {"M0": "#59636E", "M1": "#1967AD", "M2": "#DE8F05",
          "M3": "#158F78", "M4": "#8D5AAF"}


def _render_table(report: dict, output: Path) -> None:
    test = report["scores"]["test"]
    conditions = tuple(test["by_condition"])
    rows = [[condition, *[f'{test["by_condition"][condition][model]["torque_rmse_nm"]:.3f}'
                            for model in MODEL_IDS]] for condition in conditions]
    rows.append(["ALL TEST", *[f'{test["all"][model]["torque_rmse_nm"]:.3f}'
                               for model in MODEL_IDS]])
    fig, ax = plt.subplots(figsize=(13, 5.5))
    ax.axis("off")
    table = ax.table(cellText=rows, colLabels=["Condition", *MODEL_IDS],
                     cellLoc="center", colLoc="center", bbox=[0, 0.05, 1, 0.86],
                     colWidths=[0.28, 0.144, 0.144, 0.144, 0.144, 0.144])
    table.auto_set_font_size(False)
    table.set_fontsize(12)
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor("#8A99A8")
        if row == 0:
            cell.set_facecolor("#E3ECF5")
            cell.set_text_props(weight="bold", color="#18334B")
        elif row == len(rows):
            cell.set_facecolor("#D4E5F1")
            cell.set_text_props(weight="bold")
        elif row % 2 == 0:
            cell.set_facecolor("#F6F8FA")
    fig.suptitle(f'{report["profile"]} | held-out test torque RMSE [N·m]',
                 fontsize=18, weight="bold")
    fig.text(0.5, 0.01, "M0 Nominal RBD  ·  M1 Direct MLP  ·  M2 Direct KAN  ·  "
             "M3 RBD+MLP  ·  M4 RBD+KAN", ha="center", fontsize=10)
    fig.savefig(output / "test_performance_table.png", dpi=220, bbox_inches="tight")
    plt.close(fig)


def _render_condition_bars(report: dict, output: Path) -> None:
    conditions = tuple(report["scores"]["test"]["by_condition"])
    values = np.asarray([[report["scores"]["test"]["by_condition"][name][model]["torque_rmse_nm"]
                          for model in MODEL_IDS] for name in conditions])
    x = np.arange(len(conditions))
    fig, axes = plt.subplots(2, 1, figsize=(14, 9), layout="constrained")
    for ax, chosen, title in ((axes[0], ("M1", "M2"), "Direct learned models"),
                              (axes[1], ("M0", "M3", "M4"), "Physics baseline and residual models")):
        width = 0.22
        for index, model in enumerate(chosen):
            offset = (index - (len(chosen) - 1) / 2) * width
            ax.bar(x + offset, values[:, MODEL_IDS.index(model)], width=width,
                   label=f"{model} {NAMES[model]}", color=COLORS[model])
        ax.set_xticks(x, conditions, rotation=20, ha="right")
        ax.set_ylabel("Torque RMSE (N·m)")
        ax.set_title(title, fontsize=13)
        ax.grid(axis="y", alpha=0.25)
        ax.set_axisbelow(True)
        ax.spines[["top", "right"]].set_visible(False)
        ax.legend(ncol=len(chosen), fontsize=9, frameon=False)
    fig.suptitle(f'{report["profile"]} | held-out error by condition',
                 fontsize=17, weight="bold")
    fig.savefig(output / "test_rmse_by_condition.png", dpi=220)
    plt.close(fig)


def _render_trajectory_bars(report: dict, output: Path) -> None:
    trajectories = tuple(report["scores"]["test"]["by_trajectory"])
    values = np.asarray([[report["scores"]["test"]["by_trajectory"][name][model]["torque_rmse_nm"]
                          for model in MODEL_IDS] for name in trajectories])
    x = np.arange(len(trajectories))
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), layout="constrained")
    for ax, chosen, title in ((axes[0], ("M1", "M2"), "Direct models"),
                              (axes[1], ("M0", "M3", "M4"), "Physics and residual models")):
        width = 0.22
        for index, model in enumerate(chosen):
            offset = (index - (len(chosen) - 1) / 2) * width
            ax.bar(x + offset, values[:, MODEL_IDS.index(model)], width=width,
                   color=COLORS[model], label=model)
        ax.set_xticks(x, trajectories)
        ax.set_ylabel("Torque RMSE (N·m)")
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.25)
        ax.set_axisbelow(True)
        ax.spines[["top", "right"]].set_visible(False)
        ax.legend(frameon=False)
    fig.suptitle(f'{report["profile"]} | held-out error by trajectory',
                 fontsize=17, weight="bold")
    fig.savefig(output / "test_rmse_by_trajectory.png", dpi=220)
    plt.close(fig)


def _render_trace(predictions_path: Path, manifest: dict, output: Path,
                  condition: str, trajectory: str) -> None:
    with np.load(predictions_path, allow_pickle=False) as raw:
        data = {key: raw[key] for key in ("tau_reference", "condition", "trajectory", *MODEL_IDS)}
    ci = manifest["conditions"].index(condition)
    mask = ((data["condition"] == ci) & (data["trajectory"] == int(trajectory[1:])))
    if not mask.any():
        raise ValueError(f"test prediction missing: {condition}/{trajectory}")
    time_s = np.arange(int(mask.sum())) * manifest["stride"] / 100.0
    fig, axes = plt.subplots(3, 2, figsize=(15, 10), sharex=True, layout="constrained")
    for joint, ax in enumerate(axes.flat):
        ax.plot(time_s, data["tau_reference"][mask, joint], color="black", linewidth=1.7,
                label="MuJoCo label")
        for model in MODEL_IDS:
            ax.plot(time_s, data[model][mask, joint], color=COLORS[model],
                    linewidth=1.05, alpha=0.85, label=model)
        ax.set_title(f"Joint {joint + 1}")
        ax.set_ylabel("Torque (N·m)")
        ax.grid(alpha=0.22)
        ax.spines[["top", "right"]].set_visible(False)
    for ax in axes[-1]:
        ax.set_xlabel("Simulation time (s)")
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=6, bbox_to_anchor=(0.5, 0.96),
               frameon=False)
    fig.suptitle(f'{manifest["profile"]} | held-out torque prediction | {condition} / {trajectory}',
                 fontsize=17, weight="bold")
    fig.savefig(output / f"test_predictions_{condition}_{trajectory}.png", dpi=220)
    plt.close(fig)


def _write_tables(report: dict, model_root: Path, output: Path) -> None:
    fields = ("model_id", "model_name", "test_samples", "test_rmse_nm",
              "test_mae_nm", "validation_rmse_nm", "best_epoch")
    with (output / "overall_performance.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for model in MODEL_IDS:
            test = report["scores"]["test"]["all"][model]
            val = report["scores"]["validation"]["all"][model]
            training = (json.loads((model_root / model / "training.json").read_text(encoding="utf-8"))
                        if model != "M0" else {})
            writer.writerow({"model_id": model, "model_name": NAMES[model],
                             "test_samples": test["samples"], "test_rmse_nm": test["torque_rmse_nm"],
                             "test_mae_nm": test["torque_mae_nm"],
                             "validation_rmse_nm": val["torque_rmse_nm"],
                             "best_epoch": training.get("best_epoch", "")})


def render(data_root: Path, model_root: Path, evaluation_root: Path) -> None:
    output = evaluation_root / "figures"
    output.mkdir(parents=True, exist_ok=True)
    report = json.loads((evaluation_root / "metrics.json").read_text(encoding="utf-8"))
    manifest = json.loads((data_root / "manifest.json").read_text(encoding="utf-8"))
    if report["dataset_sha256"] != manifest["dataset_sha256"]:
        raise ValueError("evaluation and dataset provenance differ")
    _write_tables(report, model_root, evaluation_root)
    _render_table(report, output)
    _render_condition_bars(report, output)
    _render_trajectory_bars(report, output)
    _render_trace(evaluation_root / "test_predictions.npz", manifest, output,
                  "ag145_p05", "T16")
    print(f"Wrote tables and prediction figures to {evaluation_root}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--evaluation", type=Path, required=True)
    args = parser.parse_args()
    render(args.data, args.models, args.evaluation)


if __name__ == "__main__":
    main()
