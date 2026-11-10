"""Plot PKAN validation and continual-learning ablation results."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .train import TASKS


def plot_results(run_dir: str | Path, ablation_dir: str | Path) -> tuple[Path, Path]:
    run_dir = Path(run_dir)
    ablation_dir = Path(ablation_dir)
    main = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
    ablation = json.loads((ablation_dir / "metrics.json").read_text(encoding="utf-8"))
    figures = run_dir / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    colors = dict(zip(TASKS, ("#2271b2", "#d55e00", "#009e73", "#aa3377")))
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), sharey=True)
    for ax, report, title in ((axes[0], main, "Replay + EWC"),
                              (axes[1], ablation, "No continual learning")):
        for task in TASKS:
            values = [report["stages"][stage][task]["torque_rmse_nm"] for stage in TASKS]
            ax.plot(TASKS, values, marker="o", color=colors[task], label=task, linewidth=2)
        ax.set_yscale("log")
        ax.set_ylim(0.03, 20)
        ax.set_xlabel("Completed task")
        ax.set_title(title)
        ax.grid(True, which="both", alpha=0.25)
    axes[0].set_ylabel("Held-out torque RMSE (Nm)")
    axes[1].legend(title="Validated trajectory", ncol=2)
    fig.suptitle("A0509 + AG145 MuJoCo PKAN: T1 → T4")
    fig.tight_layout()
    continual_path = figures / "continual_learning.png"
    fig.savefig(continual_path, dpi=170)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(12, 6), sharex=False)
    for ax, task, joint in ((axes[0], "T3", 2), (axes[1], "T4", 6)):
        with (run_dir / f"{task}_validation_predictions.csv").open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        time = [float(row["ros_time_s"]) - float(rows[0]["ros_time_s"]) for row in rows]
        actual = [float(row[f"tau_true_{joint}"]) for row in rows]
        predicted = [float(row[f"tau_pred_{joint}"]) for row in rows]
        ax.plot(time, actual, label="MuJoCo inverse dynamics", color="black", linewidth=1.3)
        ax.plot(time, predicted, label="PKAN prediction", color=colors[task], linestyle="--")
        ax.set_title(f"{task} held-out interval — joint {joint}")
        ax.set_ylabel("Torque (Nm)")
        ax.grid(True, alpha=0.25)
    axes[0].legend()
    axes[1].set_xlabel("Time from validation start (s)")
    fig.tight_layout()
    torque_path = figures / "torque_validation.png"
    fig.savefig(torque_path, dpi=170)
    plt.close(fig)

    comparison = {task: {
        "replay_ewc_rmse_nm": main["final_validation"][task]["torque_rmse_nm"],
        "no_continual_rmse_nm": ablation["final_validation"][task]["torque_rmse_nm"],
        "fixed_T1_baseline_rmse_nm": main["fixed_T1_baseline"][task]["torque_rmse_nm"],
    } for task in TASKS}
    (run_dir / "comparison.json").write_text(json.dumps(comparison, indent=2) + "\n", encoding="utf-8")
    return continual_path, torque_path


def main(argv: list[str] | None = None) -> int:
    parent = Path(__file__).resolve().parent / "runs"
    parser = argparse.ArgumentParser(description="Plot AG145 PKAN validation and ablation")
    parser.add_argument("--run-dir", default=str(parent / "ag145_pkan"))
    parser.add_argument("--ablation-dir", default=str(parent / "ablation_no_continual"))
    args = parser.parse_args(argv)
    for path in plot_results(args.run_dir, args.ablation_dir):
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
