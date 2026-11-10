"""Visualize extracted PKAN inertia and torque decomposition."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .data import TRIAL_IDS


def plot_export(folder: str | Path, trajectory: str) -> Path:
    if trajectory not in TRIAL_IDS:
        raise ValueError("trajectory must be T1, T2, T3 or T4")
    folder = Path(folder)
    with (folder / f"{trajectory}_dynamics.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"empty dynamics export for {trajectory}")
    time = np.asarray([float(row["ros_time_s"]) for row in rows])
    time -= time[0]
    mass = np.asarray([[[float(row[f"M_{i}{j}_kg_m2"]) for j in range(1, 7)]
                        for i in range(1, 7)] for row in rows])
    joint = 6 if trajectory == "T4" else 2
    fig = plt.figure(figsize=(13, 7))
    grid = fig.add_gridspec(2, 2, height_ratios=(1, 1.2))
    ax_mass = fig.add_subplot(grid[0, 0])
    ax_matrix = fig.add_subplot(grid[0, 1])
    ax_torque = fig.add_subplot(grid[1, :])
    for i in range(6):
        ax_mass.plot(time, mass[:, i, i], label=f"M{i+1}{i+1}")
    ax_mass.set_title("Learned mass matrix diagonal")
    ax_mass.set_ylabel("Generalized inertia (kg m²)")
    ax_mass.set_xlabel("Simulation time from first sample (s)")
    ax_mass.legend(ncol=3, fontsize=8)
    ax_mass.grid(True, alpha=0.25)
    midpoint = mass[len(mass) // 2]
    limit = float(np.max(np.abs(midpoint)))
    image = ax_matrix.imshow(midpoint, cmap="coolwarm", vmin=-limit, vmax=limit)
    ax_matrix.set_title("M(q) at midpoint")
    ax_matrix.set_xticks(range(6), [f"J{i}" for i in range(1, 7)])
    ax_matrix.set_yticks(range(6), [f"J{i}" for i in range(1, 7)])
    fig.colorbar(image, ax=ax_matrix, label="kg m²")
    for stem, label, style in (
        ("tau_mujoco", "MuJoCo inverse torque", "-"),
        ("tau_pred", "PKAN total torque", "--"),
        ("zero_velocity_bias", "Zero-velocity bias", ":"),
        ("velocity_bias", "Velocity bias", ":"),
        ("damping_torque", "Damping", "-."),
        ("inertia_torque", "Inertia", ":"),
    ):
        values = [float(row[f"{stem}_{joint}_nm"]) for row in rows]
        ax_torque.plot(time, values, style, label=label, linewidth=1.3)
    ax_torque.set_title(f"Joint {joint} torque decomposition")
    ax_torque.set_xlabel("Simulation time from first sample (s)")
    ax_torque.set_ylabel("Torque (Nm)")
    ax_torque.legend(ncol=3, fontsize=9)
    ax_torque.grid(True, alpha=0.25)
    fig.suptitle(f"A0509 + AG145 PKAN — {trajectory}")
    fig.tight_layout()
    output = folder / "figures" / f"{trajectory}_decomposition.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170)
    plt.close(fig)
    return output


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Plot AG145 PKAN dynamic terms")
    parser.add_argument("--folder", default=str(Path(__file__).resolve().parent / "exports" / "ag145_pkan"))
    parser.add_argument("--trajectory", choices=(*TRIAL_IDS, "all"), default="all")
    args = parser.parse_args(argv)
    selected = TRIAL_IDS if args.trajectory == "all" else (args.trajectory,)
    for task in selected:
        print(plot_export(args.folder, task))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
