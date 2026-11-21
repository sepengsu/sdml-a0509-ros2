"""Plot one recorded grid robot's joint state and simulator torque.

Example:
  python scripts/plot_grid_robot_dynamics.py \
    data/grid_16_wide_v3/simulation/T16/bare_p0/trial_001/dynamics.csv \
    results/wide16_v3_t16_bare_dynamics
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


COLORS = ("#1768AC", "#E4572E", "#2A9D8F", "#8E5EA2", "#C98C16", "#59656F")
JOINTS = tuple(f"J{i}" for i in range(1, 7))


def read_dynamics(path: Path) -> dict[str, np.ndarray]:
    fields = ("q", "dq", "ddq", "target_q")
    data: dict[str, list] = {name: [] for name in (*fields, "tau", "bias", "time")}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            data["time"].append(float(row["ros_time_s"]))
            for name in fields:
                data[name].append([float(row[f"{name}{joint}"]) for joint in range(1, 7)])
            data["tau"].append(json.loads(row["qfrc_inverse"]))
            data["bias"].append(json.loads(row["qfrc_bias"]))
    arrays = {name: np.asarray(values, dtype=float) for name, values in data.items()}
    if len(arrays["time"]) < 2 or not all(np.isfinite(x).all() for x in arrays.values()):
        raise ValueError("Expected at least two complete, finite dynamics rows")
    return arrays


def style() -> None:
    plt.rcParams.update({
        "font.family": "DejaVu Sans",
        "font.size": 10,
        "axes.titlesize": 12,
        "axes.labelsize": 10,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "grid.alpha": 0.22,
        "grid.linewidth": 0.7,
        "savefig.facecolor": "white",
    })


def position_plot(data: dict[str, np.ndarray], out: Path, label: str) -> None:
    fig, axes = plt.subplots(6, 1, figsize=(14, 11), sharex=True, constrained_layout=True)
    t = data["time"]
    for joint, ax in enumerate(axes):
        ax.plot(t, data["target_q"][:, joint], color="#888888", linewidth=1.8,
                linestyle="--", label="Target")
        ax.plot(t, data["q"][:, joint], color=COLORS[joint], linewidth=1.0,
                label="Recorded")
        ax.set_ylabel(f"{JOINTS[joint]} (rad)")
        ax.set_xlim(t[0], t[-1])
    axes[0].legend(loc="upper right", ncol=2, frameon=False)
    axes[-1].set_xlabel("Simulation time (s)")
    fig.suptitle(f"{label}  |  Joint position", fontsize=17, fontweight="bold")
    fig.savefig(out / "01_joint_position.png", dpi=220)
    plt.close(fig)


def derivatives_plot(data: dict[str, np.ndarray], out: Path, label: str) -> None:
    fig = plt.figure(figsize=(14, 9), constrained_layout=True)
    grid = fig.add_gridspec(3, 1, height_ratios=(1.2, 1.2, 0.8))
    velocity, acceleration, initial = (fig.add_subplot(grid[i]) for i in range(3))
    t = data["time"]
    for joint in range(6):
        velocity.plot(t, data["dq"][:, joint], color=COLORS[joint], linewidth=1.1,
                      label=JOINTS[joint])
        acceleration.plot(t[1:], data["ddq"][1:, joint], color=COLORS[joint], linewidth=1.1)
    velocity.set(xlabel="Simulation time (s)", ylabel="Velocity (rad/s)",
                 title="Joint velocity | complete recording", xlim=(t[0], t[-1]))
    velocity.legend(loc="upper right", ncol=6, frameon=False)
    acceleration.set(xlabel="Simulation time (s)", ylabel="Acceleration (rad/s²)",
                     title="Joint acceleration | after initial sample", xlim=(t[1], t[-1]))
    initial.bar(JOINTS, data["ddq"][0], color=COLORS, width=0.55)
    initial.axhline(0, color="#5C6770", linewidth=0.8)
    initial.set(xlabel="Joint", ylabel="Acceleration (rad/s²)",
                title=f"Initial sample at t = {t[0]:g} s | shown separately to preserve the motion scale")
    fig.suptitle(f"{label}  |  Joint velocity and acceleration", fontsize=17, fontweight="bold")
    fig.savefig(out / "02_joint_velocity_acceleration.png", dpi=220)
    plt.close(fig)


def torque_plot(data: dict[str, np.ndarray], out: Path, label: str) -> None:
    fig, axes = plt.subplots(6, 1, figsize=(14, 11), sharex=True, constrained_layout=True)
    t = data["time"]
    for joint, ax in enumerate(axes):
        ax.plot(t, data["tau"][:, joint], color=COLORS[joint], linewidth=1.0,
                label="Inverse dynamics torque")
        ax.plot(t, data["bias"][:, joint], color="#343A40", linewidth=1.0,
                linestyle="--", alpha=0.75, label="Bias (gravity + velocity terms)")
        ax.set_ylabel(f"{JOINTS[joint]} (N·m)")
        ax.set_xlim(t[0], t[-1])
    axes[0].legend(loc="upper right", ncol=2, frameon=False)
    axes[-1].set_xlabel("Simulation time (s)")
    fig.suptitle(f"{label}  |  Simulator inverse dynamics torque", fontsize=17,
                 fontweight="bold")
    fig.savefig(out / "03_inverse_dynamics_torque.png", dpi=220)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--label", default="Bare · T16 · trial 001")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    data = read_dynamics(args.csv)
    style()
    position_plot(data, args.output, args.label)
    derivatives_plot(data, args.output, args.label)
    torque_plot(data, args.output, args.label)
    tracking = np.abs(data["q"] - data["target_q"])
    print(json.dumps({
        "samples": len(data["time"]),
        "duration_s": float(data["time"][-1] - data["time"][0]),
        "max_tracking_error_rad": float(tracking.max()),
        "files": [str(args.output / name) for name in (
            "01_joint_position.png", "02_joint_velocity_acceleration.png",
            "03_inverse_dynamics_torque.png")],
    }, indent=2))


if __name__ == "__main__":
    main()
