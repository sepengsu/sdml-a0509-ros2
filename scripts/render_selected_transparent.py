"""Render selected recorded 3D TCP paths as transparent, title-free PNGs."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import colors
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np

from .render_t1_t16_tcp_3d import DEFAULT_CAMPAIGN, ROOT, _load_trial


DEFAULT_OUTPUT = ROOT / "results/trajectory_3d_transparent"
DEFAULT_TRAJECTORIES = (3, 5, 15)


def render_one(campaign: Path, output: Path, number: int,
               condition: str, trial: str) -> dict:
    trajectory = f"T{number}"
    source = campaign / "simulation" / trajectory / condition / trial
    t, xyz, _ = _load_trial(source, condition, trajectory, trial)
    displacement = xyz - xyz[0]
    stride = max(1, math.ceil((len(t) - 1) / 1800))
    indices = np.unique(np.r_[np.arange(0, len(t), stride), len(t) - 1])
    sampled = displacement[indices]
    sampled_t = t[indices]
    segments = np.stack((sampled[:-1], sampled[1:]), axis=1)

    fig = plt.figure(figsize=(10, 8), dpi=200, facecolor=(1, 1, 1, 0))
    ax = fig.add_axes((0.025, 0.06, 0.76, 0.90), projection="3d")
    ax.set_facecolor((1, 1, 1, 0))
    ax.patch.set_alpha(0)
    ax.set_proj_type("ortho")
    ax.view_init(elev=27, azim=-55)
    line = Line3DCollection(segments, cmap="viridis",
                            norm=colors.Normalize(vmin=0, vmax=float(t[-1])),
                            linewidth=2.8)
    line.set_array((sampled_t[:-1] + sampled_t[1:]) / 2)
    ax.add_collection3d(line)
    ax.scatter(*displacement[0], color="#0b8f5b", s=85, depthshade=False,
               edgecolor="white", linewidth=1.2, label="Start", zorder=10)
    ax.scatter(*displacement[-1], color="#d34a45", marker="X", s=95,
               depthshade=False, edgecolor="white", linewidth=1,
               label="End", zorder=11)
    span = np.ptp(displacement, axis=0)
    center = (displacement.min(axis=0) + displacement.max(axis=0)) / 2
    half = max(12.0, float(span.max()) * 0.57)
    for setter, middle in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), center):
        setter(middle - half, middle + half)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("Displacement X (mm)", labelpad=11)
    ax.set_ylabel("Displacement Y (mm)", labelpad=11)
    ax.set_zlabel("Displacement Z (mm)", labelpad=7)
    ax.grid(True, alpha=0.35)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_facecolor((1, 1, 1, 0))
        axis.pane.set_edgecolor("#b8c2d2")
        axis.line.set_color("#9daabd")
    fig.legend(handles=[
        Line2D([0], [0], marker="o", linestyle="None", color="#0b8f5b", markersize=9, label="Start"),
        Line2D([0], [0], marker="X", linestyle="None", color="#d34a45", markersize=9, label="End"),
    ], loc="upper left", bbox_to_anchor=(0.07, 0.96), frameon=False)
    color_axis = fig.add_axes((0.89, 0.17, 0.025, 0.68))
    color_axis.patch.set_alpha(0)
    bar = fig.colorbar(line, cax=color_axis)
    bar.set_label("Simulation time (s)")
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=200, transparent=True, facecolor="none", edgecolor="none")
    plt.close(fig)
    return {"trajectory": trajectory, "source": str(source / "dynamics.csv"),
            "output": str(output), "samples": len(t), "transparent": True,
            "title": False, "footer": False}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", type=Path, default=DEFAULT_CAMPAIGN)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--condition", default="ag145_p0")
    parser.add_argument("--trial", default="trial_001")
    parser.add_argument("--trajectory", type=int, action="append")
    args = parser.parse_args()
    numbers = args.trajectory or DEFAULT_TRAJECTORIES
    if any(number < 1 or number > 16 for number in numbers) or len(set(numbers)) != len(numbers):
        raise ValueError("trajectory numbers must be distinct values from 1 to 16")
    records = [render_one(args.campaign.resolve(), args.output.resolve() / f"T{number:02d}.png", number,
                          args.condition, args.trial)
               for number in numbers]
    print(json.dumps(records, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
