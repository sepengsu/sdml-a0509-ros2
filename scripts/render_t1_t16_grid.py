"""Render the 16 recorded TCP paths as one title-free 4x4 figure."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import colors
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np

from .render_t1_t16_tcp_3d import DEFAULT_CAMPAIGN, ROOT, _load_trial


DEFAULT_OUTPUT = ROOT / "results/trajectory_3d_t1_t16_ag145_p0_grid.png"


def render(campaign: Path, output: Path, condition: str = "ag145_p0",
           trial: str = "trial_001") -> Path:
    campaign, output = campaign.resolve(), output.resolve()
    fig = plt.figure(figsize=(20, 20), dpi=200, facecolor="#ffffff")
    for number in range(1, 17):
        trajectory = f"T{number}"
        folder = campaign / "simulation" / trajectory / condition / trial
        t, xyz, _ = _load_trial(folder, condition, trajectory, trial)
        delta = xyz - xyz[0]
        stride = max(1, math.ceil((len(t) - 1) / 1200))
        indices = np.unique(np.r_[np.arange(0, len(t), stride), len(t) - 1])
        sampled = delta[indices]
        sampled_t = t[indices]

        ax = fig.add_subplot(4, 4, number, projection="3d", facecolor="#ffffff")
        ax.set_proj_type("ortho")
        ax.view_init(elev=26, azim=-55)
        segments = np.stack((sampled[:-1], sampled[1:]), axis=1)
        line = Line3DCollection(segments, cmap="viridis",
                                norm=colors.Normalize(vmin=0, vmax=float(t[-1])),
                                linewidth=2.3)
        line.set_array((sampled_t[:-1] + sampled_t[1:]) / 2)
        ax.add_collection3d(line)
        ax.scatter(*delta[0], color="#06885a", s=42, depthshade=False,
                   edgecolor="white", linewidth=0.7, zorder=10)
        ax.scatter(*delta[-1], color="#d34843", marker="X", s=45,
                   depthshade=False, edgecolor="white", linewidth=0.7, zorder=11)

        span = np.ptp(delta, axis=0)
        center = (delta.min(axis=0) + delta.max(axis=0)) / 2
        half = max(12.0, float(span.max()) * 0.58)
        for setter, middle in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), center):
            setter(middle - half, middle + half)
        ax.set_box_aspect((1, 1, 1))
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_zticks([])
        ax.grid(True, color="#dce3ed", alpha=0.7)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.pane.set_facecolor((0.97, 0.98, 1.0, 0.4))
            axis.pane.set_edgecolor("#dce3ed")
            axis.line.set_color("#a6b3c5")
        ax.text2D(0.04, 0.94, trajectory, transform=ax.transAxes,
                  fontsize=17, fontweight="bold", color="#1b2a40",
                  bbox={"boxstyle": "round,pad=0.22", "facecolor": "white",
                        "edgecolor": "#d8e0eb", "alpha": 0.92})

    fig.subplots_adjust(left=0.015, right=0.985, bottom=0.015, top=0.985,
                        wspace=0.015, hspace=0.015)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=200, facecolor="white")
    plt.close(fig)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", type=Path, default=DEFAULT_CAMPAIGN)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--condition", default="ag145_p0")
    parser.add_argument("--trial", default="trial_001")
    args = parser.parse_args()
    print(render(args.campaign, args.output, args.condition, args.trial))


if __name__ == "__main__":
    main()
