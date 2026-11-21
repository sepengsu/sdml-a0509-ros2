"""Animate all 16 recorded bare-tool TCP paths in a synchronized 4x4 grid.

All panels start and finish together by normalized path time. Each panel displays
its original simulation duration. Coordinates are TCP displacement from the
start pose in millimeters, taken from the recorded MuJoCo dynamics.csv files.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np

from .render_t1_t16_tcp_3d import DEFAULT_CAMPAIGN, ROOT, _load_trial


DEFAULT_OUTPUT = ROOT / "results/bare_t1_t16_simultaneous.mp4"


def _read_paths(campaign: Path, condition: str, trial: str, samples: int) -> list[dict]:
    records = []
    progress = np.linspace(0.0, 1.0, samples)
    for number in range(1, 17):
        trajectory = f"T{number}"
        folder = campaign / "simulation" / trajectory / condition / trial
        t, xyz, _ = _load_trial(folder, condition, trajectory, trial)
        displacement = xyz - xyz[0]
        phase = t / t[-1]
        sampled = np.column_stack(
            [np.interp(progress, phase, displacement[:, axis]) for axis in range(3)]
        )
        records.append(
            {"id": trajectory, "duration_s": float(t[-1]), "points": sampled,
             "source": str(folder / "dynamics.csv"), "source_rows": len(t)}
        )
    return records


def render(campaign: Path, output: Path, condition: str = "bare_p0",
           trial: str = "trial_001", fps: int = 15, seconds: int = 12) -> dict:
    frames = fps * seconds + 1
    records = _read_paths(campaign.resolve(), condition, trial, frames)
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 9,
        "text.color": "#e8eefb", "axes.labelcolor": "#aebbd0",
    })
    fig = plt.figure(figsize=(16, 9), dpi=120, facecolor="#0b1220")
    axes = []
    lines = []
    heads = []
    colors = plt.cm.turbo(np.linspace(0.04, 0.94, 16))
    for index, record in enumerate(records):
        ax = fig.add_subplot(4, 4, index + 1, projection="3d", facecolor="#111c2e")
        axes.append(ax)
        ax.set_proj_type("ortho")
        ax.view_init(elev=23, azim=-58)
        ax.set_box_aspect((1, 1, 1))
        points = record["points"]
        lo, hi = points.min(axis=0), points.max(axis=0)
        center = (lo + hi) / 2
        half = max(8.0, float((hi - lo).max()) * 0.61)
        for setter, middle in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), center):
            setter(middle - half, middle + half)
        ax.set_xticks([])
        ax.set_yticks([])
        ax.set_zticks([])
        ax.grid(False)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.pane.set_facecolor((0.09, 0.15, 0.24, 1.0))
            axis.pane.set_edgecolor((0.35, 0.43, 0.56, 1.0))
            axis.line.set_color("#53617a")
        ax.set_title(f"{record['id']}  ·  {record['duration_s']:g} s",
                     color="#e8eefb", fontsize=12, fontweight="bold", pad=-2)
        line, = ax.plot([], [], [], color=colors[index], lw=2.8,
                        solid_capstyle="round")
        head, = ax.plot([], [], [], marker="o", color="white", markersize=5,
                        markeredgecolor=colors[index], markeredgewidth=1.5,
                        linestyle="None")
        lines.append(line)
        heads.append(head)

    fig.subplots_adjust(left=0.015, right=0.985, bottom=0.085, top=0.895,
                        wspace=0.025, hspace=0.09)
    fig.text(0.028, 0.945, "BARE  |  16 TCP TRAJECTORIES", fontsize=21,
             fontweight="bold", color="#f8fbff", va="center")
    fig.text(0.972, 0.945, "MuJoCo  ·  Doosan A0509  ·  no gripper / no payload",
             fontsize=10, color="#aebbd0", ha="right", va="center")
    progress_label = fig.text(0.028, 0.033, "PROGRESS  0%", fontsize=11,
                              fontweight="bold", color="#f8fbff", va="center")
    fig.text(0.972, 0.033,
             "Synchronized by normalized time  ·  Each panel has its own XYZ scale",
             fontsize=9, color="#aebbd0", ha="right", va="center")
    progress_track = fig.add_axes((0.028, 0.065, 0.944, 0.006))
    progress_track.set_xlim(0, 1)
    progress_track.set_ylim(0, 1)
    progress_track.set_axis_off()
    progress_track.barh(0.5, 1.0, height=1.0, color="#26354c")
    progress_bar = progress_track.barh(0.5, 0.0, height=1.0, color="#49d6be")[0]

    def update(frame: int):
        for record, line, head in zip(records, lines, heads):
            xyz = record["points"][:frame + 1]
            line.set_data_3d(xyz[:, 0], xyz[:, 1], xyz[:, 2])
            head.set_data_3d(xyz[-1:, 0], xyz[-1:, 1], xyz[-1:, 2])
        ratio = frame / (frames - 1)
        progress_label.set_text(f"PROGRESS  {ratio:.0%}")
        progress_bar.set_width(ratio)
        return [*lines, *heads, progress_label, progress_bar]

    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    video = animation.FuncAnimation(fig, update, frames=frames, interval=1000 / fps,
                                    blit=False)
    writer = animation.FFMpegWriter(
        fps=fps, codec="libx264", bitrate=4500,
        extra_args=["-pix_fmt", "yuv420p", "-movflags", "+faststart"],
    )
    video.save(str(output), writer=writer, dpi=120)
    update(frames - 1)
    still = output.with_name(output.stem + "_final.png")
    fig.savefig(still, dpi=120, facecolor=fig.get_facecolor())
    plt.close(fig)
    manifest = {
        "video": str(output), "final_frame": str(still),
        "condition": condition, "trial": trial,
        "layout": "4x4", "synchronization": "normalized_elapsed_time",
        "fps": fps, "frames": frames, "duration_s": frames / fps,
        "trajectories": [
            {key: record[key] for key in ("id", "duration_s", "source", "source_rows")}
            for record in records
        ],
    }
    output.with_suffix(".json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", type=Path, default=DEFAULT_CAMPAIGN)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--condition", default="bare_p0")
    parser.add_argument("--trial", default="trial_001")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--seconds", type=int, default=12)
    args = parser.parse_args()
    if args.fps < 1 or args.seconds < 1:
        raise ValueError("fps and seconds must be positive")
    result = render(args.campaign, args.output, args.condition, args.trial,
                    args.fps, args.seconds)
    print(json.dumps({key: result[key] for key in ("video", "final_frame", "condition",
                                                  "fps", "frames", "duration_s")}, indent=2))


if __name__ == "__main__":
    main()
