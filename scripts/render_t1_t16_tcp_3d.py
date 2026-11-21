"""Render 16 observed 3D TCP trajectories from the Fourier T16 campaign.

The default source is AG145 without payload, trial_001. Each plot uses recorded
MuJoCo TCP positions from dynamics.csv, relative to that trajectory's start.
The ZIP contains exactly T01.png through T16.png.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import zipfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import colors
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CAMPAIGN = ROOT / "data/grid_16_fourier_t16_20261111"
DEFAULT_OUTPUT = ROOT / "results/trajectory_3d_t1_t16_ag145_p0"
TITLES = (
    "Cartesian line out and back", "Cartesian diagonal out and back",
    "Cartesian box edges", "Sequential joint sweeps",
    "J2 and J3 alternating sine", "J1 and J4 coupled sine",
    "Wrist excitation J5 and J6", "Odd joints multi-sine",
    "Even joints multi-sine", "Six-joint multi-sine",
    "J2/J3/J5 frequency sweep", "J1/J4/J6 frequency sweep",
    "Shifted start pose A", "Shifted start pose B",
    "Low-speed reversals", "Optimized six-joint Fourier",
)


def _load_trial(folder: Path, expected_condition: str, expected_trajectory: str,
                expected_trial: str) -> tuple[np.ndarray, np.ndarray, dict]:
    metadata_path = folder / "metadata.json"
    source = folder / "dynamics.csv"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if (metadata.get("mode") != "simulation" or metadata.get("condition_id") != expected_condition
            or metadata.get("trajectory_id") != expected_trajectory
            or metadata.get("trial_id") != expected_trial):
        raise ValueError(f"metadata identity mismatch: {folder}")
    timestamps, positions = [], []
    with source.open(encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            if (row["mode"] != "simulation" or row["trajectory_id"] != expected_trajectory
                    or row["trial_id"] != expected_trial):
                raise ValueError(f"CSV identity mismatch: {folder}")
            timestamps.append(float(row["ros_time_s"]))
            positions.append(json.loads(row["tcp_position"]))
    t = np.asarray(timestamps, dtype=np.float64)
    xyz = np.asarray(positions, dtype=np.float64)
    if (xyz.shape != (len(t), 3) or len(t) < 2 or not np.isfinite(xyz).all()
            or not np.isfinite(t).all() or not np.all(np.diff(t) > 0)):
        raise ValueError(f"invalid TCP path: {folder}")
    if len(t) != metadata["quality"]["rows_written"]:
        raise ValueError(f"row count mismatch: {folder}")
    return t - t[0], xyz, metadata


def _draw(t: np.ndarray, xyz_mm: np.ndarray, trajectory: str,
          title: str, condition: str, output: Path) -> dict:
    displacement = xyz_mm - xyz_mm[0]
    stride = max(1, math.ceil((len(t) - 1) / 1800))
    indices = np.unique(np.r_[np.arange(0, len(t), stride), len(t) - 1])
    sampled = displacement[indices]
    sampled_t = t[indices]
    segments = np.stack((sampled[:-1], sampled[1:]), axis=1)
    fig = plt.figure(figsize=(12, 9), dpi=150, facecolor="#f7f9fc")
    ax = fig.add_axes((0.08, 0.12, 0.80, 0.72), projection="3d", facecolor="#f7f9fc")
    ax.set_proj_type("ortho")
    ax.view_init(elev=27, azim=-55)
    norm = colors.Normalize(vmin=0.0, vmax=float(t[-1]))
    line = Line3DCollection(segments, cmap="viridis", norm=norm, linewidth=2.8)
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
    ax.set_zlabel("Displacement Z (mm)", labelpad=11)
    ax.grid(True, alpha=0.35)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.fill = False
        axis.line.set_color("#9daabd")
    ax.legend(loc="upper left", bbox_to_anchor=(0.0, 1.0), frameon=True)
    bar = fig.colorbar(line, ax=ax, fraction=0.028, pad=0.06, shrink=0.68)
    bar.set_label("Simulation time (s)")
    fig.text(0.06, 0.94, f"{trajectory}  |  {title}", fontsize=22,
             fontweight="bold", color="#14243b")
    fig.text(0.06, 0.895,
             f"Recorded MuJoCo TCP path  ·  {condition}  ·  trial_001  ·  {len(t):,} samples",
             fontsize=11.5, color="#52647a")
    fig.text(0.06, 0.055,
             "Observed displacement range  " + "  |  ".join(
                 f"{axis}: {value:.1f} mm" for axis, value in zip("XYZ", span)),
             fontsize=11, color="#52647a")
    fig.savefig(output, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    return {"samples": len(t), "duration_s": float(t[-1]),
            "range_xyz_mm": span.tolist(), "first_tcp_xyz_mm": xyz_mm[0].tolist(),
            "last_tcp_xyz_mm": xyz_mm[-1].tolist(), "plot_stride": stride}


def render(campaign: Path, output: Path, condition: str = "ag145_p0",
           trial: str = "trial_001") -> tuple[list[dict], Path]:
    campaign, output = campaign.resolve(), output.resolve()
    if output.exists() and any(output.iterdir()):
        raise FileExistsError(f"choose a new or empty output folder: {output}")
    output.mkdir(parents=True, exist_ok=True)
    records = []
    for number, title in enumerate(TITLES, start=1):
        trajectory = f"T{number}"
        folder = campaign / "simulation" / trajectory / condition / trial
        t, xyz, metadata = _load_trial(folder, condition, trajectory, trial)
        image_name = f"T{number:02d}.png"
        details = _draw(t, xyz, trajectory, title, condition, output / image_name)
        source = folder / "dynamics.csv"
        records.append({
            "trajectory": trajectory, "image": image_name, "title": title,
            "condition": condition, "trial": trial,
            "source": str(source),
            "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "dataset_split": metadata["dataset_split"], **details,
        })
    manifest = output / "manifest.json"
    manifest.write_text(json.dumps({"campaign": str(campaign), "records": records},
                                   ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    archive = output.with_suffix(".zip")
    if archive.exists():
        raise FileExistsError(f"ZIP already exists: {archive}")
    with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED,
                         compresslevel=6) as zip_stream:
        for record in records:
            zip_stream.write(output / record["image"], arcname=record["image"])
    return records, archive


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", type=Path, default=DEFAULT_CAMPAIGN)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--condition", default="ag145_p0")
    parser.add_argument("--trial", default="trial_001")
    args = parser.parse_args()
    records, archive = render(args.campaign, args.output, args.condition, args.trial)
    print(json.dumps({"images": len(records), "output": str(args.output.resolve()),
                      "zip": str(archive)}, indent=2))


if __name__ == "__main__":
    main()
