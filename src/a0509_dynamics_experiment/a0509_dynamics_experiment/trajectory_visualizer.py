"""Headless Matplotlib figures and a self-contained HTML preview."""

from __future__ import annotations

import base64
import math
from pathlib import Path
from typing import Sequence

from .trajectory_generator import TrajectoryPlan
from .urdf_kinematics import A0509Urdf, find_a0509_urdf


def _plot():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    return plt


def save_trajectory_plot(plan: TrajectoryPlan, folder: Path,
                         measured_xyz_mm: Sequence[Sequence[float]] | None = None,
                         measured_joint_rad: Sequence[Sequence[float]] | None = None,
                         measured_time_s: Sequence[float] | None = None,
                         robot_links_mm: Sequence[Sequence[float]] | None = None,
                         robot_snapshots_mm: Sequence[Sequence[Sequence[float]]] | None = None) -> None:
    plt = _plot()
    joint = plan.config.trajectory_id == "T4"
    urdf_path = find_a0509_urdf()
    urdf = A0509Urdf(urdf_path) if urdf_path else None
    fig = plt.figure(figsize=(12, 7))
    ax = fig.add_subplot(121 if joint else 111, projection="3d")
    if joint:
        time_ax = fig.add_subplot(122)
        for i in range(6):
            plotted, = time_ax.plot([p.time_s for p in plan.points],
                                    [p.q_rad[i] * 180 / math.pi for p in plan.points], label=f"J{i+1} planned")
            if measured_joint_rad and measured_time_s:
                time_ax.plot(measured_time_s, [q[i] * 180 / math.pi for q in measured_joint_rad],
                             linestyle="--", color=plotted.get_color(), alpha=0.7,
                             label="Measured" if i == 0 else None)
        time_ax.set_xlabel("Time (s)"); time_ax.set_ylabel("Joint angle (deg)")
        time_ax.legend(ncol=3, fontsize=8); time_ax.grid(True)
    if joint and urdf is None:
        q_values = [point.q_rad for point in plan.points]
        series = [[value[i] * 180 / math.pi for value in q_values] for i in range(3)]
        ax.plot(*series, color="tab:blue", linewidth=1.5)
        ax.scatter(*[series[i][0] for i in range(3)], color="green", s=45, label="Start")
        ax.scatter(*[series[i][-1] for i in range(3)], color="red", marker="x", s=55, label="End")
        ax.set_xlabel("J1 (deg)"); ax.set_ylabel("J2 (deg)"); ax.set_zlabel("J3 (deg)")
        ax.legend(fontsize=8)
    else:
        if joint:
            values = [urdf.links_mm(point.q_rad)[-1] for point in plan.points]
            robot_links_mm = urdf.links_mm(plan.points[0].q_rad)
            reference = values[0]
        else:
            values = [point.xyz_mm for point in plan.points]
            reference = plan.config.reference_tcp_mm
            if robot_links_mm is None and urdf is not None:
                candidate = urdf.links_mm((0.0,) * 6)
                if math.dist(candidate[-1], reference) < 100:
                    robot_links_mm = candidate
        assert all(value is not None for value in values)
        axes = [[value[i] for value in values] for i in range(3)]
        ax.plot(*axes, color="tab:blue", label="Planned", linewidth=1.7)
        ax.scatter(*values[0], color="green", s=60, label="Start", zorder=5)
        ax.scatter(*values[-1], color="red", marker="x", s=80, label="End", zorder=6)
        wp = [values[i] for i in plan.waypoint_indices]
        if wp:
            ax.scatter(*zip(*wp), color="orange", s=18, label="Waypoints")
        ax.scatter(*reference, marker="x", color="purple", s=75, label="TCP reference")
        ax.scatter(0, 0, 0, marker="+", color="black", s=75, label="Robot base")
        if measured_xyz_mm:
            ax.plot(*zip(*measured_xyz_mm), color="tab:red", label="Measured", alpha=0.8)
        if robot_links_mm:
            ax.plot(*zip(*robot_links_mm), color="0.35", marker="o", label="Robot links")
            if joint:
                for index in plan.waypoint_indices[1::max(1, len(plan.waypoint_indices) // 4)]:
                    links = urdf.links_mm(plan.points[index].q_rad)
                    ax.plot(*zip(*links), color="0.6", alpha=0.25)
        if robot_snapshots_mm:
            for links in robot_snapshots_mm:
                ax.plot(*zip(*links), color="0.5", alpha=0.28)
        movement = [i for i in range(1, len(values) - 1) if
                    any(abs(values[i+1][axis] - values[i-1][axis]) > 1e-6 for axis in range(3))]
        step = max(1, len(movement) // 8)
        for index in movement[::step]:
            p = values[index]
            delta = [values[index+1][i] - values[index-1][i] for i in range(3)]
            ax.quiver(*p, *delta, color="tab:blue", normalize=True, length=8.0, arrow_length_ratio=0.4)
        ax.set_xlabel("X (mm)"); ax.set_ylabel("Y (mm)"); ax.set_zlabel("Z (mm)")
        display_axes = [axis + ([p[i] for p in robot_links_mm] if robot_links_mm else [])
                        for i, axis in enumerate(axes)]
        spans = [max(series) - min(series) for series in display_axes]
        ax.set_box_aspect((1, 1, 1) if joint else [max(20.0, span) for span in spans])
        for setter, series, span in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), display_axes, spans):
            padding = max(10.0, span * 0.08)
            setter(min(series) - padding, max(series) + padding)
        ax.legend(loc="upper left", fontsize=8)
        range_text = "Range: " + ", ".join(f"{name} {min(axis):.1f}..{max(axis):.1f} mm"
                                           for name, axis in zip("XYZ", axes))
        if joint:
            ax.text2D(0.02, -0.07, range_text, transform=ax.transAxes, fontsize=8)
        else:
            ax.text2D(0.02, 0.03, range_text, transform=ax.transAxes)
    ax.set_title(f"{plan.config.trajectory_id} planned trajectory")
    if joint:
        fig.subplots_adjust(wspace=0.4, left=0.03, right=0.97)
    else:
        fig.tight_layout()
    png = folder / "trajectory_3d.png"
    fig.savefig(png, dpi=160)
    plt.close(fig)
    encoded = base64.b64encode(png.read_bytes()).decode("ascii")
    (folder / "trajectory_3d.html").write_text(
        f'<!doctype html><html><head><meta charset="utf-8"><title>{plan.config.trajectory_id}</title></head>'
        f'<body><h1>{plan.config.trajectory_id} trajectory</h1><img alt="3D trajectory" '
        f'style="max-width:100%" src="data:image/png;base64,{encoded}"></body></html>\n', encoding="utf-8")


def save_dynamics_plot(rows: Sequence[dict], folder: Path) -> None:
    plt = _plot()
    fig, axes = plt.subplots(2, 2, figsize=(12, 7))
    if rows:
        t0 = float(rows[0]["timestamp"])
        time = [float(row["timestamp"]) - t0 for row in rows]
        for i in range(1, 7):
            for ax, key, title in ((axes[0, 0], f"q{i}", "Position (rad)"),
                                   (axes[0, 1], f"dq{i}", "Velocity (rad/s)")):
                series = [row.get(key) for row in rows]
                ax.plot(time, [float(v) if v is not None else float("nan") for v in series], label=f"J{i}")
        for key, color in (("actual_motor_torque", "tab:blue"), ("actual_joint_torque", "tab:red"),
                           ("qfrc_inverse", "tab:green")):
            values = [row.get(key) for row in rows]
            if any(v is not None for v in values):
                axes[1, 0].plot(time, [v[0] if isinstance(v, (list, tuple)) else float("nan") for v in values],
                                label=key, color=color)
        axes[1, 1].plot(time, range(len(rows)), color="black")
    for ax, title in zip(axes.flat, ("Position (rad)", "Velocity (rad/s)", "Torque, joint 1 (Nm)", "Samples")):
        ax.set_title(title); ax.set_xlabel("Elapsed time (s)"); ax.grid(True)
        if ax.lines and title != "Samples": ax.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(folder / "dynamics_plot.png", dpi=150)
    plt.close(fig)
