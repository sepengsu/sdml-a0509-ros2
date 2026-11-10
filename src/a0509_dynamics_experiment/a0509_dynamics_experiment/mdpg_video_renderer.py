"""Render an mDPG visual attachment over a recorded T1 arm-only MuJoCo run.

This is a kinematic replay. It does not add unknown tool mass, inertia,
contact, vacuum, or pneumatic I/O to the recorded simulation.
"""

from __future__ import annotations

import argparse
from bisect import bisect_left
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

from .trajectory_generator import JOINT_NAMES


def _recorded_states(source: Path) -> tuple[list[float], list[list[float]]]:
    metadata = json.loads((source / "metadata.json").read_text(encoding="utf-8"))
    if (metadata.get("trajectory_id") != "T1" or metadata.get("mode") != "simulation"
            or metadata.get("gripper_type") != "none" or not metadata.get("execution_success")):
        raise ValueError("source must be a completed T1 arm-only MuJoCo trial")
    times, states = [], []
    with (source / "dynamics.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if not row["ros_time_s"] or not row["qpos"]:
                continue
            stamp = float(row["ros_time_s"])
            state = [float(value) for value in json.loads(row["qpos"])]
            if (len(state) != 6 or not math.isfinite(stamp)
                    or not all(math.isfinite(value) for value in state)
                    or (times and stamp <= times[-1])):
                raise ValueError("invalid T1 MuJoCo state sequence")
            times.append(stamp)
            states.append(state)
    if len(times) < 2:
        raise ValueError("T1 needs at least two timed qpos states")
    return times, states


def render_mdpg_replay(source_trial: str | Path, output_dir: str | Path,
                       mount_xyz, mount_rpy, fps: int = 15) -> Path:
    import imageio_ffmpeg
    import mujoco
    import numpy as np
    from PIL import Image, ImageDraw
    from mdpg_1s1_description.model import build_mdpg_scene

    if fps < 1 or fps > 60:
        raise ValueError("fps must be between 1 and 60")
    source, output_dir = Path(source_trial).resolve(), Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    times, states = _recorded_states(source)
    scene = build_mdpg_scene(output_dir / "mdpg_scene.xml", mount_xyz, mount_rpy)
    model = mujoco.MjModel.from_xml_path(str(scene))
    if model.nq != 6 or model.nv != 6:
        raise ValueError("visual scene must keep the A0509 six-axis model")
    addresses = []
    for name in JOINT_NAMES:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"visual scene is missing {name}")
        addresses.append(int(model.jnt_qposadr[joint_id]))
    tcp_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "mdpg_tcp")
    if tcp_id < 0:
        raise ValueError("visual scene has no mdpg_tcp site")
    data = mujoco.MjData(model)
    full = mujoco.MjvCamera()
    full.type = mujoco.mjtCamera.mjCAMERA_FREE
    full.lookat[:] = (0.08, -0.08, 0.55)
    full.distance, full.azimuth, full.elevation = 1.55, 125, -25
    close = mujoco.MjvCamera()
    close.type = mujoco.mjtCamera.mjCAMERA_FREE
    close.distance, close.azimuth, close.elevation = 0.34, 125, -25
    model.vis.global_.offwidth = 1280
    model.vis.global_.offheight = 720
    frame_count = round((times[-1] - times[0]) * fps) + 1
    output = output_dir / "mdpg_t1_visual_replay.mp4"
    writer = imageio_ffmpeg.write_frames(str(output), (1280, 720), fps=fps,
                                          codec="libx264", pix_fmt_in="rgb24",
                                          pix_fmt_out="yuv420p", quality=7)
    writer.send(None)
    tcp_start, tcp_end, tcp_min, tcp_max = None, None, None, None
    try:
        with (mujoco.Renderer(model, height=640, width=800) as full_renderer,
              mujoco.Renderer(model, height=640, width=480) as close_renderer):
            for frame_index in range(frame_count):
                target_time = times[0] + frame_index / fps
                pos = bisect_left(times, target_time)
                index = min(range(max(0, pos - 1), min(len(times), pos + 1)),
                            key=lambda i: abs(times[i] - target_time))
                for address, value in zip(addresses, states[index]):
                    data.qpos[address] = value
                mujoco.mj_forward(model, data)
                tcp = data.site_xpos[tcp_id].copy()
                if tcp_start is None:
                    tcp_start = tcp
                    tcp_min = tcp.copy()
                    tcp_max = tcp.copy()
                tcp_end = tcp
                tcp_min = np.minimum(tcp_min, tcp)
                tcp_max = np.maximum(tcp_max, tcp)
                full_renderer.update_scene(data, camera=full)
                full_frame = full_renderer.render()
                close.lookat[:] = tcp
                close_renderer.update_scene(data, camera=close)
                close_frame = close_renderer.render()
                frame = Image.new("RGB", (1280, 720), "#17202c")
                frame.paste(Image.fromarray(full_frame), (0, 60))
                frame.paste(Image.fromarray(close_frame), (800, 60))
                draw = ImageDraw.Draw(frame)
                draw.text((20, 14), "A0509  |  T1 linear reciprocation  |  mDPG_1S1 visual replay", fill="white")
                draw.text((20, 42), "Recorded MuJoCo arm motion; gripper geometry only; mount unverified", fill="#f9c66b")
                draw.text((20, 684), f"ROS simulation time  {target_time-times[0]:.2f} s", fill="white")
                draw.text((815, 684), "Close view: wrist + suction gripper", fill="white")
                if frame_index == 0:
                    frame.save(output_dir / "first_frame.png")
                if frame_index == frame_count // 2:
                    frame.save(output_dir / "thumbnail.png")
                if frame_index == frame_count - 1:
                    frame.save(output_dir / "last_frame.png")
                writer.send(np.asarray(frame).tobytes())
    finally:
        writer.close()
    provenance = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "trajectory_id": "T1", "gripper_type": "mdpg_1s1",
        "mode": "visual_replay", "source_trial": str(source),
        "source_gripper_type": "none", "source_dynamics_sha256": hashlib.sha256(
            (source / "dynamics.csv").read_bytes()).hexdigest(),
        "source_samples": len(times), "source_sim_time_duration_s": times[-1] - times[0],
        "mount_xyz_m": list(map(float, mount_xyz)),
        "mount_rpy_rad": list(map(float, mount_rpy)),
        "mount_status": "unverified visualization fixture; not a measured physical offset",
        "tool_mass_kg": None, "tool_inertia_kg_m2": None,
        "pneumatic_io": None, "tool_contact_dynamics": None,
        "motion_type": "kinematic replay of recorded arm qpos; no new dynamics rollout",
        "tcp_status": "CAD-bounds estimate, not calibrated",
        "tcp_start_m": tcp_start.tolist(), "tcp_end_m": tcp_end.tolist(),
        "tcp_net_displacement_m": (tcp_end - tcp_start).tolist(),
        "tcp_axis_range_m": (tcp_max - tcp_min).tolist(),
        "video": {"relative_path": output.name, "fps": fps, "frames": frame_count,
                  "resolution": [1280, 720], "sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
                  "camera": {"full_azimuth_deg": 125, "full_elevation_deg": -25,
                             "close_azimuth_deg": 125, "close_elevation_deg": -25}},
    }
    (output_dir / "metadata.json").write_text(json.dumps(provenance, indent=2) + "\n", encoding="utf-8")
    return output


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Render visual-only mDPG over a recorded T1 MuJoCo arm trial")
    parser.add_argument("source_trial")
    parser.add_argument("output_dir")
    parser.add_argument("--mount-xyz", type=float, nargs=3, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--mount-rpy", type=float, nargs=3, required=True, metavar=("R", "P", "Y"))
    parser.add_argument("--visual-fixture", action="store_true", required=True,
                        help="acknowledge that mount values are only a visualization fixture")
    parser.add_argument("--fps", type=int, default=15)
    args = parser.parse_args(argv)
    print(render_mdpg_replay(args.source_trial, args.output_dir,
                             args.mount_xyz, args.mount_rpy, args.fps))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
