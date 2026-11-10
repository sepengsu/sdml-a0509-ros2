"""Render a close-view AG145 replay from recorded MuJoCo joint samples."""

from __future__ import annotations

import argparse
from bisect import bisect_left
import csv
import json
from pathlib import Path

from .trajectory_generator import JOINT_NAMES


def render_replay(trial_dir: str | Path, fps: int = 15,
                  width: int = 960, height: int = 720) -> Path:
    import imageio_ffmpeg
    import mujoco

    folder = Path(trial_dir)
    scene = folder / "ag145_scene.xml"
    dynamics = folder / "dynamics.csv"
    if not scene.is_file() or not dynamics.is_file():
        raise FileNotFoundError("AG145 scene and dynamics.csv are required in the trial folder")
    if fps < 1 or width < 64 or height < 64:
        raise ValueError("fps and video dimensions must be positive")

    samples = []
    with dynamics.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row["ros_time_s"] and row["qpos"]:
                q = json.loads(row["qpos"])
                if len(q) == 6:
                    samples.append((float(row["ros_time_s"]), q))
    if len(samples) < 2:
        raise ValueError("at least two timed MuJoCo qpos samples are needed")

    model = mujoco.MjModel.from_xml_path(str(scene))
    if model.nq != 6 or mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ag145_tcp") < 0:
        raise ValueError("trial scene is not the six-axis A0509 with AG145")
    qpos_addresses = []
    for name in JOINT_NAMES:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ValueError(f"trial scene is missing {name}")
        qpos_addresses.append(int(model.jnt_qposadr[joint_id]))
    model.vis.global_.offwidth = width
    model.vis.global_.offheight = height
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.1, -0.1, 0.52)
    camera.distance = 1.6
    camera.azimuth = 125
    camera.elevation = -25

    output = folder / "simulation_replay.mp4"
    times = [sample[0] for sample in samples]
    frame_count = round((times[-1] - times[0]) * fps) + 1
    writer = imageio_ffmpeg.write_frames(str(output), (width, height), fps=fps,
                                          codec="libx264", pix_fmt_in="rgb24",
                                          pix_fmt_out="yuv420p", quality=7)
    writer.send(None)
    try:
        with mujoco.Renderer(model, height=height, width=width) as renderer:
            for frame_index in range(frame_count):
                target_time = times[0] + frame_index / fps
                pos = bisect_left(times, target_time)
                index = min(range(max(0, pos - 1), min(len(times), pos + 1)),
                            key=lambda i: abs(times[i] - target_time))
                for address, value in zip(qpos_addresses, samples[index][1]):
                    data.qpos[address] = value
                mujoco.mj_forward(model, data)
                renderer.update_scene(data, camera=camera)
                writer.send(renderer.render().tobytes())
    finally:
        writer.close()

    metadata_path = folder / "metadata.json"
    if metadata_path.is_file():
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        metadata["replay_video"] = {
            "relative_path": output.name, "fps": fps, "frames": frame_count,
            "resolution": [width, height], "source": "recorded MuJoCo qpos in dynamics.csv",
            "camera": {"lookat_m": [0.1, -0.1, 0.52], "distance_m": 1.6,
                       "azimuth_deg": 125, "elevation_deg": -25},
        }
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return output


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Render close-view video from an AG145 simulation trial")
    parser.add_argument("trial_dir")
    parser.add_argument("--fps", type=int, default=15)
    args = parser.parse_args(argv)
    print(render_replay(args.trial_dir, args.fps))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
