"""Render one recorded AG145 trial against the same A0509 with TCC1 fitted."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
import csv
from pathlib import Path
import shutil
import subprocess
from tempfile import TemporaryDirectory

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from a0509_dynamics_experiment.ag145_model import build_ag145_scene
from tcc1_tool_changer_description.model import write_scene


ROOT = Path(__file__).resolve().parents[1]
TRIAL = ROOT / "data/grid_16_trajectories/simulation/T5/ag145_p0/trial_001/dynamics.csv"
OUTPUT = ROOT / "output/tcc1_nominal/T5_ag145_tcc1_comparison.mp4"
WIDTH, HEIGHT = 1600, 900
PANEL_WIDTH, MAIN_HEIGHT, DETAIL_HEIGHT = 768, 460, 230


def recorded_motion(path: Path) -> tuple[str, np.ndarray, np.ndarray]:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if any(row["mode"] != "simulation" or row["gripper_type"] != "ag145"
           or float(row["payload_mass"]) != 0.0 for row in rows):
        raise ValueError("Expected a recorded AG145 simulation trial with 0 kg payload")
    trajectories = {row["trajectory_id"] for row in rows}
    if len(trajectories) != 1 or not trajectories <= {f"T{i}" for i in range(1, 17)}:
        raise ValueError("Expected one recorded T1–T16 trajectory")
    times = np.asarray([float(row["ros_time_s"]) for row in rows])
    q = np.asarray([[float(row[f"q{i}"]) for i in range(1, 7)] for row in rows])
    if len(times) < 2 or not np.isfinite(times).all() or not np.isfinite(q).all() or min(np.diff(times)) <= 0:
        raise ValueError("Expected finite and strictly increasing time samples")
    return trajectories.pop(), times - times[0], q


def camera(lookat, distance: float, azimuth: float, elevation: float) -> mujoco.MjvCamera:
    result = mujoco.MjvCamera()
    result.type = mujoco.mjtCamera.mjCAMERA_FREE
    result.lookat[:] = lookat
    result.distance = distance
    result.azimuth, result.elevation = azimuth, elevation
    return result


def font(size: int, bold: bool = False):
    names = ("arialbd.ttf", "DejaVuSans-Bold.ttf") if bold else ("arial.ttf", "DejaVuSans.ttf")
    for base in (Path("C:/Windows/Fonts"), Path("/usr/share/fonts/truetype/dejavu")):
        for name in names:
            path = base / name
            if path.is_file():
                return ImageFont.truetype(str(path), size)
    return ImageFont.load_default(size=size)


def render(trial: Path, output: Path, fps: int, poster: Path | None = None) -> None:
    trajectory, source_time, recorded_q = recorded_motion(trial)
    duration = source_time[-1]
    times = np.linspace(0.0, duration, round(duration * fps) + 1)
    joint_states = np.column_stack([np.interp(times, source_time, channel) for channel in recorded_q.T])
    output.parent.mkdir(parents=True, exist_ok=True)
    ffmpeg = shutil.which("ffmpeg")
    if not ffmpeg:
        raise RuntimeError("ffmpeg is required to encode the comparison MP4")
    command = [ffmpeg, "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
               "-s", f"{WIDTH}x{HEIGHT}", "-r", str(fps), "-i", "-", "-an", "-c:v", "libx264",
               "-preset", "fast", "-crf", "18", "-pix_fmt", "yuv420p", "-movflags", "+faststart"]
    title, heading, label, small = font(35, True), font(24, True), font(17, True), font(17)
    with TemporaryDirectory(prefix="a0509_tcc1_") as folder, ExitStack() as stack:
        scratch = Path(folder)
        base = build_ag145_scene(scratch / "baseline.xml")
        mounted = write_scene(base, scratch / "mounted.xml", "ag145", tool_changer_type="tcc1")
        models = [mujoco.MjModel.from_xml_path(str(path)) for path in (base, mounted)]
        states = [mujoco.MjData(model) for model in models]
        renders = []
        for model in models:
            if (model.nq, model.nv) != (6, 6):
                raise ValueError("Comparison requires two six-axis A0509 models")
            model.vis.global_.offwidth = PANEL_WIDTH
            model.vis.global_.offheight = MAIN_HEIGHT
            renders.append(tuple(stack.enter_context(mujoco.Renderer(model, height=h, width=PANEL_WIDTH))
                                 for h in (MAIN_HEIGHT, DETAIL_HEIGHT)))
        overview = camera((0.22, -0.14, 0.47), 1.70, 125, -18)
        detail = camera((0, 0, 0), 0.46, 128, -9)
        temp_video = output.with_name(output.stem + ".partial.mp4")
        thumbnail = None
        with subprocess.Popen([*command, str(temp_video)], stdin=subprocess.PIPE) as encoder:
            try:
                for index, (time, q) in enumerate(zip(times, joint_states)):
                    for model, state in zip(models, states):
                        state.qpos[:] = q
                        mujoco.mj_forward(model, state)
                    same_flange = states[0].body("link_6")
                    detail.lookat[:] = same_flange.xpos + same_flange.xmat.reshape(3, 3)[:, 2] * 0.13
                    delta_mm = np.linalg.norm(states[1].site("ag145_tcp").xpos -
                                              states[0].site("ag145_tcp").xpos) * 1000
                    canvas = Image.new("RGB", (WIDTH, HEIGHT), "#101923")
                    draw = ImageDraw.Draw(canvas)
                    draw.text((24, 18), "A0509  /  TCC1 installation", font=title, fill="#ffffff")
                    draw.text((1576, 41), f"{trajectory}  /  0 kg payload  /  synchronized joint replay",
                              anchor="ra", font=small, fill="#d8e3ec")
                    titles = ("AG145   ·   TCC1 OFF", "TCC1 + AG145   ·   TCC1 ON")
                    for col, (name, state, pair) in enumerate(zip(titles, states, renders)):
                        x = 24 + col * (PANEL_WIDTH + 16)
                        draw.rectangle((x, 77, x + PANEL_WIDTH - 1, 126), fill=("#233444", "#153f50")[col])
                        draw.text((x + 15, 87), name, font=heading, fill="white")
                        for renderer, view, y in zip(pair, (overview, detail), (127, 594)):
                            renderer.update_scene(state, camera=view)
                            canvas.paste(Image.fromarray(renderer.render()), (x, y))
                        draw.rectangle((x + 9, 603, x + 142, 629), fill="#101923")
                        draw.text((x + 16, 605), "WRIST DETAIL", font=label, fill="white")
                    draw.rectangle((1150, 787, 1576, 823), fill="#101923")
                    draw.rectangle((1164, 800, 1176, 812), fill="#2487b3")
                    draw.text((1183, 795), "Master", font=label, fill="white")
                    draw.rectangle((1252, 800, 1264, 812), fill="#f07d1f")
                    draw.text((1271, 795), "Tool", font=label, fill="white")
                    draw.text((1562, 795), f"TCP shift  {delta_mm:.1f} mm",
                              anchor="ra", font=label, fill="white")
                    draw.text((24, 838), f"{time:05.2f} / {duration:.2f} s", font=label, fill="white")
                    draw.text((1576, 838), "CAD mounting estimate  •  visual replay",
                              anchor="ra", font=small, fill="#d8e3ec")
                    draw.rectangle((24, 873, 1575, 879), fill="#344659")
                    draw.rectangle((24, 873, 24 + round(1551 * time / duration), 879), fill="#f07d1f")
                    if poster is not None and index == fps:
                        thumbnail = canvas.copy()
                    encoder.stdin.write(canvas.tobytes())
                encoder.stdin.close()
                if encoder.wait():
                    raise RuntimeError("ffmpeg could not encode the comparison")
                temp_video.replace(output)
            finally:
                temp_video.unlink(missing_ok=True)
        if poster is not None:
            if thumbnail is None:
                thumbnail = canvas
            poster.parent.mkdir(parents=True, exist_ok=True)
            thumbnail.save(poster)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", type=Path, default=TRIAL)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--poster", type=Path)
    parser.add_argument("--fps", type=int, default=24)
    args = parser.parse_args()
    if args.fps < 1:
        parser.error("--fps must be positive")
    render(args.trial, args.output, args.fps, args.poster)
