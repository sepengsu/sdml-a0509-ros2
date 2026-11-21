"""Replay all 16 bare A0509 trials as MuJoCo robot views in one video."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from model.campaign_rbd import load_scene
from .render_t1_t16_tcp_3d import DEFAULT_CAMPAIGN, ROOT


DEFAULT_OUTPUT = ROOT / "results/bare_t1_t16_mujoco_replay.mp4"
CONDITION = "bare_p0"
TRIAL = "trial_001"
WIDTH, HEIGHT = 1920, 1080
MARGIN, GAP, TOP, BOTTOM, LABEL = 18, 6, 62, 68, 26
CELL_W = (WIDTH - 2 * MARGIN - 3 * GAP) // 4
CELL_H = (HEIGHT - TOP - BOTTOM - 3 * GAP) // 4
RENDER_H = CELL_H - LABEL


def _font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    name = "arialbd.ttf" if bold else "arial.ttf"
    path = Path("C:/Windows/Fonts") / name
    if path.is_file():
        return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def _load_records(campaign: Path, frames: int) -> list[dict]:
    progress = np.linspace(0.0, 1.0, frames)
    records = []
    for number in range(1, 17):
        name = f"T{number}"
        folder = campaign / "simulation" / name / CONDITION / TRIAL
        metadata = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
        if (metadata.get("condition_id") != CONDITION or metadata.get("trajectory_id") != name
                or metadata.get("trial_id") != TRIAL or metadata.get("gripper_type") != "none"
                or float(metadata.get("payload_mass_kg", -1)) != 0.0):
            raise ValueError(f"incorrect bare trial metadata: {folder}")
        times, joints, tcp = [], [], []
        with (folder / "dynamics.csv").open(encoding="utf-8", newline="") as stream:
            for row in csv.DictReader(stream):
                if (row["trajectory_id"] != name or row["trial_id"] != TRIAL
                        or row["gripper_type"] != "none" or row["mode"] != "simulation"):
                    raise ValueError(f"incorrect recorded state: {folder}")
                times.append(float(row["ros_time_s"]))
                joints.append([float(row[f"q{i}"]) for i in range(1, 7)])
                tcp.append(json.loads(row["tcp_position"]))
        times = np.asarray(times, dtype=np.float64)
        joints = np.asarray(joints, dtype=np.float64)
        tcp = np.asarray(tcp, dtype=np.float64) / 1000.0
        if (joints.shape != (len(times), 6) or tcp.shape != (len(times), 3)
                or len(times) != metadata["quality"]["rows_written"]
                or not np.isfinite(joints).all() or not np.isfinite(tcp).all()
                or not np.all(np.diff(times) > 0)):
            raise ValueError(f"invalid recorded state: {folder}")
        phase = (times - times[0]) / (times[-1] - times[0])
        target = np.column_stack([
            np.interp(progress, phase, joints[:, j]) for j in range(6)
        ])
        path = np.column_stack([
            np.interp(progress, phase, tcp[:, j]) for j in range(3)
        ])
        records.append({
            "name": name, "duration_s": float(times[-1] - times[0]),
            "q": target, "tcp": path, "source": str(folder / "dynamics.csv"),
            "source_rows": len(times),
        })
    return records


def _camera(distance: float = 1.35) -> mujoco.MjvCamera:
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.05, -0.02, 0.52)
    camera.distance = distance
    camera.azimuth = 125
    camera.elevation = -22
    return camera


def _add_tcp_trail(scene: mujoco.MjvScene, points: np.ndarray) -> None:
    color = np.array([1.0, 0.72, 0.20, 1.0], dtype=np.float32)
    identity = np.eye(3, dtype=np.float64).ravel()
    zero = np.zeros(3, dtype=np.float64)
    # A few consecutive samples can coincide during dwell segments.
    for start, end in zip(points[:-1], points[1:]):
        if np.linalg.norm(end - start) < 1e-5:
            continue
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE,
                            zero, zero, identity, color)
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, 3.0, start, end)
        scene.ngeom += 1
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([0.016, 0.016, 0.016]), points[-1],
                        identity, np.array([1.0, 0.90, 0.30, 1.0], dtype=np.float32))
    scene.ngeom += 1


def _compose_frame(renderer: mujoco.Renderer, model: mujoco.MjModel,
                   data: mujoco.MjData, camera: mujoco.MjvCamera,
                   records: list[dict], frame: int, total: int,
                   *, title: str = "BARE A0509  |  MuJoCo robot replay",
                   subtitle: str = "16 recorded trajectories  ·  no gripper  ·  0 kg payload",
                   footer: str = "Recorded MuJoCo joint states  ·  TCP trail shown in orange") -> Image.Image:
    canvas = Image.new("RGB", (WIDTH, HEIGHT), "#111923")
    draw = ImageDraw.Draw(canvas)
    title_font, label_font = _font(28, True), _font(17, True)
    small_font = _font(15)
    draw.text((MARGIN, 16), title, font=title_font,
              fill="#f4f7fb")
    draw.text((WIDTH - MARGIN, 27), subtitle,
              font=small_font, fill="#d2dbe7", anchor="ra")
    for index, record in enumerate(records):
        row, col = divmod(index, 4)
        x = MARGIN + col * (CELL_W + GAP)
        y = TOP + row * (CELL_H + GAP)
        data.qpos[:] = record["q"][frame]
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=camera)
        _add_tcp_trail(renderer.scene, record["tcp"][:frame + 1])
        rgb = renderer.render()
        canvas.paste(Image.fromarray(np.asarray(rgb)), (x, y + LABEL))
        draw.rectangle((x, y, x + CELL_W, y + LABEL), fill="#263442")
        draw.text((x + 10, y + 3), record["name"], font=label_font, fill="white")
        draw.text((x + CELL_W - 10, y + 5), f"{record['duration_s']:g} s",
                  font=small_font, fill="#d2dbe7", anchor="ra")
    ratio = frame / (total - 1)
    bar_left, bar_right, bar_y = MARGIN, WIDTH - MARGIN, HEIGHT - 29
    draw.rectangle((bar_left, bar_y, bar_right, bar_y + 5), fill="#36465a")
    draw.rectangle((bar_left, bar_y, bar_left + int((bar_right - bar_left) * ratio),
                    bar_y + 5), fill="#ffc158")
    draw.text((MARGIN, HEIGHT - 48), f"Synchronized progress  {ratio:.0%}",
              font=small_font, fill="#f4f7fb")
    draw.text((WIDTH - MARGIN, HEIGHT - 48),
              footer,
              font=small_font, fill="#d2dbe7", anchor="ra")
    return canvas


def render(campaign: Path, output: Path, fps: int = 15, seconds: int = 12,
           camera_distance: float = 1.35) -> dict:
    campaign, output = campaign.resolve(), output.resolve()
    frames = fps * seconds + 1
    records = _load_records(campaign, frames)
    scene_path = campaign / "models/conditions/bare_p0.xml"
    model = load_scene(scene_path)
    if model.nq != 6 or model.body("link_6").id < 0:
        raise ValueError("expected six-axis bare A0509 MuJoCo scene")
    model.vis.global_.offwidth = CELL_W
    model.vis.global_.offheight = RENDER_H
    data = mujoco.MjData(model)
    camera = _camera(camera_distance)
    output.parent.mkdir(parents=True, exist_ok=True)
    command = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
               "-pixel_format", "rgb24", "-video_size", f"{WIDTH}x{HEIGHT}",
               "-framerate", str(fps), "-i", "-", "-an", "-c:v", "libx264",
               "-crf", "18", "-preset", "medium", "-pix_fmt", "yuv420p",
               "-movflags", "+faststart", str(output)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        with mujoco.Renderer(model, height=RENDER_H, width=CELL_W,
                             max_geom=model.ngeom + frames + 8) as renderer:
            for frame in range(frames):
                image = _compose_frame(renderer, model, data, camera, records, frame, frames)
                if frame == frames - 1:
                    image.save(output.with_name(output.stem + "_final.png"))
                assert process.stdin is not None
                process.stdin.write(image.tobytes())
                if frame % max(1, fps * 2) == 0:
                    print(f"Rendered MuJoCo frame {frame + 1}/{frames}", flush=True)
    finally:
        if process.stdin is not None:
            process.stdin.close()
    error = process.stderr.read().decode(errors="replace") if process.stderr else ""
    if process.wait() != 0:
        raise RuntimeError(f"ffmpeg encoding failed: {error}")
    manifest = {
        "video": str(output), "final_frame": str(output.with_name(output.stem + "_final.png")),
        "scene": str(scene_path), "condition": CONDITION, "trial": TRIAL,
        "source": "recorded MuJoCo qpos and TCP coordinates, replayed in MuJoCo Renderer",
        "synchronization": "normalized elapsed time", "fps": fps, "frames": frames,
        "resolution": [WIDTH, HEIGHT], "camera_distance_m": camera_distance,
        "trajectories": [{key: record[key] for key in
                          ("name", "duration_s", "source", "source_rows")}
                         for record in records],
    }
    output.with_suffix(".json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", type=Path, default=DEFAULT_CAMPAIGN)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--seconds", type=int, default=12)
    parser.add_argument("--camera-distance", type=float, default=1.35)
    args = parser.parse_args()
    if args.fps < 1 or args.seconds < 1 or args.camera_distance <= 0:
        raise ValueError("fps, seconds and camera distance must be positive")
    result = render(args.campaign, args.output, args.fps, args.seconds,
                    args.camera_distance)
    print(json.dumps({key: result[key] for key in
                      ("video", "final_frame", "scene", "fps", "frames")}, indent=2))


if __name__ == "__main__":
    main()
