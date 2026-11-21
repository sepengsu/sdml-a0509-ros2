"""Render the generated P1-P5 target paths in one MuJoCo comparison video.

This is a visualization of planned joint targets, not recorded robot motion.
"""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src/sim_real_gap_getting/examples/lr"
SCENE = ROOT / "src/doosan-robot2/dsr_description2/mujoco_models/a0509/scene_forward_wall.xml"
OUTPUT = ROOT / "output/a0509_lr_p1_p5_mujoco_preview.mp4"
sys.path.insert(0, str(ROOT / "src/sim_real_gap_getting"))
from sim_real_gap_getting.trajectory import load_trajectory  # noqa: E402


WIDTH, HEIGHT = 1280, 720
FPS = 15
SETUP_FRAMES = FPS * 4 + 1
CYCLE_FRAMES = FPS * 15 + 1
FRAMES = SETUP_FRAMES + CYCLE_FRAMES - 1
SECONDS = (FRAMES - 1) / FPS
MARGIN, GAP, TOP, BOTTOM = 18, 8, 55, 36
CELL_W = (WIDTH - 2 * MARGIN - 2 * GAP) // 3
CELL_H = (HEIGHT - TOP - BOTTOM - GAP) // 2
LABEL_H = 34
RENDER_H = CELL_H - LABEL_H
PROFILES = (
    ("p1_slow", "P1  Slow", "#4cc9f0"),
    ("p2_medium", "P2  Medium", "#f8961e"),
    ("p3_variable", "P3  Variable", "#90be6d"),
    ("p4_reversal", "P4  Reversal + hold", "#f94144"),
    ("p5_multijoint", "P5  Multijoint", "#b084f5"),
)


def _font(size: int, bold: bool = False):
    path = Path("C:/Windows/Fonts") / ("arialbd.ttf" if bold else "arial.ttf")
    if path.is_file():
        return ImageFont.truetype(str(path), size)
    return ImageFont.load_default()


def _camera():
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (.17, 0, .45)
    camera.distance = 1.25
    camera.azimuth = 120
    camera.elevation = -25
    return camera


def _records(model, data):
    flange = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_6")
    setup_path = SOURCE / "a0509_lr_forward_10cm_setup.json"
    setup = load_trajectory(setup_path)
    setup_indices = np.rint(np.linspace(0, len(setup.points) - 1, SETUP_FRAMES)).astype(int)
    setup_q = np.asarray([setup.points[i].q_rad for i in setup_indices], dtype=float)
    records = []
    for stem, title, color in PROFILES:
        path = SOURCE / f"a0509_lr_{stem}.json"
        trajectory = load_trajectory(path)
        if not np.allclose(setup_q[-1], trajectory.points[0].q_rad, atol=1e-8):
            raise ValueError(f"setup does not join {stem}")
        indices = np.rint(np.linspace(0, len(trajectory.points) - 1, CYCLE_FRAMES)).astype(int)
        cycle_q = np.asarray([trajectory.points[i].q_rad for i in indices], dtype=float)
        joints = np.concatenate((setup_q, cycle_q[1:]), axis=0)
        tcp = np.empty((FRAMES, 3), dtype=float)
        for frame, q in enumerate(joints):
            data.qpos[:] = q
            mujoco.mj_forward(model, data)
            tcp[frame] = data.xpos[flange]
        records.append({"source": str(path), "setup_source": str(setup_path),
                        "title": title, "color": color,
                        "setup_duration_s": setup.duration_s,
                        "duration_s": trajectory.duration_s, "q": joints, "tcp": tcp})
    return records


def _trail(scene, points, color):
    rgb = tuple(int(color[i:i + 2], 16) / 255 for i in (1, 3, 5))
    rgba = np.asarray((*rgb, 1), dtype=np.float32)
    identity = np.eye(3).ravel()
    zero = np.zeros(3)
    for start, end in zip(points[:-1], points[1:]):
        if np.linalg.norm(end - start) < 1e-5:
            continue
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE,
                            zero, zero, identity, rgba)
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, 3.0, start, end)
        scene.ngeom += 1
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([.013, .013, .013]), points[-1], identity, rgba)
    scene.ngeom += 1


def _curve_panel(draw, records, frame, left, top):
    right = left + CELL_W
    bottom = top + CELL_H
    draw.rectangle((left, top, right, bottom), fill="#182637")
    draw.text((left + 12, top + 9), "TCP Y (colors) + forward X (gold)",
              font=_font(16, True), fill="white")
    x0, x1 = left + 43, right - 15
    y0, y1 = top + 48, bottom - 30
    for y_mm in (-100, 0, 100):
        y = y1 - (y_mm + 150) / 300 * (y1 - y0)
        draw.line((x0, y, x1, y), fill="#3c5265", width=1)
        draw.text((left + 3, y - 7), str(y_mm), font=_font(12), fill="#b9c8d7")
    for record in records:
        ys = record["tcp"][:, 1] * 1000
        coords = [(x0 + k / (FRAMES - 1) * (x1 - x0),
                   y1 - (float(value) + 150) / 300 * (y1 - y0))
                  for k, value in enumerate(ys)]
        draw.line(coords, fill=record["color"], width=2)
        draw.ellipse((coords[frame][0] - 4, coords[frame][1] - 4,
                      coords[frame][0] + 4, coords[frame][1] + 4), fill=record["color"])
    forward_x = (records[0]["tcp"][:, 0] - records[0]["tcp"][0, 0]) * 1000
    x_coords = [(x0 + k / (FRAMES - 1) * (x1 - x0),
                 y1 - (float(value) + 150) / 300 * (y1 - y0))
                for k, value in enumerate(forward_x)]
    draw.line(x_coords, fill="#ffd166", width=2)
    draw.ellipse((x_coords[frame][0] - 4, x_coords[frame][1] - 4,
                  x_coords[frame][0] + 4, x_coords[frame][1] + 4), fill="#ffd166")
    cursor_x = x0 + frame / (FRAMES - 1) * (x1 - x0)
    draw.line((cursor_x, y0, cursor_x, y1), fill="#f5f7fa", width=1)
    draw.text((x0, bottom - 22), "start / setup", font=_font(12), fill="#b9c8d7")
    draw.text((x1, bottom - 22), "end", anchor="ra", font=_font(12), fill="#b9c8d7")


def render(output: Path = OUTPUT) -> dict:
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.vis.global_.offwidth = CELL_W
    model.vis.global_.offheight = RENDER_H
    wall = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "rear_wall")
    model.geom_rgba[wall, 3] = .10  # visibility only; collision geometry is unchanged
    data = mujoco.MjData(model)
    records = _records(model, data)
    camera = _camera()
    output.parent.mkdir(parents=True, exist_ok=True)
    command = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
               "-pix_fmt", "rgb24", "-s", f"{WIDTH}x{HEIGHT}",
               "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264",
               "-crf", "19", "-preset", "medium", "-pix_fmt", "yuv420p",
               "-movflags", "+faststart", str(output)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        with mujoco.Renderer(model, height=RENDER_H, width=CELL_W,
                             max_geom=model.ngeom + FRAMES + 8) as renderer:
            for frame in range(FRAMES):
                canvas = Image.new("RGB", (WIDTH, HEIGHT), "#101a28")
                draw = ImageDraw.Draw(canvas)
                draw.text((MARGIN, 13), "A0509   |   Left / Right Sim-to-Real trajectory preview",
                          font=_font(25, True), fill="#f4f7fb")
                phase = ("SETUP  +X 100 mm" if frame < SETUP_FRAMES else
                         "LATERAL CYCLE")
                draw.text((WIDTH - MARGIN, 20), f"{phase}  ·  simulation only",
                          font=_font(15), fill="#ffc56e", anchor="ra")
                for index, record in enumerate(records):
                    row, col = divmod(index, 3)
                    x = MARGIN + col * (CELL_W + GAP)
                    y = TOP + row * (CELL_H + GAP)
                    data.qpos[:] = record["q"][frame]
                    mujoco.mj_forward(model, data)
                    renderer.update_scene(data, camera=camera)
                    _trail(renderer.scene, record["tcp"][:frame + 1], record["color"])
                    canvas.paste(Image.fromarray(renderer.render()), (x, y + LABEL_H))
                    draw.rectangle((x, y, x + CELL_W, y + LABEL_H), fill="#27384a")
                    draw.text((x + 10, y + 8), record["title"], font=_font(16, True),
                              fill=record["color"])
                    draw.text((x + CELL_W - 9, y + 9), f"{record['duration_s']:g} s cycle",
                              font=_font(13), fill="#e1e8f0", anchor="ra")
                _curve_panel(draw, records, frame,
                             MARGIN + 2 * (CELL_W + GAP), TOP + CELL_H + GAP)
                draw.text((MARGIN, HEIGHT - 26),
                          "Bare A0509 model  ·  rear wall X = -350 mm  ·  TCP trails shown in color",
                          font=_font(14), fill="#d8e0ea")
                progress = (min(1.0, frame / (SETUP_FRAMES - 1)) if frame < SETUP_FRAMES else
                            (frame - SETUP_FRAMES + 1) / (CYCLE_FRAMES - 1))
                draw.text((WIDTH - MARGIN, HEIGHT - 26),
                          f"{phase} {progress:.0%}",
                          font=_font(14, True), fill="#f4f7fb", anchor="ra")
                if frame == SETUP_FRAMES + CYCLE_FRAMES // 4:
                    canvas.save(output.with_suffix(".png"))
                assert process.stdin is not None
                process.stdin.write(canvas.tobytes())
                if frame % FPS == 0:
                    print(f"rendered {frame + 1}/{FRAMES}", flush=True)
    finally:
        if process.stdin is not None:
            process.stdin.close()
    error = process.stderr.read().decode(errors="replace") if process.stderr else ""
    if process.wait() != 0:
        raise RuntimeError(f"ffmpeg encoding failed: {error}")
    manifest = {"video": str(output), "thumbnail": str(output.with_suffix(".png")),
                "source": "100 Hz planned setup and cycle joint targets, replayed in MuJoCo renderer; not recorded motion",
                "scene": str(SCENE), "fps": FPS, "seconds": SECONDS,
                "setup_preview_seconds": (SETUP_FRAMES - 1) / FPS,
                "setup_motion_seconds": records[0]["setup_duration_s"],
                "resolution": [WIDTH, HEIGHT],
                "profiles": [{"name": stem, "json": record["source"],
                              "cycle_duration_s": record["duration_s"]}
                             for (stem, _, _), record in zip(PROFILES, records)]}
    output.with_suffix(".json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


if __name__ == "__main__":
    print(json.dumps(render(), indent=2))
