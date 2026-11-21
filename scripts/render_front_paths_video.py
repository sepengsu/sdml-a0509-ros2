"""Render the four forward-region target trajectories in a MuJoCo comparison video."""

from __future__ import annotations

import json
import argparse
from pathlib import Path
import subprocess
import sys

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from render_lr_paths_video import _font, _trail


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src/sim_real_gap_getting/examples/front"
SCENE = ROOT / "src/doosan-robot2/dsr_description2/mujoco_models/a0509/scene_forward_wall.xml"
OUTPUT = ROOT / "output/a0509_front_f1_f4_mujoco_preview.mp4"
sys.path.insert(0, str(ROOT / "src/sim_real_gap_getting"))
from sim_real_gap_getting.trajectory import load_trajectory  # noqa: E402


FPS = 15
FRAMES = 397
PLANNED_SECONDS = 66.0  # 16 s setup + longest 50 s cycle
WIDTH, HEIGHT = 1280, 720
MARGIN, GAP, TOP, BOTTOM, LABEL = 18, 8, 57, 37, 34
CELL_W = (WIDTH - 2 * MARGIN - GAP) // 2
CELL_H = (HEIGHT - TOP - BOTTOM - GAP) // 2
RENDER_H = CELL_H - LABEL
PROFILES = (
    ("f1_slow_forward", "F1  Slow front/back", "#4cc9f0"),
    ("f2_medium_forward", "F2  Medium front/back", "#f8961e"),
    ("f3_front_postures", "F3  Front posture loop", "#90be6d"),
    ("f4_wrist_and_holds", "F4  Wrist + holds", "#b084f5"),
)


def _camera():
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (.26, 0, .45)
    camera.distance = 1.45
    camera.azimuth = 120
    camera.elevation = -25
    return camera


def _records(model, data, tcp_site):
    setup_path = SOURCE / "a0509_front_setup_10cm.json"
    setup = load_trajectory(setup_path)
    flange = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_6")
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, tcp_site)
    records = []
    for stem, title, color in PROFILES:
        path = SOURCE / f"a0509_front_{stem}.json"
        cycle = load_trajectory(path)
        if not np.allclose(setup.points[-1].q_rad, cycle.points[0].q_rad, atol=1e-8):
            raise ValueError(f"setup does not join {stem}")
        joints = np.empty((FRAMES, 6), dtype=float)
        tcp = np.empty((FRAMES, 3), dtype=float)
        for frame in range(FRAMES):
            planned_s = frame * PLANNED_SECONDS / (FRAMES - 1)
            if planned_s <= setup.duration_s:
                sample = setup.points[min(round(planned_s * 100), len(setup.points) - 1)]
            else:
                cycle_s = min(planned_s - setup.duration_s, cycle.duration_s)
                sample = cycle.points[min(round(cycle_s * 100), len(cycle.points) - 1)]
            joints[frame] = sample.q_rad
            data.qpos[:] = joints[frame]
            mujoco.mj_forward(model, data)
            tcp[frame] = data.site_xpos[site] if site >= 0 else data.xpos[flange]
        records.append({"title": title, "color": color, "source": str(path),
                        "duration_s": cycle.duration_s, "q": joints, "tcp": tcp})
    return records, setup_path


def render(output: Path = OUTPUT, *, scene: Path = SCENE,
           tool_label: str = "bare arm", tcp_site: str = "bare_flange") -> dict:
    model = mujoco.MjModel.from_xml_path(str(scene))
    model.vis.global_.offwidth = CELL_W
    model.vis.global_.offheight = RENDER_H
    wall = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "rear_wall")
    model.geom_rgba[wall, 3] = .10  # visual alpha only; collision geometry is unchanged
    data = mujoco.MjData(model)
    records, setup_path = _records(model, data, tcp_site)
    camera = _camera()
    output.parent.mkdir(parents=True, exist_ok=True)
    command = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
               "-pix_fmt", "rgb24", "-s", f"{WIDTH}x{HEIGHT}", "-r", str(FPS),
               "-i", "-", "-an", "-c:v", "libx264", "-crf", "19",
               "-preset", "medium", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
               str(output)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        with mujoco.Renderer(model, height=RENDER_H, width=CELL_W,
                             max_geom=model.ngeom + FRAMES + 8) as renderer:
            for frame in range(FRAMES):
                planned_s = frame * PLANNED_SECONDS / (FRAMES - 1)
                phase = ("SETUP +X 100 mm" if planned_s < 10 else
                         "SETUP +Z 60 mm" if planned_s < 16 else
                         f"CYCLES  t={planned_s - 16:.1f} s")
                canvas = Image.new("RGB", (WIDTH, HEIGHT), "#101a28")
                draw = ImageDraw.Draw(canvas)
                draw.text((MARGIN, 13), "A0509   |   Four forward-region trajectories",
                          font=_font(25, True), fill="#f4f7fb")
                draw.text((WIDTH - MARGIN, 19), f"{phase}  ·  SIMULATION ONLY",
                          font=_font(15), fill="#ffc56e", anchor="ra")
                for index, record in enumerate(records):
                    row, col = divmod(index, 2)
                    x = MARGIN + col * (CELL_W + GAP)
                    y = TOP + row * (CELL_H + GAP)
                    data.qpos[:] = record["q"][frame]
                    mujoco.mj_forward(model, data)
                    renderer.update_scene(data, camera=camera)
                    _trail(renderer.scene, record["tcp"][:frame + 1], record["color"])
                    canvas.paste(Image.fromarray(renderer.render()), (x, y + LABEL))
                    draw.rectangle((x, y, x + CELL_W, y + LABEL), fill="#27384a")
                    draw.text((x + 10, y + 8), record["title"],
                              font=_font(16, True), fill=record["color"])
                    draw.text((x + CELL_W - 10, y + 9), f"{record['duration_s']:g} s cycle",
                              font=_font(13), fill="#e1e8f0", anchor="ra")
                    tcp = record["tcp"][frame] * 1000
                    draw.text((x + 9, y + LABEL + RENDER_H - 23),
                              f"TCP  X {tcp[0]:.0f}   Y {tcp[1]:+.0f} mm",
                              font=_font(13, True), fill="#f4f7fb",
                              stroke_width=2, stroke_fill="#1c2530")
                draw.text((MARGIN, HEIGHT - 25),
                          f"Planned joint targets in MuJoCo  ·  {tool_label}  ·  rear wall X = -350 mm",
                          font=_font(14), fill="#d8e0ea")
                draw.text((WIDTH - MARGIN, HEIGHT - 25),
                          f"{planned_s:.1f} / {PLANNED_SECONDS:.0f} s planned time  ·  2.5× preview",
                          font=_font(14), fill="#f4f7fb", anchor="ra")
                if frame == 150:
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
                "source": "planned 100 Hz joint targets rendered in MuJoCo; not recorded motion",
                "setup": str(setup_path), "scene": str(scene), "tool_model": tool_label,
                "fps": FPS, "frames": FRAMES, "resolution": [WIDTH, HEIGHT],
                "planned_seconds": PLANNED_SECONDS, "preview_seconds": (FRAMES - 1) / FPS,
                "profiles": [{"name": stem, "json": record["source"],
                              "cycle_duration_s": record["duration_s"]}
                             for (stem, _, _), record in zip(PROFILES, records)]}
    output.with_suffix(".json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--tool", choices=("bare", "ag145", "mdpg_1s1"), default="bare")
    args = parser.parse_args()
    if args.tool == "bare":
        output, scene, label, tcp_site = OUTPUT, SCENE, "bare arm", "bare_flange"
    else:
        output = ROOT / f"output/a0509_front_f1_f4_{args.tool}_preview.mp4"
        scene = ROOT / f"results/front_gripper_preflight/{'ag145' if args.tool == 'ag145' else 'mdpg'}_scene.xml"
        label = "AG145 model" if args.tool == "ag145" else "mDPG assumed mount model"
        tcp_site = "ag145_tcp" if args.tool == "ag145" else "mdpg_tcp"
    print(json.dumps(render(output, scene=scene, tool_label=label, tcp_site=tcp_site), indent=2))
