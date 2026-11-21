"""Render a MuJoCo robot-motion preview of the uncollected wide16 v3 targets."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import mujoco
import numpy as np

from model.campaign_rbd import DEFAULT_CAMPAIGN, load_scene
from .render_bare_16_mujoco_replay import (
    CELL_W, RENDER_H, WIDTH, HEIGHT, _compose_frame,
)

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/a0509_dynamics_experiment"))
from a0509_dynamics_experiment.wide16_trajectory import generate_wide


OUTPUT = ROOT / "results/wide16_v3_bare_mujoco_preview.mp4"
FPS, FRAMES = 15, 181


def main() -> None:
    model = load_scene(DEFAULT_CAMPAIGN / "models/conditions/bare_p0.xml")
    model.vis.global_.offwidth = CELL_W
    model.vis.global_.offheight = RENDER_H
    data = mujoco.MjData(model)
    flange = model.body("link_6").id
    records = []
    for number in range(1, 17):
        plan = generate_wide(f"T{number}")
        indices = np.rint(np.linspace(0, len(plan.points) - 1, FRAMES)).astype(int)
        q = np.asarray([plan.points[index].q_rad for index in indices])
        tcp = np.empty((FRAMES, 3), dtype=float)
        for frame, joint in enumerate(q):
            data.qpos[:] = joint
            mujoco.mj_forward(model, data)
            tcp[frame] = data.xpos[flange]
        records.append({"name": f"T{number}", "duration_s": plan.points[-1].time_s,
                        "q": q, "tcp": tcp})
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.0, 0.0, 0.53)
    camera.distance = 1.7
    camera.azimuth = 125
    camera.elevation = -22
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    command = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
               "-pixel_format", "rgb24", "-video_size", f"{WIDTH}x{HEIGHT}",
               "-framerate", str(FPS), "-i", "-", "-an", "-c:v", "libx264",
               "-crf", "18", "-preset", "medium", "-pix_fmt", "yuv420p",
               "-movflags", "+faststart", str(OUTPUT)]
    process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        with mujoco.Renderer(model, height=RENDER_H, width=CELL_W,
                             max_geom=model.ngeom + FRAMES + 8) as renderer:
            for frame in range(FRAMES):
                image = _compose_frame(
                    renderer, model, data, camera, records, frame, FRAMES,
                    title="BARE A0509  |  Wide16 v3 planned motion",
                    subtitle="16 planned trajectories  ·  no gripper  ·  0 kg payload",
                    footer="MuJoCo plan preview  ·  Not a collected trial  ·  TCP trail in orange",
                )
                if frame == FRAMES - 1:
                    image.save(OUTPUT.with_name(OUTPUT.stem + "_final.png"))
                assert process.stdin is not None
                process.stdin.write(image.tobytes())
    finally:
        if process.stdin is not None:
            process.stdin.close()
    error = process.stderr.read().decode(errors="replace") if process.stderr else ""
    if process.wait() != 0:
        raise RuntimeError(f"ffmpeg failed: {error}")
    OUTPUT.with_suffix(".json").write_text(json.dumps({
        "video": str(OUTPUT), "profile": "wide16_v3", "condition": "bare_p0",
        "source": "planned joint targets rendered in MuJoCo; not recorded dynamics",
        "fps": FPS, "frames": FRAMES, "resolution": [WIDTH, HEIGHT],
    }, indent=2) + "\n", encoding="utf-8")
    print(OUTPUT)


if __name__ == "__main__":
    main()
