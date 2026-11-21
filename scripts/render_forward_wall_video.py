"""Render MuJoCo dynamic states of the J1-fixed forward wall trajectory."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from build_forward_wall_trajectory import ROOT, wall_model


FPS = 30
RENDER_SIZE = (960, 720)
FRAME_SIZE = (960, 900)


def font(size: int, bold: bool = False):
    file = Path("C:/Windows/Fonts") / ("arialbd.ttf" if bold else "arial.ttf")
    return ImageFont.truetype(str(file), size) if file.exists() else ImageFont.load_default()


def add_trail(scene, points: np.ndarray) -> None:
    color = np.array([1.0, 0.76, 0.22, 1.0], dtype=np.float32)
    identity = np.eye(3).ravel()
    zero = np.zeros(3)
    for start, end in zip(points[:-1], points[1:]):
        if np.linalg.norm(end - start) < 1e-5:
            continue
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_LINE,
                            zero, zero, identity, color)
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_LINE, 5.0, start, end)
        scene.ngeom += 1
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([0.014, 0.014, 0.014]), points[-1], identity,
                        np.array([1.0, 0.88, 0.3, 1.0], dtype=np.float32))
    scene.ngeom += 1


def render(states_path: Path, output_path: Path, preview_only: bool = False,
           right_angle: bool = False) -> Path:
    states = np.load(states_path)
    times = states["time_s"]
    positions = states["q_rad"]
    tcp = states["tcp_m"]
    model = wall_model()
    model.vis.global_.offwidth, model.vis.global_.offheight = RENDER_SIZE
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.17, 0.0, 0.43) if right_angle else (0.0, 0.0, 0.53)
    camera.distance = 1.9 if right_angle else 1.75
    camera.azimuth = 215
    camera.elevation = -22
    output_path.parent.mkdir(parents=True, exist_ok=True)
    playback_speed = 2 if right_angle else 1
    frame_count = round(times[-1] * FPS / playback_speed) + 1

    def compose(renderer, frame: int) -> Image.Image:
        index = min(round(frame * 100 * playback_speed / FPS), len(times) - 1)
        data.qpos[:] = positions[index]
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, camera=camera)
        trail = tcp[:index + 1:3]
        if not np.array_equal(trail[-1], tcp[index]):
            trail = np.vstack((trail, tcp[index]))
        add_trail(renderer.scene, trail)
        image = Image.new("RGB", FRAME_SIZE, "#17202d")
        image.paste(Image.fromarray(renderer.render().copy()), (0, 100))
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, FRAME_SIZE[0], 100), fill="#17202d")
        draw.rectangle((0, 0, 12, 100), fill="#5ed7c5")
        title = ("A0509  |  RIGHT ANGLE x2: 20 cm FORWARD / 20 cm DOWN" if right_angle
                 else "A0509  |  FORWARD-ONLY WALL PRACTICE")
        draw.text((32, 17), title,
                  font=font(29, True), fill="white")
        subtitle = ("J1 fixed  |  rear wall X=-350 mm  |  +X forward, -Z down  |  2x playback" if right_angle
                    else "J1 target fixed at 0 deg  |  rear wall at X=-350 mm  |  DR_BASE +X forward")
        draw.text((33, 62), subtitle,
                  font=font(17), fill="#bbcedd")
        forward_mm = (tcp[index, 0] - tcp[0, 0]) * 1000
        if right_angle:
            down_mm = (tcp[0, 2] - tcp[index, 2]) * 1000
            phase_index = min(max(int((times[index] - 1) // 8), 0), 3)
            phase = ("FORWARD 1", "DOWN 1", "FORWARD 2", "DOWN 2")[phase_index]
            if times[index] < 1 or times[index] >= 33:
                phase = "HOLD"
            footer = (f"t={times[index]:.1f}s  |  {phase}  |  +X={forward_mm:.1f} mm  "
                      f"|  -Z={down_mm:.1f} mm  |  J1={np.degrees(positions[index,0]):.3f} deg")
        else:
            footer = (f"t={times[index]:.2f} s   |   TCP forward={forward_mm:.1f} mm   "
                      f"|   J1={np.degrees(positions[index,0]):.3f} deg   |   no wall contact")
        draw.text((32, 838), footer, font=font(18), fill="white")
        draw.rectangle((32, 872, 928, 879), fill="#425266")
        draw.rectangle((32, 872, 32 + round(896 * frame / (frame_count - 1)), 879), fill="#5ed7c5")
        return image

    with mujoco.Renderer(model, height=RENDER_SIZE[1], width=RENDER_SIZE[0],
                         max_geom=model.ngeom + len(tcp) + 16) as renderer:
        if preview_only:
            preview = output_path.with_suffix(".png")
            compose(renderer, frame_count // 2).save(preview)
            return preview
        command = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
                   "-pixel_format", "rgb24", "-video_size", "960x900", "-framerate", str(FPS),
                   "-i", "-", "-an", "-c:v", "libx264", "-crf", "18", "-preset", "medium",
                   "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output_path)]
        process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            for frame in range(frame_count):
                image = compose(renderer, frame)
                if frame == frame_count // 2:
                    image.save(output_path.with_name(output_path.stem + "_middle.png"))
                if frame == frame_count - 1:
                    image.save(output_path.with_name(output_path.stem + "_final.png"))
                process.stdin.write(image.tobytes())
            process.stdin.close()
            error = process.stderr.read().decode("utf-8", errors="replace")
            if process.wait() != 0:
                raise RuntimeError(f"ffmpeg failed: {error}")
        except Exception:
            process.kill()
            raise
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--states", type=Path,
                        default=ROOT / "results/forward_wall/forward_wall_simulation.npz")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "results/forward_wall/forward_wall_simulation.mp4")
    parser.add_argument("--preview-only", action="store_true")
    parser.add_argument("--right-angle", action="store_true")
    args = parser.parse_args()
    print(render(args.states.resolve(), args.output.resolve(), args.preview_only, args.right_angle))


if __name__ == "__main__":
    main()
