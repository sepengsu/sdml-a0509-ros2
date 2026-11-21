"""Render the A0509 practice Home before and after changing only J1."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import mujoco
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "src/doosan-robot2/dsr_description2/mujoco_models/a0509/scene.xml"
OLD_HOME_DEG = (-41.36, -32.01, 84.97994995117188, -3.99, 115.02, 45.0)
NEW_HOME_DEG = (0.0, -32.01, 84.97994995117188, -3.99, 115.02, 45.0)
SIZE = (1000, 800)


def _font(size: int, *, bold: bool = False):
    name = "arialbd.ttf" if bold else "arial.ttf"
    path = Path("C:/Windows/Fonts") / name
    return ImageFont.truetype(str(path), size) if path.exists() else ImageFont.load_default()


def render_pair(output_dir: Path, azimuth_deg: float = 305) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(SCENE))
    model.vis.global_.offwidth = SIZE[0]
    model.vis.global_.offheight = SIZE[1]
    data = mujoco.MjData(model)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.05, 0.0, 0.48)
    camera.distance = 1.55
    camera.azimuth = azimuth_deg
    camera.elevation = -22
    paths = []
    with mujoco.Renderer(model, height=SIZE[1], width=SIZE[0]) as renderer:
        for stem, title, pose, accent in (
            ("before", "BEFORE  |  J1 = -41.36 deg", OLD_HOME_DEG, "#f5a34a"),
            ("after", "AFTER  |  J1 = 0 deg", NEW_HOME_DEG, "#5ed7c5"),
        ):
            for index, joint_name in enumerate(f"joint_{i}" for i in range(1, 7)):
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                data.qpos[model.jnt_qposadr[joint_id]] = math.radians(pose[index])
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera=camera)
            picture = Image.fromarray(renderer.render().copy())
            canvas = Image.new("RGB", (SIZE[0], SIZE[1] + 150), "#17202d")
            canvas.paste(picture, (0, 100))
            draw = ImageDraw.Draw(canvas)
            draw.rectangle((0, 0, SIZE[0], 100), fill="#17202d")
            draw.rectangle((0, 0, 12, 100), fill=accent)
            draw.text((34, 19), title, font=_font(34, bold=True), fill="#ffffff")
            draw.text((35, 66), "A0509 MuJoCo model  |  same camera, only J1 changed",
                      font=_font(20), fill="#b8c7d9")
            draw.text((34, SIZE[1] + 112),
                      "J1..J6 (deg): " + ", ".join(f"{value:.2f}" for value in pose),
                      font=_font(20), fill="#ffffff")
            target = output_dir / f"home_pose_{stem}.png"
            canvas.save(target)
            paths.append(target)
    return paths[0], paths[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path,
                        default=ROOT / "results/home_pose_comparison")
    parser.add_argument("--azimuth", type=float, default=305)
    args = parser.parse_args()
    for path in render_pair(args.output_dir.resolve(), args.azimuth):
        print(path)


if __name__ == "__main__":
    main()
