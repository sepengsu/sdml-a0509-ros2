"""Render the seven declared A0509 simulation conditions without running ROS.

Images are MuJoCo model renders at one shared joint pose, not experiment samples.
The mDPG red-face mount uses the explicitly unmeasured CAD-center fixture.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from a0509_dynamics_experiment.ag145_model import (AG145_BOX_GRASP_ANGLE_RAD,
                                                   build_ag145_scene, source_paths)
from a0509_dynamics_experiment.payload_model import attach_payload, load_payload_spec
from mdpg_1s1_description.model import build_mdpg_scene


ROOT = Path(__file__).resolve().parents[3]
OUTPUT = ROOT / "results" / "condition_gallery"
PAYLOAD_CONFIG = ROOT / "src/a0509_dynamics_experiment/config/payload_conditions.json"
JOINT_POSE_RAD = (-0.5, -0.5, 1.6, 0.0, -0.6, 0.0)
CONDITIONS = (
    ("bare_p0", "none", 0.0),
    ("ag145_p0", "ag145", 0.0),
    ("ag145_p05", "ag145", 0.5),
    ("ag145_p10", "ag145", 1.0),
    ("mdpg_p0", "mdpg_1s1", 0.0),
    ("mdpg_p05", "mdpg_1s1", 0.5),
    ("mdpg_p10", "mdpg_1s1", 1.0),
)


def _font(size: int):
    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def _camera(lookat, distance: float):
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = lookat
    camera.distance = distance
    camera.azimuth = 125
    camera.elevation = -22
    return camera


def _scene(condition: str, gripper: str, mass: float) -> tuple[Path, dict | None]:
    folder = OUTPUT / condition
    folder.mkdir(parents=True, exist_ok=True)
    if gripper == "none":
        scene = source_paths()[0]
    elif gripper == "ag145":
        scene = build_ag145_scene(folder / "scene.xml",
                                 grasp_angle_rad=AG145_BOX_GRASP_ANGLE_RAD if mass else None)
    else:
        scene = build_mdpg_scene(folder / "scene.xml", (0, 0, 0), (0, 0, 0),
                                 dynamics_model="box_approx")
    payload = None
    if mass:
        payload = load_payload_spec(PAYLOAD_CONFIG, gripper, mass)
        attach_payload(scene, payload)
    return Path(scene), payload


def render_condition(condition: str, gripper: str, mass: float) -> dict:
    scene, payload = _scene(condition, gripper, mass)
    model = mujoco.MjModel.from_xml_path(str(scene))
    if model.nq != 6:
        raise ValueError(f"{condition}: expected six robot joints, found {model.nq}")
    data = mujoco.MjData(model)
    data.qpos[:] = JOINT_POSE_RAD
    mujoco.mj_forward(model, data)

    flange = model.body("link_6").id
    tool_name = {"none": None, "ag145": "ag145_tcp", "mdpg_1s1": "mdpg_tcp"}[gripper]
    focus = data.site_xpos[model.site(tool_name).id].copy() if tool_name else data.xpos[flange].copy()
    close_distance = 0.43 if gripper == "ag145" else (0.33 if gripper == "mdpg_1s1" else 0.30)
    model.vis.global_.offwidth = 1600
    model.vis.global_.offheight = 900
    with (mujoco.Renderer(model, height=800, width=1000) as full_renderer,
          mujoco.Renderer(model, height=800, width=600) as close_renderer):
        full_renderer.update_scene(data, camera=_camera((0.05, -0.02, 0.54), 1.5))
        full = full_renderer.render()
        close_renderer.update_scene(data, camera=_camera(focus, close_distance))
        close = close_renderer.render()

    canvas = Image.new("RGB", (1600, 900), "#17202c")
    canvas.paste(Image.fromarray(np.asarray(full)), (0, 70))
    canvas.paste(Image.fromarray(np.asarray(close)), (1000, 70))
    draw = ImageDraw.Draw(canvas)
    title = f"A0509 | {condition} | payload {mass:.1f} kg"
    draw.text((22, 14), title, font=_font(28), fill="white")
    draw.text((22, 50), "Full robot", font=_font(14), fill="#e4eaf0")
    draw.text((1020, 50), "Tool close-up", font=_font(14), fill="#e4eaf0")
    draw.text((22, 875), "MuJoCo model render; fixed joint pose, no trajectory execution", font=_font(14), fill="#e4eaf0")
    if gripper == "mdpg_1s1":
        draw.text((1018, 875), "CAD-face fixture, unmeasured", font=_font(14), fill="#ffc76a")
    output = OUTPUT / f"{condition}.png"
    canvas.save(output)
    return {
        "condition": condition,
        "gripper_type": gripper,
        "payload_mass_kg": mass,
        "image": output.name,
        "scene": str(scene),
        "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
        "joint_pose_rad": list(JOINT_POSE_RAD),
        "mdpg_mount_xyz_m": [0, 0, 0] if gripper == "mdpg_1s1" else None,
        "mdpg_mount_rpy_rad": [0, 0, 0] if gripper == "mdpg_1s1" else None,
        "mdpg_mount_verified": False if gripper == "mdpg_1s1" else None,
        "ag145_grasp_angle_rad": AG145_BOX_GRASP_ANGLE_RAD if gripper == "ag145" and mass else None,
        "payload_spec_sha256": payload["spec_sha256"] if payload else None,
        "render_only": True,
    }


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    records = [render_condition(*condition) for condition in CONDITIONS]
    (OUTPUT / "manifest.json").write_text(json.dumps(records, indent=2) + "\n", encoding="utf-8")
    for record in records:
        print(OUTPUT / record["image"])


if __name__ == "__main__":
    main()
