"""Render slide-ready A0509 views from the repository URDF and MJCF models."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont


ROOT = Path(__file__).resolve().parents[3]
DESCRIPTION = ROOT / "src/doosan-robot2/dsr_description2"
URDF_SOURCE = DESCRIPTION / "urdf/a0509.urdf"
MJCF_SOURCE = DESCRIPTION / "mujoco_models/a0509/scene.xml"
MJCF_ASSETS = DESCRIPTION / "mujoco_models/a0509/assets"
OUTPUT = ROOT / "results/ppt_a0509_models"
RENDER_CACHE = OUTPUT
POSE = (-0.5, -0.5, 1.6, 0.0, -0.6, 0.0)


def font(size: int):
    path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def converted_urdf() -> Path:
    """Point URDF visual meshes at the same named local STL conversions.

    MuJoCo cannot read package:// Collada URIs directly. The original URDF is
    never changed; this render copy keeps its links, transforms and inertials.
    """
    tree = ET.parse(URDF_SOURCE)
    robot = tree.getroot()
    count = 0
    for link in robot.findall("link"):
        for collision in link.findall("collision"):
            link.remove(collision)
        for index, visual in enumerate(link.findall("visual")):
            visual.set("name", f"{link.attrib['name']}_visual_{index}")
        for mesh in link.findall("visual/geometry/mesh"):
            stem = Path(mesh.attrib["filename"]).stem
            stl = MJCF_ASSETS / f"{stem}.stl"
            if not stl.is_file():
                raise FileNotFoundError(f"URDF visual has no matching STL: {stem}")
            mesh.set("filename", stl.resolve().as_posix())
            count += 1
    if count < 6:
        raise ValueError(f"Expected A0509 link visuals, found {count}")
    extension = ET.SubElement(robot, "mujoco")
    ET.SubElement(extension, "compiler", discardvisual="false", strippath="false")
    output = RENDER_CACHE / "a0509_render_copy.urdf"
    ET.indent(tree)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output


def camera():
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = (0.10, 0.0, 0.48)
    cam.distance = 1.42
    cam.azimuth = 125
    cam.elevation = -20
    return cam


def studio_image(pixels: np.ndarray, segmentation: np.ndarray, model: mujoco.MjModel) -> Image.Image:
    robot_ids = [index for index in range(model.ngeom)
                 if model.geom(index).name != "floor"]
    visible = (np.isin(segmentation[:, :, 0], robot_ids)
               & (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM)))
    mask = Image.fromarray((visible * 255).astype(np.uint8), mode="L")
    bounds = mask.getbbox()
    if bounds is None:
        raise RuntimeError("robot is absent from segmentation render")
    width, height = mask.size
    top = np.array((248, 250, 252), dtype=float)
    bottom = np.array((222, 232, 241), dtype=float)
    gradient = np.linspace(0, 1, height)[:, None, None]
    background = np.broadcast_to(top[None, None, :] * (1 - gradient)
                                 + bottom[None, None, :] * gradient,
                                 (height, width, 3)).astype(np.uint8).copy()
    image = Image.fromarray(background, mode="RGB").convert("RGBA")
    shadow = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    draw = ImageDraw.Draw(shadow)
    middle = (bounds[0] + bounds[2]) // 2
    ground = min(height - 36, bounds[3] + 4)
    draw.ellipse((middle - 180, ground - 10, middle + 180, ground + 38),
                 fill=(35, 54, 77, 60))
    image = Image.alpha_composite(image, shadow.filter(ImageFilter.GaussianBlur(18)))
    image.paste(Image.fromarray(pixels, mode="RGB"), (0, 0), mask)
    return image.convert("RGB")


def render_model(path: Path, kind: str) -> tuple[Image.Image, Image.Image]:
    model = mujoco.MjModel.from_xml_path(str(path))
    if model.nq != 6:
        raise ValueError(f"{kind} model has {model.nq} positions rather than six")
    data = mujoco.MjData(model)
    data.qpos[:] = POSE
    mujoco.mj_forward(model, data)
    model.vis.global_.offwidth = 1400
    model.vis.global_.offheight = 1400
    if kind == "urdf":
        # STL geometry is shared with MJCF; URDF source uses white DAE visuals.
        model.geom_rgba[:, :] = (0.93, 0.95, 0.98, 1.0)
        model.vis.headlight.diffuse[:] = (0.80, 0.80, 0.80)
        model.vis.headlight.ambient[:] = (0.35, 0.35, 0.35)
    with mujoco.Renderer(model, width=1200, height=1200) as renderer:
        renderer.update_scene(data, camera=camera())
        pixels = np.asarray(renderer.render())
        renderer.enable_segmentation_rendering()
        segmentation = np.asarray(renderer.render())
    robot_ids = [index for index in range(model.ngeom) if model.geom(index).name != "floor"]
    visible = (np.isin(segmentation[:, :, 0], robot_ids)
               & (segmentation[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM)))
    cutout = Image.fromarray(pixels, mode="RGB").convert("RGBA")
    cutout.putalpha(Image.fromarray((visible * 255).astype(np.uint8), mode="L"))
    return studio_image(pixels, segmentation, model), cutout


def slide_card(model_image: Image.Image, label: str, subtitle: str) -> Image.Image:
    canvas = Image.new("RGB", (1200, 1350), "#f3f5f8")
    canvas.paste(model_image, (0, 110))
    draw = ImageDraw.Draw(canvas)
    draw.text((55, 30), label, fill="#172536", font=font(44))
    draw.text((56, 89), subtitle, fill="#52647a", font=font(22))
    return canvas


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    RENDER_CACHE.mkdir(parents=True, exist_ok=True)
    urdf_copy = converted_urdf()
    urdf_studio, urdf_cutout = render_model(urdf_copy, "urdf")
    mjcf_studio, mjcf_cutout = render_model(MJCF_SOURCE, "mjcf")
    urdf_image = slide_card(urdf_studio, "A0509  |  URDF", "ROS 2 robot description")
    mjcf_image = slide_card(mjcf_studio, "A0509  |  MJCF", "MuJoCo simulation model")
    urdf_image.save(OUTPUT / "a0509_urdf.png")
    mjcf_image.save(OUTPUT / "a0509_mjcf.png")
    urdf_cutout.save(OUTPUT / "a0509_urdf_transparent.png")
    mjcf_cutout.save(OUTPUT / "a0509_mjcf_transparent.png")
    comparison = Image.new("RGB", (2400, 1350), "#f3f5f8")
    comparison.paste(urdf_image, (0, 0))
    comparison.paste(mjcf_image, (1200, 0))
    ImageDraw.Draw(comparison).line((1200, 0, 1200, 1350), fill="#d5dfe8", width=3)
    comparison.save(OUTPUT / "a0509_urdf_vs_mjcf_16x9.png")
    manifest = {
        "urdf_source": str(URDF_SOURCE),
        "urdf_sha256": hashlib.sha256(URDF_SOURCE.read_bytes()).hexdigest(),
        "mjcf_source": str(MJCF_SOURCE),
        "mjcf_sha256": hashlib.sha256(MJCF_SOURCE.read_bytes()).hexdigest(),
        "pose_rad": list(POSE),
        "urdf_render_note": "Original URDF links/transforms/inertia; matching A0509 STL visual geometry used because MuJoCo cannot read package:// DAE URLs directly.",
        "mjcf_render_note": "Original MuJoCo scene and material colors.",
        "images": ["a0509_urdf.png", "a0509_mjcf.png", "a0509_urdf_vs_mjcf_16x9.png",
                   "a0509_urdf_transparent.png", "a0509_mjcf_transparent.png"],
    }
    (OUTPUT / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    for image in manifest["images"]:
        print(OUTPUT / image)


if __name__ == "__main__":
    main()
