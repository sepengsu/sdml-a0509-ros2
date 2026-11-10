"""Attach the CAD-derived mDPG visuals to A0509 without inventing dynamics."""

from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET


PARTS = {
    "pump_cover": "0.78 0.80 0.83 1",
    "led_indicator": "1.0 0.65 0.06 1",
    "bottom_cover": "0.32 0.34 0.37 1",
    "solid33": "0.12 0.12 0.14 1",
    "air_pad_body": "0.18 0.19 0.21 1",
    "air_pad_contact": "0.10 0.27 0.45 1",
}
CAD_NOMINAL_TCP_Z_M = -0.09  # CAD bound, not a calibrated tool frame.


def _share(package: str, local_parts: tuple[str, ...]) -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        found = Path(get_package_share_directory(package))
        if found.is_dir():
            return found
    except (ImportError, LookupError, OSError):
        pass
    repo = Path(__file__).resolve().parents[3]
    candidate = repo.joinpath("src", *local_parts)
    if candidate.is_dir():
        return candidate
    raise FileNotFoundError(f"{package} share directory unavailable")


def _triple(values, label: str) -> tuple[float, float, float]:
    if values is None:
        raise ValueError(f"{label} is TODO: provide a measured mount transform")
    if isinstance(values, str):
        values = values.split()
    try:
        result = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} must contain three finite numbers") from error
    if len(result) != 3 or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} must contain three finite numbers")
    return result


def _rpy_quat(rpy: tuple[float, float, float]) -> str:
    roll, pitch, yaw = (value / 2 for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    quat = (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)
    return " ".join(f"{value:.12g}" for value in quat)


def build_mdpg_scene(output: str | Path, mount_xyz, mount_rpy,
                     arm_scene: str | Path | None = None,
                     gripper_share: str | Path | None = None) -> Path:
    """Generate a visual-only MJCF; the tool adds zero mass and no contacts.

    The caller must supply the flange transform. Mass, inertia, vacuum and
    pneumatic IO are intentionally absent until measured values are provided.
    """
    xyz = _triple(mount_xyz, "mount_xyz")
    rpy = _triple(mount_rpy, "mount_rpy")
    if arm_scene is None:
        arm_scene = _share("dsr_description2", ("doosan-robot2", "dsr_description2")) / "mujoco_models/a0509/scene.xml"
    if gripper_share is None:
        gripper_share = _share("mdpg_1s1_description", ("mdpg_1s1_description",))
    arm_scene = Path(arm_scene).resolve()
    gripper_share = Path(gripper_share).resolve()
    scene = ET.parse(arm_scene).getroot()
    include = scene.find("include")
    if include is None:
        raise ValueError("A0509 MuJoCo scene is missing its arm include")
    arm = ET.parse(arm_scene.parent / include.attrib["file"]).getroot()
    scene.remove(include)
    compiler = scene.find("compiler")
    if compiler is not None:
        compiler.attrib.pop("meshdir", None)
    assets = scene.find("asset")
    worldbody = scene.find("worldbody")
    arm_assets = arm.find("asset")
    arm_worldbody = arm.find("worldbody")
    if any(item is None for item in (assets, worldbody, arm_assets, arm_worldbody)):
        raise ValueError("A0509 scene lacks assets or worldbody")
    for mesh in arm_assets.findall("mesh"):
        mesh.set("file", (arm_scene.parent.parent / mesh.attrib["file"]).resolve().as_posix())
    assets.extend(list(arm_assets))
    worldbody.extend(list(arm_worldbody))
    actuator = arm.find("actuator")
    if actuator is not None:
        scene.append(actuator)
    flange = worldbody.find(".//body[@name='link_6']")
    if flange is None:
        raise ValueError("A0509 model has no link_6")
    # A MJCF frame attaches visuals to link_6 without creating a massless body.
    tool = ET.SubElement(flange, "frame", name="mdpg_base_link",
                         pos=" ".join(str(value) for value in xyz), quat=_rpy_quat(rpy))
    ET.SubElement(tool, "site", name="mdpg_tcp", pos=f"0 0 {CAD_NOMINAL_TCP_Z_M}",
                  size="0.004", rgba="0.1 0.5 1 1")
    for part, color in PARTS.items():
        mesh_file = gripper_share / "meshes" / f"{part}.stl"
        if not mesh_file.is_file():
            raise FileNotFoundError(mesh_file)
        ET.SubElement(assets, "mesh", name=f"mdpg_{part}",
                      file=mesh_file.as_posix(), scale="0.001 0.001 0.001")
        ET.SubElement(tool, "geom", name=f"mdpg_{part}_visual", type="mesh",
                      mesh=f"mdpg_{part}", rgba=color, density="0",
                      contype="0", conaffinity="0", group="2")
    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(scene)
    ET.ElementTree(scene).write(output, encoding="utf-8", xml_declaration=True)
    return output
