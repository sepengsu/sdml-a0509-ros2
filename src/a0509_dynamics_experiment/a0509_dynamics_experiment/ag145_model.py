"""Build an A0509 MuJoCo scene with the repository's DH AG145 meshes.

The gripper is fixed at its open URDF pose. Its 1.3 kg mass is included as a
single approximate rigid body; finger actuation and calibrated inertia are not
claimed by this model.
"""

from __future__ import annotations

from collections import defaultdict
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET


AG145_MASS_KG = 1.3
AG145_TCP_OFFSET_MM = (0.0, 0.0, 200.0)
AG145_MOUNT_RPY = (0.0, 0.0, 0.7853981634)


def _urdf_quat(rpy: str) -> str:
    """Convert URDF fixed-axis roll/pitch/yaw to an MJCF wxyz quaternion."""
    roll, pitch, yaw = (float(value) / 2 for value in rpy.split())
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    values = (cy * cp * cr + sy * sp * sr,
              cy * cp * sr - sy * sp * cr,
              cy * sp * cr + sy * cp * sr,
              sy * cp * cr - cy * sp * sr)
    return " ".join(f"{value:.12g}" for value in values)


def _package_share(package: str, local_parts: tuple[str, ...]) -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory(package))
        if path.is_dir():
            return path
    except (ImportError, LookupError, OSError):
        pass
    roots = (Path.cwd(), Path(__file__).resolve().parents[2].parent)
    for root in roots:
        candidate = root.joinpath("src", *local_parts)
        if candidate.is_dir():
            return candidate
    raise FileNotFoundError(f"{package} model package not found")


def source_paths() -> tuple[Path, Path]:
    arm = _package_share("dsr_description2", ("doosan-robot2", "dsr_description2"))
    gripper = _package_share("dh_ag145_description", ("dh_gripper", "dh_gripper_description", "dh_ag145_description"))
    return arm / "mujoco_models" / "a0509" / "scene.xml", gripper / "urdf" / "dh_ag145.urdf"


def _compatible_stl(path: Path, output_dir: Path, target_faces: int = 8_000) -> Path:
    """Reduce STL triangles for tractable software rendering in MuJoCo."""
    import numpy as np

    raw = path.read_bytes()
    if len(raw) < 84:
        raise ValueError(f"invalid STL: {path}")
    faces = struct.unpack_from("<I", raw, 80)[0]
    if len(raw) != 84 + 50 * faces:
        raise ValueError(f"invalid binary STL: {path}")
    if faces <= target_faces:
        return path
    triangle_dtype = np.dtype([("normal", "<f4", (3,)),
                               ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    triangles = np.frombuffer(raw, dtype=triangle_dtype, offset=84, count=faces)
    vertices = triangles["vertices"].reshape(-1, 3).astype(np.float64)
    try:
        import fast_simplification
    except ImportError:
        fast_simplification = None
    if fast_simplification is not None:
        centers, vertex_ids = np.unique(np.round(vertices, 5), axis=0, return_inverse=True)
        centers, indices = fast_simplification.simplify(
            centers, vertex_ids.reshape(-1, 3).astype(np.int32),
            target_count=target_faces, agg=5.0,
        )
    else:
        spacing = float(np.max(np.ptp(vertices, axis=0))) / 120
        for _ in range(12):
            bins = np.rint(vertices / spacing).astype(np.int32)
            _, vertex_ids = np.unique(bins, axis=0, return_inverse=True)
            counts = np.bincount(vertex_ids)
            centers = np.column_stack([
                np.bincount(vertex_ids, weights=vertices[:, axis]) / counts
                for axis in range(3)
            ])
            indices = vertex_ids.reshape(-1, 3)
            distinct = ((indices[:, 0] != indices[:, 1]) &
                        (indices[:, 0] != indices[:, 2]) &
                        (indices[:, 1] != indices[:, 2]))
            indices = indices[distinct]
            _, unique_at = np.unique(np.sort(indices, axis=1), axis=0, return_index=True)
            indices = indices[np.sort(unique_at)]
            if 0 < len(indices) <= target_faces:
                break
            spacing *= 1.5
        else:
            raise ValueError(f"could not simplify AG145 mesh: {path}")
    positions = centers[indices].astype(np.float32)
    normals = np.cross(positions[:, 1] - positions[:, 0],
                       positions[:, 2] - positions[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > 1e-8
    positions = positions[valid]
    normals = normals[valid] / lengths[valid, None]
    output_triangles = np.zeros(len(positions), dtype=triangle_dtype)
    output_triangles["normal"] = normals
    output_triangles["vertices"] = positions
    output_dir.mkdir(parents=True, exist_ok=True)
    reduced = output_dir / path.name
    reduced.write_bytes(raw[:80] + struct.pack("<I", len(positions)) + output_triangles.tobytes())
    return reduced


def build_ag145_scene(output: str | Path, arm_scene: str | Path | None = None,
                      gripper_urdf: str | Path | None = None) -> Path:
    """Write a self-contained XML that references installed mesh assets."""
    output = Path(output)
    if arm_scene is None or gripper_urdf is None:
        found_scene, found_urdf = source_paths()
        arm_scene = arm_scene or found_scene
        gripper_urdf = gripper_urdf or found_urdf
    arm_scene = Path(arm_scene).resolve()
    gripper_urdf = Path(gripper_urdf).resolve()
    if not arm_scene.is_file() or not gripper_urdf.is_file():
        raise FileNotFoundError("A0509 scene or AG145 URDF missing")

    scene = ET.parse(arm_scene).getroot()
    include = scene.find("include")
    if include is None:
        raise ValueError("A0509 scene does not include the arm model")
    arm_model = ET.parse(arm_scene.parent / include.attrib["file"]).getroot()
    scene.remove(include)
    compiler = scene.find("compiler")
    if compiler is not None:
        compiler.attrib.pop("meshdir", None)
    assets = scene.find("asset")
    worldbody = scene.find("worldbody")
    if assets is None or worldbody is None:
        raise ValueError("A0509 scene missing assets or worldbody")
    arm_assets = arm_model.find("asset")
    arm_worldbody = arm_model.find("worldbody")
    if arm_assets is None or arm_worldbody is None:
        raise ValueError("A0509 arm model missing assets or worldbody")
    for mesh in arm_assets.findall("mesh"):
        relative = mesh.attrib["file"]
        mesh.set("file", (arm_scene.parent.parent / relative).resolve().as_posix())
    assets.extend(list(arm_assets))
    worldbody.extend(list(arm_worldbody))
    actuator = arm_model.find("actuator")
    if actuator is not None:
        scene.append(actuator)

    flange = worldbody.find(".//body[@name='link_6']")
    if flange is None:
        raise ValueError("A0509 arm model missing link_6")
    urdf = ET.parse(gripper_urdf).getroot()
    links = {link.attrib["name"]: link for link in urdf.findall("link")}
    children = defaultdict(list)
    for joint in urdf.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is not None and child is not None and child.attrib["link"] in links:
            children[parent.attrib["link"]].append((joint, child.attrib["link"]))

    mount = ET.SubElement(flange, "body", name="ag145_base_link", pos="0 0 0",
                          quat=_urdf_quat(" ".join(str(x) for x in AG145_MOUNT_RPY)))
    # Total gripper mass is from the manufacturer. Inertia uses a coarse box
    # envelope because the repository URDF supplies no calibrated inertials.
    ET.SubElement(mount, "inertial", pos="0 0 0.10", mass=str(AG145_MASS_KG),
                  diaginertia="0.0073 0.0050 0.0033")
    ET.SubElement(mount, "site", name="ag145_tcp", pos="0 0 0.20",
                  size="0.006", rgba="1 0.2 0.1 1")
    ET.SubElement(mount, "geom", name="ag145_body_collision", type="box",
                  pos="0 0 0.075", size="0.035 0.035 0.075", density="0",
                  rgba="0 0 0 0", contype="1", conaffinity="1")
    for side, x in (("left", -0.065), ("right", 0.065)):
        ET.SubElement(mount, "geom", name=f"ag145_{side}_finger_collision",
                      type="box", pos=f"{x} 0 0.175", size="0.012 0.018 0.028",
                      density="0", rgba="0 0 0 0", contype="1", conaffinity="1")

    gripper_share = gripper_urdf.parent.parent
    seen: set[str] = set()

    def add_link(link_name: str, parent_body: ET.Element) -> None:
        link = links[link_name]
        visual = link.find("visual")
        if visual is not None:
            mesh = visual.find("geometry/mesh")
            if mesh is not None:
                filename = mesh.attrib["filename"].split("/meshes/", 1)[-1]
                mesh_name = "ag145_" + Path(filename).stem
                if mesh_name not in seen:
                    mesh_file = gripper_share / "meshes" / filename
                    if not mesh_file.is_file():
                        raise FileNotFoundError(mesh_file)
                    mesh_file = _compatible_stl(mesh_file, output.parent / "ag145_meshes")
                    ET.SubElement(assets, "mesh", name=mesh_name,
                                  file=mesh_file.resolve().as_posix(),
                                  scale=mesh.attrib.get("scale", "0.001 0.001 0.001"))
                    seen.add(mesh_name)
                origin = visual.find("origin")
                color = visual.find("material/color")
                ET.SubElement(parent_body, "geom", name=f"ag145_{link_name}_visual",
                              type="mesh", mesh=mesh_name, density="0", contype="0",
                              conaffinity="0", pos=origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0",
                              quat=_urdf_quat(origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0"),
                              rgba=color.attrib.get("rgba", "0.4 0.4 0.4 1") if color is not None else "0.4 0.4 0.4 1")
        for joint, child_name in children[link_name]:
            origin = joint.find("origin")
            body = ET.SubElement(parent_body, "body", name=f"ag145_{child_name}",
                                 pos=origin.attrib.get("xyz", "0 0 0") if origin is not None else "0 0 0",
                                 quat=_urdf_quat(origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0"))
            add_link(child_name, body)

    add_link("base_link", mount)
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(scene)
    ET.ElementTree(scene).write(output, encoding="utf-8", xml_declaration=True)
    return output
