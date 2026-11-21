"""Combine the copied 60th obstacle scene with this workspace's rear wall.

This produces a local MuJoCo comparison scene; it does not alter doosan-robot2
or declare the physical robot workspace safe.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path
import tarfile
import xml.etree.ElementTree as ET

from .lr_trajectory import _scene_path


CAPSULE_SCENE = "dsr_description2/mujoco_models/a0509/scene.xml"
CAPSULE_MESH = "dsr_description2/mujoco_models/a0509/assets/time capsule.stl"


def build_60th_front_scene(archive: Path, output_dir: Path) -> Path:
    archive = Path(archive).resolve()
    output_dir = Path(output_dir).resolve()
    wall_scene = _scene_path()
    if not archive.is_file():
        raise FileNotFoundError(archive)
    if not wall_scene.is_file():
        raise FileNotFoundError(wall_scene)
    with tarfile.open(archive, "r:gz") as bundle:
        for name in (CAPSULE_SCENE, CAPSULE_MESH):
            member = bundle.getmember(name)
            if not member.isfile():
                raise ValueError(f"archive member is not a file: {name}")
        original_scene = bundle.extractfile(CAPSULE_SCENE).read()
        capsule_mesh = bundle.extractfile(CAPSULE_MESH).read()
    source = ET.parse(io.BytesIO(original_scene)).getroot()
    mesh = source.find("./asset/mesh[@name='time_capsule_mesh']")
    capsule = source.find("./worldbody/body[@name='time_capsule_body']")
    if mesh is None or capsule is None:
        raise ValueError("60th archive has no named time capsule mesh and body")
    output_dir.mkdir(parents=True, exist_ok=True)
    mesh_path = output_dir / "time capsule.stl"
    mesh_path.write_bytes(capsule_mesh)
    scene = ET.parse(wall_scene).getroot()
    include = scene.find("include")
    assets, world = scene.find("asset"), scene.find("worldbody")
    if include is None or assets is None or world is None:
        raise ValueError("rear-wall scene lacks arm, asset or worldbody")
    include.set("file", (wall_scene.parent / include.attrib["file"]).resolve().as_posix())
    mesh.set("file", mesh_path.as_posix())
    assets.append(mesh)
    world.append(capsule)
    output = output_dir / "scene_forward_wall_60th_capsule.xml"
    ET.indent(scene)
    ET.ElementTree(scene).write(output, encoding="utf-8", xml_declaration=True)
    (output_dir / "scene_sources.json").write_text(json.dumps({
        "rear_wall_scene": str(wall_scene),
        "archive": str(archive),
        "archive_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        "capsule_scene_member": CAPSULE_SCENE,
        "capsule_mesh_member": CAPSULE_MESH,
        "physical_obstacle_position_verified": False,
        "execution_scope": "simulation_only",
    }, indent=2) + "\n", encoding="utf-8")
    return output


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Build 60th model + rear wall comparison scene")
    parser.add_argument("--archive", type=Path,
                        default=Path.cwd() / "doosan_changes_export.tar.gz")
    parser.add_argument("--output-dir", type=Path,
                        default=Path.cwd() / "results/front_60th_scene")
    args = parser.parse_args(argv)
    print(build_60th_front_scene(args.archive, args.output_dir))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
