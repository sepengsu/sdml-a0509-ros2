"""Add TCC1 to a standalone A0509 MJCF using the shared Xacro parameters."""

from __future__ import annotations

from math import cos, isfinite, sin
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


GRIPPERS = {
    "ag145": ("ag145_base_link", "ag145_tcp", "tool_to_ag145"),
    "mdpg_1s1": ("mdpg_base_link", "mdpg_tcp", "tool_to_mdpg"),
}


def package_share() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("tcc1_tool_changer_description"))
    except (ImportError, LookupError):
        return Path(__file__).resolve().parents[1]


def _numbers(value, count: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != count or any(
        isinstance(item, bool) or not isinstance(item, (int, float)) or not isfinite(item)
        for item in value
    ):
        raise ValueError(f"TCC1 {field} requires {count} finite numbers")
    return tuple(float(item) for item in value)


def _pose(spec: dict, field: str) -> tuple[tuple[float, ...], tuple[float, ...]]:
    return (_numbers(spec.get("xyz_m"), 3, f"{field}.xyz_m"),
            _numbers(spec.get("rpy_rad"), 3, f"{field}.rpy_rad"))


def _inertial(component: dict, name: str) -> dict[str, str]:
    mass = _numbers([component.get("mass_kg")], 1, f"{name}.mass_kg")[0]
    com = _numbers(component.get("com_m"), 3, f"{name}.com_m")
    if component.get("inertia_kg_m2") is None:
        unit = _numbers(component.get("inertia_per_unit_mass_m2"), 6, f"{name}.inertia_per_unit_mass_m2")
        values = tuple(mass * value for value in unit)
    else:
        values = _numbers(component["inertia_kg_m2"], 6, f"{name}.inertia_kg_m2")
    xx, yy, zz, xy, xz, yz = values
    moments = np.linalg.eigvalsh(((xx, xy, xz), (xy, yy, yz), (xz, yz, zz)))
    if mass <= 0 or moments[0] <= 0 or moments[2] > moments[0] + moments[1] + moments[2] * 1e-10:
        raise ValueError(f"TCC1 {name} mass or principal moments are not physically valid")
    if not component.get("parameter_source") or not isinstance(component.get("verified"), bool):
        raise ValueError(f"TCC1 {name} requires a parameter source and verification status")
    return dict(mass=str(mass), pos=_vector(com), fullinertia=_vector(values))


def _active_accessories(config: dict, part: str):
    for name, entry in config.get("accessories", {}).get(part, {}).items():
        # An unknown mass deliberately leaves the optional part out of dynamics.
        if entry.get("mass_kg") is not None:
            yield name, entry


def load_configuration(path: str | Path | None, gripper: str, mode: str,
                       *, changer: bool = True, rack: bool = False) -> dict:
    if mode not in ("visual", "dynamics") or gripper not in ("none", *GRIPPERS):
        raise ValueError(f"Invalid TCC1 model options: {gripper=}, {mode=}")
    path = Path(path) if path else package_share() / "config/tcc1_parameters.yaml"
    config = yaml.safe_load(path.read_text(encoding="utf-8"))
    if config.get("version") != 1:
        raise ValueError("Unsupported TCC1 configuration version")
    if _numbers(config.get("mesh_scale"), 3, "mesh_scale") != (0.001,) * 3:
        raise ValueError("TCC1 raw CAD meshes are in millimetres and require a 0.001 scale")
    parts = ("master", "tool") if changer and gripper != "none" else ("master",) if changer else ()
    mounts = ["link_6_to_master"] if changer else []
    if changer and gripper != "none":
        mounts.extend(("master_to_tool", GRIPPERS[gripper][2]))
        _pose(config["gripper_tcp"][gripper], f"gripper_tcp.{gripper}")
    for mount in (*mounts, *(("world_to_rack",) if rack else ())):
        _pose(config["mounts"][mount], f"mounts.{mount}")
    for name in (*parts, *(("rack",) if rack else ())):
        comp = config["components"][name]
        _pose(comp["link_to_raw_cad"], f"{name}.link_to_raw_cad")
        for filename in {comp["mesh"], comp.get("mujoco_mesh", comp["mesh"])}:
            mesh = package_share() / filename
            if not mesh.is_file():
                raise FileNotFoundError(mesh)
        if mode == "dynamics" and name in parts:
            _inertial(comp, name)
            for label, accessory in _active_accessories(config, name):
                _inertial(accessory, f"accessories.{name}.{label}")
    return config


def _vector(values) -> str:
    return " ".join(f"{value:.12g}" for value in values)


def _mjcf_pose(spec: dict) -> dict[str, str]:
    rx, ry, rz = (angle / 2 for angle in spec["rpy_rad"])
    cr, cp, cy = cos(rx), cos(ry), cos(rz)
    sr, sp, sy = sin(rx), sin(ry), sin(rz)
    quat = (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)
    return dict(pos=_vector(spec["xyz_m"]), quat=_vector(quat))


def _standalone_tree(source: Path) -> ET.ElementTree:
    tree = ET.parse(source)
    scene = tree.getroot()
    compiler = scene.find("compiler")
    mesh_dir = source.parent / (compiler.attrib.pop("meshdir", "") if compiler is not None else "")
    # The stock bare scene uses one include for the arm. The gripper builders
    # already write an expanded scene with absolute mesh paths.
    include = scene.find("include")
    if include is not None:
        arm = ET.parse(source.parent / include.attrib["file"]).getroot()
        scene.remove(include)
        scene.find("asset").extend(list(arm.find("asset")))
        scene.find("worldbody").extend(list(arm.find("worldbody")))
        scene.append(arm.find("actuator"))
    for mesh in scene.findall("asset/mesh"):
        mesh.set("file", (mesh_dir / mesh.attrib["file"]).resolve().as_posix())
    return tree


def write_scene(source: str | Path, output: str | Path, gripper_type: str,
                *, tool_changer_type: str = "none", mode: str = "visual",
                rack: bool = False, config_path: str | Path | None = None) -> Path:
    """Reparent an existing gripper under the TCC1 fixed bodies, retaining payloads."""
    if tool_changer_type not in ("none", "tcc1"):
        raise ValueError(f"Unknown tool changer type: {tool_changer_type}")
    source, output = Path(source).resolve(), Path(output).resolve()
    changer = tool_changer_type == "tcc1"
    if not changer and not rack:
        # Return the source to keep the legacy scene and include paths intact.
        return source
    config = load_configuration(config_path, gripper_type, mode, changer=changer, rack=rack)
    tree = _standalone_tree(source)
    scene = tree.getroot()
    world = scene.find("worldbody")
    flange = world.find(".//body[@name='link_6']")
    assets = scene.find("asset")
    if flange is None or assets.find("mesh[@name='tcc1_master_mesh']") is not None:
        raise ValueError("Expected an A0509 scene without a TCC1 module")

    def add_part(name: str, parent: ET.Element, mount: str) -> ET.Element:
        component = config["components"][name]
        body = ET.SubElement(parent, "body", name=f"tcc1_{name}", **_mjcf_pose(config["mounts"][mount]))
        if mode == "dynamics" and name != "rack":
            ET.SubElement(body, "inertial", **_inertial(component, name))
            for label, accessory in _active_accessories(config, name):
                attachment = ET.SubElement(body, "body", name=f"tcc1_{name}_{label}")
                ET.SubElement(attachment, "inertial", **_inertial(accessory, f"accessories.{name}.{label}"))
        mesh = f"tcc1_{name}_mesh"
        ET.SubElement(assets, "mesh", name=mesh,
                      file=(package_share() / component.get("mujoco_mesh", component["mesh"])).resolve().as_posix(),
                      scale=_vector(config["mesh_scale"]))
        ET.SubElement(body, "geom", name=f"tcc1_{name}_visual", type="mesh", mesh=mesh,
                      **_mjcf_pose(component["link_to_raw_cad"]),
                      rgba=_vector(component["rgba"]), density="0", contype="0", conaffinity="0")
        return body

    if changer:
        master = add_part("master", flange, "link_6_to_master")
        if gripper_type != "none":
            name, tcp_name, mount = GRIPPERS[gripper_type]
            gripper = next((element for element in flange if element.get("name") == name
                            and element.tag in ("body", "frame")), None)
            if gripper is None:
                raise ValueError(f"Existing scene is missing gripper {name} on link_6")
            tool = add_part("tool", master, "master_to_tool")
            flange.remove(gripper)
            tool.append(gripper)
            gripper.attrib.update(_mjcf_pose(config["mounts"][mount]))
            # mDPG originally puts the TCP inside its CAD correction frame.
            tcp_parent = gripper.find(f".//site[@name='{tcp_name}']/..")
            if tcp_parent is None:
                raise ValueError(f"Existing gripper is missing TCP {tcp_name}")
            tcp = tcp_parent.find(f"site[@name='{tcp_name}']")
            tcp_parent.remove(tcp)
            tcp.attrib.update(_mjcf_pose(config["gripper_tcp"][gripper_type]))
            gripper.append(tcp)
    if rack:
        if world.find("body[@name='tcc1_rack']") is not None:
            raise ValueError("The scene already contains a TCC1 Rack")
        add_part("rack", world, "world_to_rack")
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output
