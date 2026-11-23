"""Original A0509 assets and T1-T16 geometry, resolved from ROS2 or source."""
import importlib
import inspect
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from a0509_dynamics_experiment.wide16_trajectory import DESIGNS, AMPLITUDE_SCALE, DURATION_S, RAMP_S

ROOT = next((p for p in Path(__file__).resolve().parents if (p / "src/doosan-robot2").is_dir()), None)
CONDITIONS = ("bare_p0", "ag145_p0", "ag145_p05", "ag145_p10", "mdpg_p0", "mdpg_p05", "mdpg_p10")
SPLITS = {**{f"T{i}": "train" for i in (*range(1, 12), 15)}, "T12": "validation",
          **{f"T{i}": "test" for i in (13, 14, 16)}}
TORQUE_LIMITS = np.array((194., 194., 163., 50., 50., 50.))
LOCAL_PARTS = {
    "dsr_description2": "doosan-robot2/dsr_description2",
    "a0509_dynamics_experiment": "a0509_dynamics_experiment",
    "dh_ag145_description": "dh_gripper/dh_gripper_description/dh_ag145_description",
    "mdpg_1s1_description": "mdpg_1s1_description",
}


def package_share(name):
    try:
        from ament_index_python.packages import get_package_share_directory
        directory = Path(get_package_share_directory(name))
        if directory.is_dir():
            return directory
    except (ImportError, LookupError, OSError):
        pass
    if ROOT is not None:
        directory = ROOT / "src" / LOCAL_PARTS[name]
        if directory.is_dir():
            return directory
    raise FileNotFoundError(f"Required model package unavailable: {name}")


def asset_files():
    result = {}
    for name, relative in (("dsr_description2", "mujoco_models/a0509"),
                           ("dh_ag145_description", "meshes"),
                           ("dh_ag145_description", "urdf"),
                           ("mdpg_1s1_description", "meshes")):
        share = package_share(name)
        for path in (share / relative).rglob("*"):
            if path.is_file():
                result[f"{name}/{path.relative_to(share).as_posix()}"] = path
    result["a0509_dynamics_experiment/config/payload_conditions.json"] = (
        package_share("a0509_dynamics_experiment") / "config/payload_conditions.json")
    for name in ("a0509_dynamics_experiment", "mdpg_1s1_description"):
        directory = Path(importlib.import_module(name).__file__).parent
        result.update({f"{name}/{p.name}": p for p in directory.glob("*.py")})
    return result


def _headless(builder, output, **kwargs):
    # Older installed model builders lack include_visuals. Remove only massless,
    # non-colliding tool visuals after building; never remove inertial/collision terms.
    if "include_visuals" in inspect.signature(builder).parameters:
        builder(output, include_visuals=False, **kwargs)
        return
    builder(output, **kwargs)
    tree = ET.parse(output)
    root = tree.getroot()
    for parent in root.iter():
        for geom in list(parent.findall("geom")):
            name = geom.get("name", "")
            if name.startswith(("ag145_", "mdpg_")) and name.endswith("_visual"):
                if geom.get("contype", "1") != "0" or geom.get("density") != "0":
                    raise ValueError("Refusing to remove a physical tool geom")
                parent.remove(geom)
    used = {g.get("mesh") for g in root.iter("geom")}
    for asset in root.findall("asset"):
        for mesh in list(asset.findall("mesh")):
            if mesh.get("name", "").startswith(("ag145_", "mdpg_")) and mesh.get("name") not in used:
                asset.remove(mesh)
    tree.write(output, encoding="utf-8", xml_declaration=True)


def motor_model(condition, path):
    if condition not in CONDITIONS:
        raise ValueError("Unknown tool/payload condition")
    from a0509_dynamics_experiment.ag145_model import build_ag145_scene
    from a0509_dynamics_experiment.payload_model import attach_payload, load_payload_spec
    from mdpg_1s1_description.model import build_mdpg_scene
    arm = package_share("dsr_description2") / "mujoco_models/a0509/scene.xml"
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if condition == "bare_p0":
        scene = ET.parse(arm).getroot()
        include = scene.find("include")
        model_path = arm.parent / include.get("file")
        model = ET.parse(model_path).getroot()
        scene.remove(include)
        scene.find("compiler").attrib.pop("meshdir", None)
        for mesh in model.find("asset").findall("mesh"):
            mesh.set("file", (model_path.parent.parent / mesh.get("file")).resolve().as_posix())
        scene.find("asset").extend(list(model.find("asset")))
        scene.find("worldbody").extend(list(model.find("worldbody")))
        scene.append(model.find("actuator"))
        ET.ElementTree(scene).write(path, encoding="utf-8", xml_declaration=True)
    elif condition.startswith("ag145"):
        _headless(build_ag145_scene, path, arm_scene=arm,
                  grasp_angle_rad=.60 if condition in ("ag145_p05", "ag145_p10") else None)
    else:
        _headless(build_mdpg_scene, path, arm_scene=arm, mount_xyz=(0, 0, 0),
                  mount_rpy=(0, 0, 0), dynamics_model="box_approx")
    mass = .5 if condition.endswith("p05") else 1. if condition.endswith("p10") else 0.
    if mass:
        spec = package_share("a0509_dynamics_experiment") / "config/payload_conditions.json"
        attach_payload(path, load_payload_spec(spec, "ag145" if condition.startswith("ag145") else "mdpg_1s1", mass))
    tree = ET.parse(path)
    root = tree.getroot()
    root.remove(root.find("actuator"))
    actuators = ET.SubElement(root, "actuator")
    for j, limit in enumerate(TORQUE_LIMITS, 1):
        ET.SubElement(actuators, "motor", name=f"motor_{j}", joint=f"joint_{j}", gear="1",
                      ctrllimited="true", ctrlrange=f"{-limit} {limit}",
                      forcelimited="true", forcerange=f"{-limit} {limit}")
    option = root.find("option")
    if option is None:
        option = ET.SubElement(root, "option")
    option.set("timestep", ".002")
    option.set("integrator", "implicitfast")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return mj.MjModel.from_xml_path(str(path))


def target_states(trajectory, speed, sample_times):
    if not 1 <= speed <= 2 or not math.isfinite(speed):
        raise ValueError("Speed must be between 1 and 2")
    home, components = DESIGNS[trajectory]
    t = np.asarray(sample_times, dtype=np.float64)
    base_t = np.clip((t - 1.) * speed, 0., DURATION_S)
    u = np.minimum(np.minimum(base_t, DURATION_S-base_t)/RAMP_S, 1.)
    envelope = 10*u**3 - 15*u**4 + 6*u**5
    first = (30*u**2 - 60*u**3 + 30*u**4)/RAMP_S
    second = (60*u - 180*u**2 + 120*u**3)/RAMP_S**2
    first = np.where(base_t > DURATION_S/2, -first, first)
    q = np.tile(home, (len(t), 1)).astype(float)
    dq, ddq = np.zeros_like(q), np.zeros_like(q)
    for joint, amplitude, cycles, phase in components:
        a = math.radians(amplitude * AMPLITUDE_SCALE[joint])
        omega = 2*math.pi*cycles/DURATION_S
        angle = omega*base_t + phase
        sine, cosine = np.sin(angle), np.cos(angle)
        q[:, joint] += a*envelope*sine
        dq[:, joint] += a*(first*sine + envelope*omega*cosine)*speed
        ddq[:, joint] += a*(second*sine + 2*first*omega*cosine - envelope*omega**2*sine)*speed**2
    hold = (t <= 1.) | (t >= 1.+DURATION_S/speed)
    q[hold], dq[hold], ddq[hold] = home, 0., 0.
    return q, dq, ddq
