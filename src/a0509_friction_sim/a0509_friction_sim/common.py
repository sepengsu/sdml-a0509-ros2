from __future__ import annotations

import ast
from functools import lru_cache
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from .runtime import sha256, write_json, run_lock, available_ram, run_worker
from .models import CONDITIONS, SPLITS, motor_model, target_states, asset_files
from a0509_dynamics_experiment.wide16_trajectory import generate_wide

PACKAGE = Path(__file__).resolve().parent


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def load_config(path):
    config = json.loads(Path(path).read_text(encoding="utf-8"))
    if config["schema_version"] != 4 or config["background_damping_policy"] != "preserve_original":
        raise ValueError("Only the preserved-background-damping v4 simulation profile is supported")
    if mj.__version__ != "3.2.7" or np.__version__ != "1.26.4":
        raise ValueError("Validated runtime requires mujoco==3.2.7 and numpy==1.26.4; install a0509_friction_sim/requirements.txt")
    if digest(config["legacy"]) != config["original_config_sha256"]:
        raise ValueError("Legacy coefficients/formulas changed; create and validate another profile")
    if sha256(PACKAGE / "reference_original.py") != config["original_generator_sha256"]:
        raise ValueError("Original generator snapshot SHA-256 differs")
    sim = config["simulation"]
    if sim["integrator"] != "implicitfast":
        raise ValueError("This validated profile requires the implicitfast integrator")
    ratio = 1./sim["sample_hz"]/sim["timestep_s"]
    if not np.isclose(ratio, round(ratio)) or ratio < 1 or not 1 <= sim["speed_scale"] <= 2:
        raise ValueError("Sample period must contain an integer number of physical steps")
    return config


@lru_cache(maxsize=1)
def original_functions():
    source = PACKAGE / "reference_original.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    names = {"_perturb_plan", "_lugre_friction", "_smooth_friction", "_asymmetric_friction"}
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in names]
    if len(functions) != len(names):
        raise ValueError("Original reference functions missing")
    namespace = {"np": np}
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source), "exec"), namespace)
    return namespace


@lru_cache(maxsize=16)
def legacy_base_plan(trajectory):
    plan = generate_wide(trajectory)
    points = plan.points[::10]
    return tuple(np.asarray(values) for values in (
        [p.time_s for p in points], [p.q_rad for p in points],
        [p.dq_rad_s for p in points], [p.ddq_rad_s2 for p in points]))


def reconstruct_legacy(entry, config, limits):
    result = original_functions()["_perturb_plan"](
        legacy_base_plan(entry["trajectory"]), entry["trajectory"], entry["trial"], config["legacy"], limits)
    t, q, dq, ddq, seed, factor = result
    if seed != entry["seed"] or factor != entry["amplitude_factor"]:
        raise ValueError("Original perturbation seed/amplitude could not be reproduced")
    trim = config["legacy"]["trim_seconds_each_end"]
    selected = np.flatnonzero((t >= trim) & (t <= t[-1]-trim))
    return (t, q, dq, ddq), selected


def build_baseline(condition, destination, config):
    model = motor_model(condition, destination)
    if (not np.array_equal(model.dof_damping, config["background_damping_nm_s_rad"])
            or np.any(model.dof_frictionloss)):
        raise ValueError("Source damping/frictionloss differs from the audited baseline")
    effort = np.asarray(config["effort_limits_nm"])
    if (not np.array_equal(model.actuator_ctrlrange[:, 1], effort)
            or not np.array_equal(model.actuator_forcerange[:, 1], effort)
            or not np.array_equal(model.actuator_ctrlrange[:, 0], -effort)
            or not np.array_equal(model.actuator_forcerange[:, 0], -effort)):
        raise ValueError("Configured torque limits differ from the unchanged source motors")
    model.opt.timestep = config["simulation"]["timestep_s"]
    model.opt.jacobian = mj.mjtJacobian.mjJAC_DENSE
    tree = ET.parse(destination)
    option = tree.getroot().find("option")
    option.set("timestep", str(model.opt.timestep))
    option.set("jacobian", "dense")
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    return model


def save_npz(path, arrays):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".partial")
    with temporary.open("wb") as stream:
        np.savez_compressed(stream, **arrays)
    with np.load(temporary, allow_pickle=False) as check:
        for name, values in arrays.items():
            if check[name].shape != np.shape(values) or not np.isfinite(check[name]).all():
                raise RuntimeError(f"Invalid saved array {name}")
    temporary.replace(path)


def identity(config, source):
    files = {f"a0509_friction_sim/{p.name}": p for p in PACKAGE.glob("*.py")}
    files["a0509_friction_sim/requirements.txt"] = PACKAGE / "requirements.txt"
    files.update(asset_files())
    record = {"config": config, "source_manifest_sha256": sha256(source/"manifest.json"),
              "assets": {name: sha256(p) for name, p in files.items()},
              "mujoco": mj.__version__, "numpy": np.__version__}
    return digest(record), record


def checked_source(source, entry):
    path = (source/entry["relative_npz"]).resolve()
    if not path.is_relative_to(source.resolve()) or sha256(path) != entry["npz_sha256"]:
        raise ValueError("Original source chunk path or checksum differs")
    return path


def marker_complete(folder, run_identity):
    try:
        value = json.loads((folder/"metadata.json").read_text(encoding="utf-8"))
        if value["identity"] != run_identity or value["status"] != "complete":
            return None
        for name, checksum in value["files"].items():
            if sha256(folder/name) != checksum:
                return None
        return value
    except (OSError, ValueError, KeyError, TypeError):
        return None
