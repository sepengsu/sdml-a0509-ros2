"""MuJoCo rigid-body inverse dynamics for nominal and reference tool models.

This module is offline. `qfrc_inverse` is a simulator generalized force, not
a physical torque sensor. MuJoCo 3.2.7 includes joint damping in mj_inverse;
`qfrc_bias` excludes it, so damping is added exactly once below.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .data import REPO_ROOT


ARM_SCENE = REPO_ROOT / "src/doosan-robot2/dsr_description2/mujoco_models/a0509/scene.xml"
AG145_SCENE = REPO_ROOT / "results/ag145/simulation/T1/trial_005/ag145_scene.xml"
AG145_BASE_INERTIA = (0.0073, 0.0050, 0.0033)  # existing approximation, kg m^2
AG145_BASE_MASS = 1.3  # mass already used in the repository scene, kg


@dataclass(frozen=True)
class ToolSpec:
    name: str
    mass_kg: float
    com_local_m: tuple[float, float, float]
    diagonal_inertia_local_kg_m2: tuple[float, float, float]
    role: str

    def scaled_nominal(self) -> "ToolSpec":
        return self.nominal_at(0.8)

    def nominal_at(self, factor: float) -> "ToolSpec":
        if self.mass_kg == 0:
            return self
        if not 0.25 <= factor <= 1.0:
            raise ValueError("nominal mass factor must be between 0.25 and 1")
        mismatch = (1.0 - factor) / 0.2
        inertia_factor = 1.25 * factor - 0.25
        return ToolSpec(
            self.name + "_nominal", self.mass_kg * factor,
            (self.com_local_m[0] + 0.005 * mismatch,
             self.com_local_m[1] - 0.005 * mismatch,
             self.com_local_m[2] - 0.01 * mismatch),
            tuple(value * inertia_factor for value in self.diagonal_inertia_local_kg_m2),
            "synthetic_uncertain_estimate",
        )

    def metadata(self) -> dict:
        return asdict(self)


def tool_specs() -> dict[str, ToolSpec]:
    """Explicit virtual-study parameters; only AG145 is in an existing scene."""
    def virtual(name: str, mass: float, com, role: str) -> ToolSpec:
        return ToolSpec(name, mass, tuple(com),
                        tuple(value * mass / AG145_BASE_MASS for value in AG145_BASE_INERTIA), role)

    return {
        "bare": ToolSpec("bare", 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), "no_tool"),
        "ag145": ToolSpec("ag145", AG145_BASE_MASS, (0.0, 0.0, 0.10),
                           AG145_BASE_INERTIA, "existing_approximate_AG145_scene"),
        "virtual_light": virtual("virtual_light", 0.9, (0.010, 0.0, 0.08), "synthetic_train"),
        "virtual_heavy": virtual("virtual_heavy", 1.7, (-0.010, 0.015, 0.12), "synthetic_train"),
        "virtual_unseen": virtual("virtual_unseen", 1.5, (0.015, -0.010, 0.11), "synthetic_test_only"),
    }


def _ag145_xml(spec: ToolSpec) -> str:
    scene_text = AG145_SCENE.read_text(encoding="utf-8")
    scene_text = scene_text.replace("/ros2_ws/", REPO_ROOT.as_posix().rstrip("/") + "/")
    root = ET.fromstring(scene_text)
    tool = root.find(".//body[@name='ag145_base_link']")
    if tool is None:
        raise ValueError("AG145 scene lacks its fixed tool body")
    inertial = tool.find("inertial")
    if inertial is None:
        raise ValueError("AG145 scene lacks its existing approximate inertial")
    inertial.set("mass", f"{spec.mass_kg:.12g}")
    inertial.set("pos", " ".join(f"{value:.12g}" for value in spec.com_local_m))
    inertial.set("diaginertia", " ".join(f"{value:.12g}" for value in spec.diagonal_inertia_local_kg_m2))
    return ET.tostring(root, encoding="unicode")


def make_model(spec: ToolSpec) -> mujoco.MjModel:
    if spec.mass_kg == 0:
        if any(spec.com_local_m) or any(spec.diagonal_inertia_local_kg_m2):
            raise ValueError("zero-mass tool cannot have CoM or inertia")
        return mujoco.MjModel.from_xml_path(str(ARM_SCENE))
    if spec.mass_kg <= 0 or not all(value > 0 for value in spec.diagonal_inertia_local_kg_m2):
        raise ValueError("tool mass and principal inertias must be positive")
    return mujoco.MjModel.from_xml_string(_ag145_xml(spec))


def _quat_matrix(quaternion: np.ndarray) -> np.ndarray:
    matrix = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(matrix, quaternion.astype(np.float64))
    return matrix.reshape(3, 3)


def flange_parameters(spec: ToolSpec) -> np.ndarray:
    """[m, COM xyz, Ixx,Iyy,Izz,Ixy,Ixz,Iyz] in link_6 coordinates."""
    if spec.mass_kg == 0:
        return np.zeros(10, dtype=np.float64)
    # The existing AG145 scene mounts the tool at a 45-degree Z rotation.
    root = ET.fromstring(_ag145_xml(spec))
    body = root.find(".//body[@name='ag145_base_link']")
    rotation = _quat_matrix(np.asarray([float(v) for v in body.attrib["quat"].split()]))
    center = rotation @ np.asarray(spec.com_local_m)
    inertia = rotation @ np.diag(spec.diagonal_inertia_local_kg_m2) @ rotation.T
    return np.asarray([spec.mass_kg, *center, inertia[0, 0], inertia[1, 1],
                       inertia[2, 2], inertia[0, 1], inertia[0, 2], inertia[1, 2]])


class RBDModel:
    def __init__(self, model: mujoco.MjModel):
        self.model = model
        self.data = mujoco.MjData(model)
        self.qpos_addresses = []
        self.dof_addresses = []
        for index in range(1, 7):
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{index}")
            if joint < 0:
                raise ValueError(f"joint_{index} missing from MuJoCo model")
            self.qpos_addresses.append(int(model.jnt_qposadr[joint]))
            self.dof_addresses.append(int(model.jnt_dofadr[joint]))
        if model.nq != 6 or model.nv != 6:
            raise ValueError("expected exactly six A0509 degrees of freedom")
        if np.any(model.dof_frictionloss[self.dof_addresses]):
            raise ValueError("Coulomb friction requires a separate torque definition")

    def evaluate(self, q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> dict[str, np.ndarray]:
        q, dq, ddq = (np.asarray(values, dtype=np.float64) for values in (q, dq, ddq))
        if any(values.shape != (6,) or not np.isfinite(values).all() for values in (q, dq, ddq)):
            raise ValueError("q, dq and ddq must each contain six finite values")
        data = self.data
        data.qpos[:] = self.model.qpos0
        data.qvel[:] = 0
        data.qacc[:] = 0
        data.qpos[self.qpos_addresses] = q
        mujoco.mj_forward(self.model, data)
        gravity = data.qfrc_bias[self.dof_addresses].copy()
        data.qvel[self.dof_addresses] = dq
        mujoco.mj_forward(self.model, data)
        mass = np.empty((6, 6), dtype=np.float64)
        mujoco.mj_fullM(self.model, mass, data.qM)
        bias = data.qfrc_bias[self.dof_addresses].copy()
        damping = self.model.dof_damping[self.dof_addresses] * dq
        reconstructed = mass @ ddq + bias + damping
        data.qacc[self.dof_addresses] = ddq
        mujoco.mj_inverse(self.model, data)
        inverse = data.qfrc_inverse[self.dof_addresses].copy()
        if not np.allclose(reconstructed, inverse, atol=2e-5, rtol=1e-6):
            raise RuntimeError("MuJoCo inverse dynamics differs from M ddq + bias + D dq")
        return {"mass_matrix": mass, "bias": bias, "gravity": gravity,
                "damping": damping, "torque": inverse,
                "velocity_bias": bias - gravity}

    def evaluate_many(self, q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> dict[str, np.ndarray]:
        if q.shape != dq.shape or q.shape != ddq.shape or q.ndim != 2 or q.shape[1] != 6:
            raise ValueError("expected matching [samples, 6] arrays")
        result: dict[str, list[np.ndarray]] = {}
        for state in zip(q, dq, ddq):
            for key, value in self.evaluate(*state).items():
                result.setdefault(key, []).append(value)
        return {key: np.stack(values) for key, values in result.items()}
