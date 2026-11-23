"""One backend per friction law: no native/external double application."""
from __future__ import annotations

import math
import mujoco as mj
import numpy as np


def scenarios(config, baseline=True):
    result = [{"name": "baseline", "model": "baseline", "level": "none", "scale": 0.}] if baseline else []
    for family in config["legacy"]["models"]:
        if family == "lugre" and not config["include_lugre_after_reproduction"]:
            continue
        for level, scale in config["legacy"]["levels"].items():
            result.append({"name": f"{family}_{level}", "model": family, "level": level, "scale": scale})
    return result


def parameters(scenario, config):
    old, scale, family = config["legacy"], scenario["scale"], scenario["model"]
    fc = np.asarray(old["friction_base_nm"])*scale
    b = np.asarray(old["viscous_base_nm_s_rad"])*scale
    background = np.asarray(config["background_damping_nm_s_rad"], dtype=float)
    return {**scenario, "fc_nm": fc.tolist(), "viscous_nm_s_rad": b.tolist(),
            "native_damping_nm_s_rad": (background + (b if family == "viscous" else 0)).tolist(),
            "native_frictionloss_nm": (fc if family == "coulomb" else np.zeros(6)).tolist(),
            "external_backend": family in ("stribeck", "asymmetric", "lugre"),
            "stribeck": old["stribeck"], "asymmetric": old["asymmetric"], "lugre": old["lugre"],
            "lugre_initial_z_rad": [0.]*6, "strength_scaling": "scale Fc and B; sigma0 and sigma1 unchanged"}


def configure_model(model, scenario, config):
    p = parameters(scenario, config)
    model.dof_damping[:] = p["native_damping_nm_s_rad"]
    model.dof_frictionloss[:] = p["native_frictionloss_nm"]
    model.opt.jacobian = mj.mjtJacobian.mjJAC_DENSE
    return p


def resistance(velocity, scenario, config):
    """Positive required compensation torque; physical applied force is negative."""
    p, old = parameters(scenario, config), config["legacy"]
    v, fc, b = np.asarray(velocity), np.asarray(p["fc_nm"]), np.asarray(p["viscous_nm_s_rad"])
    family = scenario["model"]
    if family == "viscous":
        return b*v
    if family == "stribeck":
        settings = old["stribeck"]
        return (fc*(1+(settings["static_to_coulomb_ratio"]-1)*np.exp(-(np.abs(v)/settings["transition_rad_s"])**2))
                * np.tanh(v/settings["smoothing_rad_s"]) + b*v)
    if family == "asymmetric":
        settings = old["asymmetric"]
        strength = fc*np.where(v >= 0, settings["positive_ratio"], settings["negative_ratio"])
        return strength*np.tanh(v/settings["smoothing_rad_s"]) + .5*b*v
    if family == "baseline":
        return np.zeros_like(v)
    raise ValueError("Coulomb uses native constraints; LuGre needs a complete causal history")


class FrictionState:
    def __init__(self, scenario, config):
        self.scenario, self.config = scenario, config
        self.p = parameters(scenario, config)
        self.fc, self.b = np.asarray(self.p["fc_nm"]), np.asarray(self.p["viscous_nm_s_rad"])
        self.z = np.zeros(6)
        self.z_dot = np.zeros(6)
        self.last_time = None

    def advance(self, time_s, velocity):
        v = np.asarray(velocity, dtype=float)
        family = self.scenario["model"]
        if self.last_time is not None and time_s <= self.last_time:
            raise ValueError("Friction history must advance exactly once per increasing physics time")
        dt = 0. if self.last_time is None else time_s-self.last_time
        self.last_time = time_s
        if family in ("baseline", "coulomb", "viscous"):
            return np.zeros(6)  # Native model owns these terms.
        if family != "lugre":
            return -resistance(v, self.scenario, self.config)
        settings = self.config["legacy"]["lugre"]
        sigma0, sigma1 = settings["stiffness_nm_rad"], settings["microdamping_nm_s_rad"]
        fs = settings["static_to_coulomb_ratio"]*self.fc
        g = (self.fc+(fs-self.fc)*np.exp(-(np.abs(v)/settings["transition_rad_s"])**2))/sigma0
        rate = np.abs(v)/g
        decay = np.exp(-rate*dt)
        next_z = self.z*decay + np.where(rate > 1e-12,
                    v*(-np.expm1(-rate*dt))/np.maximum(rate, 1e-12), v*dt)
        self.z_dot = (next_z-self.z)/dt if dt else np.zeros(6)
        self.z = next_z
        return -(sigma0*self.z + sigma1*self.z_dot + settings["viscous_ratio"]*self.b*v)


def dry_force(data, nv=6):
    if not data.nefc:
        return np.zeros(nv)
    mask = np.asarray(data.efc_type) == int(mj.mjtConstraint.mjCNSTR_FRICTION_DOF)
    if not np.any(mask):
        return np.zeros(nv)
    jacobian = np.asarray(data.efc_J).reshape(data.nefc, nv)
    return jacobian[mask].T @ np.asarray(data.efc_force)[mask]


def decompose(model, data, background):
    mass = np.empty((6, 6))
    mj.mj_fullM(model, mass, data.qM)
    rigid = mass@data.qacc + data.qfrc_bias
    native_dry = dry_force(data)
    added_damping = -(model.dof_damping-np.asarray(background))*data.qvel
    added = added_damping + native_dry + data.qfrc_applied
    balance = rigid-(data.qfrc_actuator+data.qfrc_passive+data.qfrc_constraint+data.qfrc_applied)
    return {"tau_rbd_nm": rigid, "qfrc_passive": data.qfrc_passive.copy(),
            "qfrc_constraint": data.qfrc_constraint.copy(), "qfrc_native_dry": native_dry,
            "qfrc_additional_friction": added, "qfrc_external_friction": data.qfrc_applied.copy(),
            "qfrc_background_damping": -np.asarray(background)*data.qvel,
            "qfrc_additional_damping": added_damping, "force_balance_residual_nm": balance}


def inverse_at(model, data, q, dq, ddq, external=None, background=None):
    data.qpos[:] = q
    data.qvel[:] = dq
    data.qfrc_applied[:] = 0. if external is None else external
    mj.mj_forward(model, data)
    if data.ncon:
        raise ValueError("Contact in an offline comparison state")
    data.qacc[:] = ddq
    mj.mj_inverse(model, data)
    parts = decompose(model, data, np.zeros(6) if background is None else background)
    return data.qfrc_inverse.copy(), parts
