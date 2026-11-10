"""Optimize T16 against an A0509 MuJoCo parameter-sensitivity proxy.

This is not the paper's rigid-body identification regressor. The project has
an inverse-dynamics evaluator but no linear inertial-parameter regressor. We
differentiate inverse torque w.r.t. each link's mass, local COM z, principal
Izz, and each joint's damping, then normalize columns with a *fixed* baseline
scale before minimizing the condition number of an identifiable column subset.
Measured test torques and learned models are never read.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.linalg import qr
from scipy.optimize import minimize
import yaml

from .ag145_model import build_ag145_scene, source_paths
from .simulation_executor import MujocoObserver
from .t16_fourier import FUNDAMENTAL_HZ, PERIOD_S, coefficients


SEED = 20261111
HOME = np.array((-.25, -.72, 1.25, .18, -.40, .12), dtype=float)
OMEGA = 2 * np.pi * FUNDAMENTAL_HZ * np.arange(1, 4)
TORQUE_MARGIN = 0.90


def states(free: np.ndarray, times: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Vectorized analytic 3-harmonic trajectory, no numerical differentiation."""
    free = np.asarray(free, dtype=float).reshape(6, 4)
    a, b = coefficients(free)
    a, b = np.asarray(a), np.asarray(b)
    sine = np.sin(np.outer(times, OMEGA))
    cosine = np.cos(np.outer(times, OMEGA))
    q = HOME + (sine / OMEGA) @ a.T + ((1 - cosine) / OMEGA) @ b.T
    dq = cosine @ a.T + sine @ b.T
    ddq = (-(sine * OMEGA) @ a.T + (cosine * OMEGA) @ b.T)
    return q, dq, ddq


def _limits(scene: Path) -> dict:
    arm_scene, _ = source_paths()
    joint_yaml = arm_scene.parents[3] / "a0509_mujoco_sim" / "config" / "joint_limits.yaml"
    if not joint_yaml.is_file():
        joint_yaml = Path(__file__).resolve().parents[2] / "a0509_mujoco_sim" / "config" / "joint_limits.yaml"
    limits = yaml.safe_load(joint_yaml.read_text(encoding="utf-8"))["joint_limits"]
    root = ET.parse(scene).getroot()
    joints = [root.find(f".//joint[@name='joint_{i}']") for i in range(1, 7)]
    if any(j is None or "actuatorfrcrange" not in j.attrib for j in joints):
        raise RuntimeError("A0509 MuJoCo joint torque limits are missing")
    urdf = arm_scene.parents[2] / "urdf" / "a0509.urdf"
    urdf_root = ET.parse(urdf).getroot()
    result = {"position": [], "velocity": [], "acceleration": [], "torque": [],
              "sources": {"joint_limits": str(joint_yaml), "torque": str(scene), "urdf": str(urdf)}}
    for i, joint in enumerate(joints, 1):
        cfg = limits[f"joint_{i}"]
        urdf_joint = urdf_root.find(f".//joint[@name='joint_{i}']")
        urdf_limit = urdf_joint.find("limit") if urdf_joint is not None else None
        low, high = float(cfg["min_position"]), float(cfg["max_position"])
        if urdf_limit is not None:
            low = max(low, float(urdf_limit.attrib.get("lower", low)))
            high = min(high, float(urdf_limit.attrib.get("upper", high)))
        force = [float(x) for x in joint.attrib["actuatorfrcrange"].split()]
        if len(force) != 2 or not force[0] < 0 < force[1] or low >= high:
            raise RuntimeError(f"invalid verified limits for joint_{i}")
        result["position"].append((low, high))
        result["velocity"].append(float(cfg["max_velocity"]))
        result["acceleration"].append(float(cfg["max_acceleration"]))
        result["torque"].append(min(-force[0], force[1]))
    return result


class TorqueProxy:
    def __init__(self, scene: Path):
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        self.body = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"link_{i}")
                     for i in range(1, 7)]
        self.joints = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}")
                       for i in range(1, 7)]
        if min(*self.body, *self.joints) < 0 or self.model.nv != 6:
            raise RuntimeError("expected the six-joint A0509 MuJoCo model")
        self.qadr = [int(self.model.jnt_qposadr[j]) for j in self.joints]
        self.dadr = [int(self.model.jnt_dofadr[j]) for j in self.joints]
        self.parameters = []
        for j, body in enumerate(self.body):
            for field, axis, step in (("body_mass", None, None),
                                      ("body_ipos", 2, 0.005),
                                      ("body_inertia", 2, None)):
                array = getattr(self.model, field)
                original = float(array[body] if axis is None else array[body, axis])
                if field != "body_ipos" and original <= 0:
                    raise RuntimeError(f"nonpositive A0509 inertial parameter: J{j+1} {field}")
                delta = step if step is not None else original * 1e-2
                self.parameters.append((f"link_{j+1}.{field}" + (".z" if axis is not None else ""),
                                        field, body, axis, original, delta))
            dof = self.dadr[j]
            original = float(self.model.dof_damping[dof])
            if original <= 0:
                raise RuntimeError("nonpositive joint damping")
            self.parameters.append((f"joint_{j+1}.dof_damping", "dof_damping", dof, None,
                                    original, original * 1e-2))
        self.names = [p[0] for p in self.parameters]

    def _torque(self, q, dq, ddq):
        data = self.data
        data.qpos[:] = self.model.qpos0
        data.qvel[:] = 0
        data.qacc[:] = 0
        data.qpos[self.qadr] = q
        data.qvel[self.dadr] = dq
        data.qacc[self.dadr] = ddq
        mujoco.mj_inverse(self.model, data)
        return data.qfrc_inverse[self.dadr].copy()

    def torque_many(self, q, dq, ddq):
        return np.asarray([self._torque(*state) for state in zip(q, dq, ddq)])

    def _set(self, parameter, value):
        _, field, index, axis, _, _ = parameter
        array = getattr(self.model, field)
        if axis is None:
            array[index] = value
        else:
            array[index, axis] = value

    def matrix(self, q, dq, ddq, torque_limits):
        blocks = []
        row_scale = np.asarray(torque_limits, dtype=float)
        for state in zip(q, dq, ddq):
            columns = []
            for parameter in self.parameters:
                _, _, _, _, original, delta = parameter
                try:
                    self._set(parameter, original + delta)
                    plus = self._torque(*state)
                    self._set(parameter, original - delta)
                    minus = self._torque(*state)
                finally:
                    self._set(parameter, original)
                columns.append((plus - minus) / (2 * delta))
            blocks.append(np.column_stack(columns) / row_scale[:, None])
        return np.vstack(blocks)


def _metrics(matrix, rank_tolerance=1e-4):
    singular = np.linalg.svd(matrix, compute_uv=False)
    rank = int(np.sum(singular > singular[0] * rank_tolerance)) if singular[0] else 0
    condition = float(singular[0] / singular[rank - 1]) if rank else math.inf
    return {"condition_number": condition, "minimum_nonzero_singular_value": float(singular[rank-1]) if rank else 0.,
            "effective_rank": rank, "singular_values": singular.tolist()}


def optimize(output: Path, maxiter: int = 45) -> dict:
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    scene = build_ag145_scene(output / "ag145_scene.xml")
    limits = _limits(scene)
    proxy = TorqueProxy(scene)
    observer = MujocoObserver(scene, tcp_site="ag145_tcp")
    sample_time = np.linspace(0, PERIOD_S, 21)
    dense_time = np.linspace(0, PERIOD_S, 1001)
    torque_limit = np.asarray(limits["torque"])
    pos_low, pos_high = np.asarray(limits["position"]).T
    velocity_limit = np.asarray(limits["velocity"])
    acceleration_limit = np.asarray(limits["acceleration"])
    damping = np.asarray([proxy.model.dof_damping[d] for d in proxy.dadr])
    bounds_per_joint = np.minimum(0.07, 0.45 * torque_limit / (3 * damping))
    variable_bounds = [(-float(b), float(b)) for b in bounds_per_joint for _ in range(4)]
    rng = np.random.default_rng(SEED)
    baseline = rng.uniform(-.45, .45, 24) * np.repeat(bounds_per_joint, 4)

    def feasible_margin(x):
        q, dq, ddq = states(x, sample_time)
        torque = np.abs(proxy.torque_many(q, dq, ddq))
        return np.concatenate(((q.min(axis=0) - pos_low - .03),
                               (pos_high - q.max(axis=0) - .03),
                               .8 * velocity_limit - np.max(np.abs(dq), axis=0),
                               .8 * acceleration_limit - np.max(np.abs(ddq), axis=0),
                               TORQUE_MARGIN * torque_limit - np.max(torque, axis=0)))

    for _ in range(100):
        if np.min(feasible_margin(baseline)) >= 0:
            break
        baseline *= .8
    else:
        raise RuntimeError("could not form a feasible seeded Fourier baseline")

    q0, dq0, ddq0 = states(baseline, sample_time)
    raw_baseline = proxy.matrix(q0, dq0, ddq0, torque_limit)
    column_scale = np.maximum(np.sqrt(np.mean(raw_baseline**2, axis=0)), 1e-9)
    baseline_matrix = raw_baseline / column_scale
    _, r, pivot = qr(baseline_matrix, pivoting=True, mode="economic")
    rank = min(int(np.sum(np.abs(np.diag(r)) > abs(r[0, 0]) * 1e-4)),
               _metrics(baseline_matrix)["effective_rank"])
    while rank >= 6:
        selected = np.sort(pivot[:rank])
        baseline_metrics = _metrics(baseline_matrix[:, selected])
        if baseline_metrics["effective_rank"] == rank:
            break
        rank -= 1
    if rank < 6:
        raise RuntimeError(f"parameter-sensitivity proxy rank too low ({rank})")
    cache = {}

    def score(x):
        key = tuple(np.asarray(x).round(12))
        if key not in cache:
            q, dq, ddq = states(x, sample_time)
            matrix = proxy.matrix(q, dq, ddq, torque_limit)[:, selected] / column_scale[selected]
            result = _metrics(matrix)
            condition = result["condition_number"]
            cache[key] = (math.log(condition) if result["effective_rank"] == rank and
                          math.isfinite(condition) else 1e6)
        return cache[key]

    initial_score = score(baseline)
    lower = np.asarray([item[0] for item in variable_bounds])
    upper = np.asarray([item[1] for item in variable_bounds])
    def constrained(x):
        return np.concatenate((feasible_margin(x), x - lower, upper - x))
    result = minimize(score, baseline, method="COBYLA",
                      constraints={"type": "ineq", "fun": constrained},
                      options={"maxiter": maxiter, "rhobeg": float(np.min(bounds_per_joint) * .5),
                               "tol": 1e-5, "catol": 1e-5, "disp": False})
    candidate = np.asarray(result.x)
    candidate_score = score(candidate) if np.min(feasible_margin(candidate)) >= -1e-5 else math.inf
    if candidate_score >= initial_score - 1e-6:
        raise RuntimeError(f"T16 optimization did not improve baseline: {initial_score:.5g} vs {candidate_score:.5g}; "
                           f"SciPy: {result.message}; nfev={result.nfev}; min_margin={np.min(feasible_margin(candidate)):.4g}")

    q, dq, ddq = states(candidate, dense_time)
    if (np.any(q < pos_low + .03) or np.any(q > pos_high - .03) or
            np.any(np.abs(dq) > .8 * velocity_limit) or
            np.any(np.abs(ddq) > .8 * acceleration_limit)):
        raise RuntimeError("dense Fourier kinematic validation failed")
    dense_torque = proxy.torque_many(q, dq, ddq)
    if np.any(np.abs(dense_torque) > TORQUE_MARGIN * torque_limit):
        raise RuntimeError("dense inverse-dynamics torque validation failed")
    for row in q:
        observer.require_contact_free(row)
    if (not np.allclose(q[[0, -1]], HOME, atol=1e-10) or
            not np.allclose(dq[[0, -1]], 0, atol=1e-10) or
            not np.allclose(ddq[[0, -1]], 0, atol=1e-10)):
        raise RuntimeError("Fourier endpoint C2 constraints failed")
    optimized_matrix = proxy.matrix(*states(candidate, sample_time), torque_limit)[:, selected] / column_scale[selected]
    optimized_metrics = _metrics(optimized_matrix)
    if optimized_metrics["effective_rank"] < rank or optimized_metrics["condition_number"] >= baseline_metrics["condition_number"]:
        raise RuntimeError(f"condition number or effective rank did not improve: "
                           f"baseline={baseline_metrics['condition_number']:.6g}, "
                           f"optimized={optimized_metrics['condition_number']:.6g}, "
                           f"rank={rank}/{optimized_metrics['effective_rank']}, "
                           f"score={initial_score:.6g}/{candidate_score:.6g}")
    a, b = coefficients(candidate.reshape(6, 4))
    report = {
        "status": "validated_simulation_only", "name": "Optimized Fourier Excitation",
        "objective_kind": "parameter-sensitivity proxy", "not_paper_regressor": True,
        "period_s": PERIOD_S, "fundamental_hz": FUNDAMENTAL_HZ, "harmonics": 3,
        "sample_hz": 100, "random_seed": SEED, "home_rad": HOME.tolist(),
        "free_coefficients_rad_s": candidate.reshape(6, 4).tolist(),
        "a_rad_s": a, "b_rad_s": b,
        "proxy_parameter_names": proxy.names, "selected_parameter_names": [proxy.names[i] for i in selected],
        "column_scale_fixed_from_baseline": column_scale.tolist(),
        "svd_relative_rank_tolerance": 1e-4, "sample_count_in_objective": len(sample_time),
        "initial_free_coefficients_rad_s": baseline.reshape(6, 4).tolist(),
        "initial": baseline_metrics, "optimized": optimized_metrics,
        "optimizer": {"method": "SciPy COBYLA", "success": bool(result.success),
                      "message": str(result.message), "iterations": int(getattr(result, "nit", 0)),
                      "function_evaluations": int(result.nfev)},
        "safety": {"validated_dense_samples": len(dense_time), "contact_free_samples": len(dense_time),
                   "maximum_abs_torque_Nm": np.max(np.abs(dense_torque), axis=0).tolist(),
                   "maximum_abs_dq_rad_s": np.max(np.abs(dq), axis=0).tolist(),
                   "maximum_abs_ddq_rad_s2": np.max(np.abs(ddq), axis=0).tolist(),
                   "minimum_q_rad": q.min(axis=0).tolist(), "maximum_q_rad": q.max(axis=0).tolist(),
                   "limits": limits, "torque_limit_fraction": TORQUE_MARGIN,
                   "valid_for_real_robot": False},
        "scene_path_at_optimization": str(scene),
        "limitations": ["AG145 inertial/collision model is an existing approximation, not calibrated hardware.",
                        "No independent collision checker for a physical workcell is available.",
                        "The proxy is not a full minimal RBD identification regressor."],
    }
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--work-dir", type=Path, default=Path("data/t16_fourier_design"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maxiter", type=int, default=45)
    args = parser.parse_args(argv)
    report = optimize(args.work_dir, args.maxiter)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "initial_condition": report["initial"]["condition_number"],
                      "optimized_condition": report["optimized"]["condition_number"],
                      "effective_rank": report["optimized"]["effective_rank"],
                      "optimizer": report["optimizer"]}, indent=2))


if __name__ == "__main__":
    main()
