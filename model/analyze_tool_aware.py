"""Compare tool-dependent MuJoCo inverse dynamics at identical recorded states.

This is an offline counterfactual: it does not imply that a different tool
would track the original Cartesian command or be safe on a physical robot.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
TRIALS = {
    "T1": "results/ag145/simulation/T1/trial_005",
    "T2": "results/ag145/simulation/T2/trial_001",
    "T3": "results/ag145/simulation/T3/trial_001",
    "T4": "results/ag145/simulation/T4/trial_001",
    "T5": "results/dynamics10/simulation/T5/trial_003",
    **{f"T{i}": f"results/dynamics10/simulation/T{i}/trial_001" for i in range(6, 15)},
}
ACTUAL_T1 = {
    "none": "results/simulation/T1/trial_004",
    "ag145": TRIALS["T1"],
    "mdpg_1s1": "results/mdpg_box_approx/simulation/T1/trial_001",
}
SCENES = {
    "none": "src/doosan-robot2/dsr_description2/mujoco_models/a0509/scene.xml",
    "ag145": "results/ag145/simulation/T1/trial_005/ag145_scene.xml",
    "mdpg_1s1": "results/mdpg_box_approx/simulation/T1/trial_001/mdpg_scene.xml",
}
JOINTS = [f"J{i}" for i in range(1, 7)]


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_trial(path: Path):
    meta = json.loads((path / "metadata.json").read_text(encoding="utf-8"))
    with (path / "dynamics.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    return meta, rows


def _six(row: dict, prefix: str):
    return np.asarray([float(row[f"{prefix}{i}"]) for i in range(1, 7)], dtype=float)


def _rms(values, axis=None):
    return np.sqrt(np.mean(np.square(values), axis=axis))


def _portable_scene(source: Path, root: Path, output: Path) -> Path:
    """Map Docker's /ros2_ws asset paths to this checkout, without changing source XML."""
    if source.name == "scene.xml":
        return source
    tree = ET.parse(source)
    for element in tree.getroot().iter():
        asset = element.get("file")
        if asset and asset.startswith("/ros2_ws/"):
            installed = "/ros2_ws/install/mdpg_1s1_description/share/mdpg_1s1_description/"
            mapped = (root / "src/mdpg_1s1_description" / asset.removeprefix(installed)
                      if asset.startswith(installed) else root / asset.removeprefix("/ros2_ws/"))
            if not mapped.is_file():
                raise FileNotFoundError(mapped)
            element.set("file", mapped.as_posix())
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output


def _state(model, data, q, dq, ddq):
    data.qpos[:] = q
    data.qvel[:] = dq
    mujoco.mj_forward(model, data)
    mass = np.empty((6, 6), dtype=float)
    mujoco.mj_fullM(model, mass, data.qM)
    bias = data.qfrc_bias.copy()
    data.qacc[:] = ddq
    mujoco.mj_inverse(model, data)
    return data.qfrc_inverse.copy(), bias, mass


def analyze(root: Path = ROOT, output: Path | None = None, stride: int = 1,
            warmup_seconds: float = .5):
    root = Path(root).resolve()
    output = Path(output or root / "model/tool_aware_analysis").resolve()
    if stride < 1:
        raise ValueError("stride must be >= 1")
    if warmup_seconds < 0:
        raise ValueError("warmup_seconds must be nonnegative")
    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="a0509_tool_aware_") as temporary:
        models = {name: mujoco.MjModel.from_xml_path(str(
                      _portable_scene(root / scene, root, Path(temporary) / f"{name}.xml")))
                  for name, scene in SCENES.items()}
    if any((model.nq, model.nv) != (6, 6) for model in models.values()):
        raise ValueError("all scenes must contain the same six A0509 DoFs")
    masses = {name: float(np.sum(model.body_mass)) for name, model in models.items()}
    if not np.isclose(masses["mdpg_1s1"] - masses["none"], .62, atol=1e-6):
        raise ValueError("mDPG scene does not contain expected 0.62 kg estimate")
    if not np.isclose(masses["ag145"] - masses["none"], 1.3, atol=1e-6):
        raise ValueError("AG145 scene does not contain expected 1.3 kg estimate")
    mdpg_meta = json.loads((root / ACTUAL_T1["mdpg_1s1"] / "metadata.json").read_text())
    if (mdpg_meta.get("gripper_model", {}).get("dynamics_model") != "box_approx"
            or mdpg_meta.get("gripper_model", {}).get("mount_verified") is not False):
        raise ValueError("mDPG model provenance differs from unverified box_approx fixture")

    actual = []
    for name, relative in ACTUAL_T1.items():
        path = root / relative
        meta, rows = _load_trial(path)
        if meta.get("gripper_type") != name or meta.get("mode") != "simulation":
            raise ValueError(f"{relative}: gripper metadata mismatch")
        valid = []
        duplicate_stamps = 0
        for row in rows:
            try:
                q, target = _six(row, "q"), _six(row, "target_q")
                if np.isfinite(q).all() and np.isfinite(target).all():
                    valid.append(q - target)
            except (ValueError, KeyError):
                pass
        if not valid:
            raise ValueError(f"{relative}: no tracking data")
        err = np.stack(valid)
        logged_accel = []
        inverse_replay_errors = []
        replay_data = mujoco.MjData(models[name])
        for index, row in enumerate(rows):
            try:
                value = _six(row, "ddq")
                if np.isfinite(value).all():
                    logged_accel.append(value)
                    if index % stride == 0 and row.get("qfrc_inverse"):
                        state = (_six(row, "q"), _six(row, "dq"), value)
                        recorded = np.asarray(json.loads(row["qfrc_inverse"]), dtype=float)
                        if all(np.isfinite(item).all() for item in (*state, recorded)):
                            replayed, _, _ = _state(models[name], replay_data, *state)
                            inverse_replay_errors.append(replayed - recorded)
            except (ValueError, KeyError):
                pass
        actual.append({
            "tool": name, "trial": relative, "rows": len(rows), "valid_tracking_rows": len(valid),
            "joint_tracking_rmse_rad": _rms(err, axis=0).tolist(),
            "joint_tracking_max_abs_rad": np.max(np.abs(err), axis=0).tolist(),
            "tracking_rmse_all_rad": float(_rms(err)),
            "logged_ddq_rms_rad_s2": float(_rms(np.stack(logged_accel))) if logged_accel else None,
            "inverse_replay_rmse_nm": float(_rms(np.stack(inverse_replay_errors))) if inverse_replay_errors else None,
            "observed_joint_state_hz": meta.get("quality", {}).get("joint_states", {}).get("observed_hz"),
            "estimated_missing_rate": meta.get("quality", {}).get("joint_states", {}).get("estimated_missing_rate"),
            "reference_tcp_mm": meta.get("trajectory_config", {}).get("reference_tcp_mm"),
            "dynamics_sha256": _sha256(path / "dynamics.csv"),
        })

    comparison = []
    per_joint = []
    profiles = {}
    dynamic_profiles = {}
    for trajectory, relative in TRIALS.items():
        path = root / relative
        meta, rows = _load_trial(path)
        if (meta.get("gripper_type") != "ag145" or meta.get("trajectory_id") != trajectory
                or meta.get("mode") != "simulation" or meta.get("execution_success", True) is False):
            raise ValueError(f"{relative}: trial provenance mismatch")
        valid = []
        for row in rows:
            try:
                state = (_six(row, "q"), _six(row, "dq"), _six(row, "ddq"))
                logged = np.asarray(json.loads(row["qfrc_inverse"]), dtype=float)
                t = float(row["ros_time_s"])
                if all(np.isfinite(v).all() for v in (*state, logged)) and np.isfinite(t):
                    if valid and t == valid[-1][0]:
                        valid[-1] = (t, state, logged)
                        duplicate_stamps += 1
                    elif valid and t < valid[-1][0]:
                        raise RuntimeError(f"{relative}: simulation time moved backward")
                    else:
                        valid.append((t, state, logged))
            except (ValueError, KeyError, TypeError, json.JSONDecodeError):
                pass
        if len(valid) < 5:
            raise ValueError(f"{relative}: no complete states")
        times = np.asarray([row[0] for row in valid])
        velocities = np.stack([row[1][1] for row in valid])
        if np.any(np.diff(times) <= 0):
            raise ValueError(f"{relative}: simulation time is not strictly increasing")
        # All cohorts use the same centered five-sample velocity slope. Older
        # T1-T4 CSVs used a different acceleration filter from T5-T14.
        uniform_accel = np.empty_like(velocities)
        for index in range(len(valid)):
            lo, hi = max(0, index - 2), min(len(valid), index + 3)
            centered = times[lo:hi] - np.mean(times[lo:hi])
            uniform_accel[index] = np.sum(centered[:, None] * velocities[lo:hi], axis=0) / np.sum(centered**2)
        eligible = [i for i in range(len(valid)) if times[i] - times[0] >= warmup_seconds]
        sampled = [(valid[i][0], (valid[i][1][0], valid[i][1][1], uniform_accel[i]),
                    valid[i][2], valid[i][1][2]) for i in eligible[::stride]]
        if not sampled:
            raise ValueError(f"{relative}: no states after warmup")
        logged_accel = np.stack([row[1][2] for row in valid])
        acceleration_stats = {
            "logged_ddq_rms_rad_s2": float(_rms(logged_accel)),
            "logged_ddq_abs_max_rad_s2": float(np.max(np.abs(logged_accel))),
            "standardized_ddq_rms_rad_s2": float(_rms(uniform_accel)),
            "standardized_ddq_abs_max_rad_s2": float(np.max(np.abs(uniform_accel))),
        }
        data = {name: mujoco.MjData(model) for name, model in models.items()}
        torque = {name: [] for name in models}
        static = {name: [] for name in models}
        bias = {name: [] for name in models}
        matrix = {name: [] for name in models}
        replay_check = []
        for _, state, logged, original_accel in sampled:
            replay_tau, _, _ = _state(models["ag145"], data["ag145"], state[0], state[1], original_accel)
            replay_check.append(replay_tau - logged)
            for name, model in models.items():
                tau, b, m = _state(model, data[name], *state)
                torque[name].append(tau)
                bias[name].append(b)
                matrix[name].append(m)
                static_tau, _, _ = _state(model, data[name], state[0], np.zeros(6), np.zeros(6))
                static[name].append(static_tau)
        torque = {name: np.stack(values) for name, values in torque.items()}
        static = {name: np.stack(values) for name, values in static.items()}
        bias = {name: np.stack(values) for name, values in bias.items()}
        matrix = {name: np.stack(values) for name, values in matrix.items()}
        replay_rmse = float(_rms(np.stack(replay_check)))
        if replay_rmse > 1e-5:
            raise ValueError(f"{trajectory}: AG145 replay mismatch {replay_rmse} Nm")
        time = np.asarray([item[0] for item in sampled]); time -= times[0]
        for name in ("ag145", "mdpg_1s1"):
            delta = torque[name] - torque["none"]
            static_delta = static[name] - static["none"]
            dynamic_delta = delta - static_delta
            bias_delta = bias[name] - bias["none"]
            mass_delta = matrix[name] - matrix["none"]
            states_ddq = np.stack([item[1][2] for item in sampled])
            inertia_delta = np.einsum("nij,nj->ni", mass_delta, states_ddq)
            record = {
                "trajectory": trajectory, "tool": name,
                "source_trial": relative, "source_rows": len(rows), "sample_stride": stride,
                "valid_source_rows": len(valid), "duplicate_simulation_stamps_removed": duplicate_stamps,
                "excluded_warmup_rows": len(valid) - len(eligible), "warmup_seconds": warmup_seconds,
                **acceleration_stats,
                "valid_counterfactual_rows": len(sampled),
                "torque_delta_rms_all_nm": float(_rms(delta)),
                "torque_delta_abs_max_all_nm": float(np.max(np.abs(delta))),
                "static_load_delta_rms_all_nm": float(_rms(static_delta)),
                "dynamic_delta_rms_all_nm": float(_rms(dynamic_delta)),
                "dynamic_delta_abs_max_all_nm": float(np.max(np.abs(dynamic_delta))),
                "bias_delta_rms_all_nm": float(_rms(bias_delta)),
                "inertia_delta_rms_all_nm": float(_rms(inertia_delta)),
                "mass_matrix_delta_frobenius_mean": float(np.mean(np.linalg.norm(mass_delta, axis=(1, 2)))),
                "ag145_replay_rmse_nm": replay_rmse,
                "source_dynamics_sha256": _sha256(path / "dynamics.csv"),
            }
            comparison.append(record)
            for j, joint in enumerate(JOINTS):
                per_joint.append({"trajectory": trajectory, "tool": name, "joint": joint,
                                  "delta_torque_rms_nm": float(_rms(delta[:, j])),
                                  "delta_torque_abs_max_nm": float(np.max(np.abs(delta[:, j]))),
                                  "delta_static_load_rms_nm": float(_rms(static_delta[:, j])),
                                  "delta_dynamic_rms_nm": float(_rms(dynamic_delta[:, j])),
                                  "delta_bias_rms_nm": float(_rms(bias_delta[:, j])),
                                  "delta_inertia_rms_nm": float(_rms(inertia_delta[:, j]))})
            if trajectory == "T1":
                profiles[name] = (time, delta)
                dynamic_profiles[name] = (time, dynamic_delta)

    for filename, records in (("actual_T1_trials.csv", actual),
                              ("counterfactual_summary.csv", comparison),
                              ("per_joint_effects.csv", per_joint)):
        with (output / filename).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, records[0].keys())
            writer.writeheader(); writer.writerows(records)
    fig, axes = plt.subplots(1, 3, figsize=(20, 4))
    for name, color in (("ag145", "#2878b5"), ("mdpg_1s1", "#c34c40")):
        selected = [row for row in comparison if row["tool"] == name]
        shift = -.18 if name == "ag145" else .18
        axes[0].bar(np.arange(len(TRIALS)) + shift, [r["torque_delta_rms_all_nm"] for r in selected],
                    width=.35, label=name, color=color)
        axes[1].bar(np.arange(len(TRIALS)) + shift, [r["dynamic_delta_rms_all_nm"] for r in selected],
                    width=.35, label=name, color=color)
        axes[2].bar(np.arange(len(TRIALS)) + shift, [r["mass_matrix_delta_frobenius_mean"] for r in selected],
                    width=.35, label=name, color=color)
    for ax, title, ylabel in zip(axes,
                                  ("Total inverse torque effect", "Dynamic effect after static load", "Mass matrix effect"),
                                  ("RMS delta (Nm)", "RMS delta (Nm)", "Mean Frobenius norm")):
        ax.set_xticks(range(len(TRIALS)), TRIALS.keys()); ax.set_title(title); ax.set_ylabel(ylabel)
        ax.grid(axis="y", alpha=.25); ax.legend()
    fig.suptitle("AG145-recorded q, dq with common acceleration estimate in three tool models")
    fig.tight_layout(); fig.savefig(output / "tool_effect_by_trajectory.png", dpi=160); plt.close(fig)
    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    for j, ax in enumerate(axes.flat):
        for name, color in (("ag145", "#2878b5"), ("mdpg_1s1", "#c34c40")):
            time, delta = profiles[name]
            ax.plot(time, delta[:, j], label=name, color=color, linewidth=1)
        ax.set_title(JOINTS[j]); ax.set_ylabel("Delta inverse torque (Nm)"); ax.grid(alpha=.25)
    axes[0, 0].legend(); axes[-1, 0].set_xlabel("Simulation time (s)")
    axes[-1, 1].set_xlabel("Simulation time (s)")
    fig.suptitle("T1 tool effect at identical measured states")
    fig.tight_layout(); fig.savefig(output / "T1_tool_torque_delta.png", dpi=160); plt.close(fig)
    fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    for j, ax in enumerate(axes.flat):
        for name, color in (("ag145", "#2878b5"), ("mdpg_1s1", "#c34c40")):
            time, delta = dynamic_profiles[name]
            ax.plot(time, delta[:, j], label=name, color=color, linewidth=1)
        ax.set_title(JOINTS[j]); ax.set_ylabel("Dynamic delta (Nm)"); ax.grid(alpha=.25)
    axes[0, 0].legend(); axes[-1, 0].set_xlabel("Simulation time (s)")
    axes[-1, 1].set_xlabel("Simulation time (s)")
    fig.suptitle("T1 tool effect after removing same-pose static load")
    fig.tight_layout(); fig.savefig(output / "T1_dynamic_delta.png", dpi=160); plt.close(fig)
    summary = {
        "method": ("offline MuJoCo inverse dynamics at identical AG145-recorded q,dq and a common "
                   "centered five-sample slope of observed dq for ddq; first "
                   f"{warmup_seconds} s excluded as initialization transient; static load is inverse "
                   "torque at the same q with dq=ddq=0; stride applies only to counterfactuals; "
                   "logged ddq is used solely to validate source replay"),
        "scene_sha256": {name: _sha256(root / scene) for name, scene in SCENES.items()},
        "model_total_mass_kg": masses, "actual_T1_trials": actual,
        "counterfactual": comparison,
        "torque_definition": "MuJoCo reconstructed generalized inverse force, not measured actuator or physical torque",
        "limits": ["mDPG has one actual T1 simulation trial; T2-T14 are counterfactual only",
                   "mDPG box inertia, mounting transform and TCP are unverified",
                   "AG145 inertia is also approximate; the tool geometries and TCP frames differ",
                   "counterfactual states do not establish tracking or collision feasibility for a different tool",
                   "no independent physical torque or real robot validation"],
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--warmup-seconds", type=float, default=.5)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = analyze(output=args.output, stride=args.stride,
                     warmup_seconds=args.warmup_seconds)
    print("Saved", args.output or ROOT / "model/tool_aware_analysis")
    for row in result["counterfactual"]:
        print(row["trajectory"], row["tool"], row["torque_delta_rms_all_nm"])
