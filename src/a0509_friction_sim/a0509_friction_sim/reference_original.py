"""Generate three distinct offline MuJoCo inverse-dynamics trials per friction scenario.

This is a synthetic state-labeling campaign. It does not execute a robot, ROS,
or closed-loop MuJoCo rollout. Existing repeated source trials are deterministic
copies, so each new trial uses a seeded smooth trajectory perturbation instead.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
import time

import mujoco
import numpy as np

from .combined import CONDITIONS, ROOT, SOURCE_REPO


CONFIG_PATH = ROOT / "src/model/friction_collection_config.json"
SOURCE_MANIFEST = ROOT / "data/wide16_v3_joint_dataset/manifest.json"
DEFAULT_OUTPUT = ROOT / "data/friction_diverse_v1"
def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def config_digest(config: dict) -> str:
    canonical = json.dumps(config, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(canonical).hexdigest()


def scenarios(config: dict) -> list[dict]:
    return [{"name": f"{model}_{level}", "model": model, "level": level, "scale": scale}
            for model in config["models"] for level, scale in config["levels"].items()]


def _split_lookup(config: dict) -> dict[str, str]:
    lookup = {trajectory: split for split, names in config["split_policy"].items()
              for trajectory in names}
    if len(lookup) != 16 or set(lookup) != {f"T{i}" for i in range(1, 17)}:
        raise ValueError("expected exactly T1-T16 in disjoint splits")
    return lookup


def _validate_config(config: dict) -> None:
    if config["trials"] != 3 or config["stride_from_100hz"] < 1:
        raise ValueError("this collection requires exactly three trials and a positive stride")
    if tuple(config["models"]) != ("coulomb", "viscous", "stribeck", "asymmetric", "lugre"):
        raise ValueError("friction model set changed")
    if len(config["levels"]) != 3 or any(float(v) <= 0 for v in config["levels"].values()):
        raise ValueError("expected three positive severity levels")
    if any(len(config[key]) != 6 or not np.isfinite(config[key]).all()
           or np.any(np.asarray(config[key]) <= 0)
           for key in ("friction_base_nm", "viscous_base_nm_s_rad")):
        raise ValueError("invalid six-joint friction parameters")
    _split_lookup(config)


def _source_paths(source_repo: Path, trajectory: str, condition: str) -> tuple[Path, Path]:
    campaign = source_repo / "data/grid_16_wide_v3"
    scene = campaign / "models/conditions" / f"{condition}.xml"
    plan = campaign / "simulation" / trajectory / "bare_p0/trial_001/trajectory.csv"
    if not scene.is_file() or not plan.is_file():
        raise FileNotFoundError(f"scene or source trajectory missing: {scene}, {plan}")
    return scene, plan


def _read_plan(path: Path, stride: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    times, q, dq, ddq = [], [], [], []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        for index, row in enumerate(reader):
            if index % stride:
                continue
            times.append(float(row["time_s"]))
            q.append([float(row[f"q{j}_rad"]) for j in range(1, 7)])
            dq.append([float(row[f"dq{j}_rad_s"]) for j in range(1, 7)])
            ddq.append([float(row[f"ddq{j}_rad_s2"]) for j in range(1, 7)])
    arrays = tuple(np.asarray(values, dtype=np.float64) for values in (times, q, dq, ddq))
    t, q_array, dq_array, ddq_array = arrays
    if (len(t) < 100 or t[0] != 0 or any(value.shape != (len(t), 6)
                                           for value in (q_array, dq_array, ddq_array))
            or not all(np.isfinite(value).all() for value in arrays)
            or not np.allclose(np.diff(t), stride / 100, atol=1e-7)):
        raise ValueError(f"invalid 100 Hz source plan: {path}")
    return arrays


def _perturb_plan(plan: tuple[np.ndarray, ...], trajectory: str, trial: int,
                  config: dict, joint_limits: np.ndarray) -> tuple[np.ndarray, ...]:
    t, q, dq, ddq = plan
    settings = config["perturbation"]
    seed = config["seed_base"] + 1000 * int(trajectory[1:]) + trial
    rng = np.random.default_rng(seed)
    amplitude = rng.uniform(settings["amplitude_min_rad"], settings["amplitude_max_rad"], 6)
    frequency = rng.uniform(settings["frequency_min_hz"], settings["frequency_max_hz"], 6)
    phase = rng.uniform(-np.pi, np.pi, 6)
    u = t / t[-1]
    sine = np.sin(np.pi * u)
    cosine = np.cos(np.pi * u)
    envelope = sine ** 4
    envelope_d = 4 * sine ** 3 * cosine * np.pi / t[-1]
    envelope_dd = 4 * np.pi ** 2 * (3 * sine ** 2 * cosine ** 2 - sine ** 4) / t[-1] ** 2
    omega = 2 * np.pi * frequency
    argument = t[:, None] * omega + phase
    wave, wave_d = np.sin(argument), np.cos(argument)
    change_q = amplitude * envelope[:, None] * wave
    change_dq = amplitude * (envelope_d[:, None] * wave
                             + envelope[:, None] * omega * wave_d)
    change_ddq = amplitude * (envelope_dd[:, None] * wave
                              + 2 * envelope_d[:, None] * omega * wave_d
                              - envelope[:, None] * omega ** 2 * wave)
    margin = settings["joint_limit_margin_rad"]
    factor = 1.0
    while factor >= 1 / 64:
        candidate = q + factor * change_q
        if (np.all(candidate >= joint_limits[:, 0] + margin)
                and np.all(candidate <= joint_limits[:, 1] - margin)):
            break
        factor /= 2
    else:
        raise ValueError(f"trial perturbation exceeds joint limits: {trajectory}/{trial}")
    return (t, candidate, dq + factor * change_dq, ddq + factor * change_ddq, seed, factor)


def _smooth_friction(velocity: np.ndarray, fc: np.ndarray, b: np.ndarray,
                     config: dict) -> np.ndarray:
    settings = config["stribeck"]
    fs = fc * settings["static_to_coulomb_ratio"]
    vs = settings["transition_rad_s"]
    epsilon = settings["smoothing_rad_s"]
    return ((fc + (fs - fc) * np.exp(-(np.abs(velocity) / vs) ** 2))
            * np.tanh(velocity / epsilon) + b * velocity)


def _asymmetric_friction(velocity: np.ndarray, fc: np.ndarray, b: np.ndarray,
                         config: dict) -> np.ndarray:
    settings = config["asymmetric"]
    strength = np.where(velocity >= 0, settings["positive_ratio"] * fc,
                        settings["negative_ratio"] * fc)
    return strength * np.tanh(velocity / settings["smoothing_rad_s"]) + 0.5 * b * velocity


def _lugre_friction(times: np.ndarray, velocity: np.ndarray, fc: np.ndarray,
                    b: np.ndarray, config: dict) -> np.ndarray:
    settings = config["lugre"]
    stiffness = settings["stiffness_nm_rad"]
    microdamping = settings["microdamping_nm_s_rad"]
    fs = settings["static_to_coulomb_ratio"] * fc
    vs = settings["transition_rad_s"]
    z = np.zeros(6, dtype=np.float64)
    torque = np.empty_like(velocity)
    for index, v in enumerate(velocity):
        dt = 0 if index == 0 else times[index] - times[index - 1]
        g = (fc + (fs - fc) * np.exp(-(np.abs(v) / vs) ** 2)) / stiffness
        rate = np.abs(v) / g
        decay = np.exp(-rate * dt)
        next_z = z * decay + np.where(rate > 1e-12, v * (-np.expm1(-rate * dt))
                                       / np.maximum(rate, 1e-12), v * dt)
        z_dot = (next_z - z) / dt if dt else np.zeros(6)
        torque[index] = stiffness * next_z + microdamping * z_dot + settings["viscous_ratio"] * b * v
        z = next_z
    return torque


def _model_ids(model: mujoco.MjModel) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    joints = np.asarray([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT,
                                           f"joint_{joint}") for joint in range(1, 7)], dtype=int)
    if np.any(joints < 0) or model.nq != 6 or model.nv != 6:
        raise ValueError("expected six-joint A0509 model")
    return (model.jnt_qposadr[joints].astype(int),
            model.jnt_dofadr[joints].astype(int), model.jnt_range[joints].copy())


def _inverse(model: mujoco.MjModel, data: mujoco.MjData, q: np.ndarray,
             dq: np.ndarray, ddq: np.ndarray, qpos: np.ndarray,
             dofs: np.ndarray) -> np.ndarray:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0
    data.qacc[:] = 0
    data.qpos[qpos] = q
    data.qvel[dofs] = dq
    mujoco.mj_forward(model, data)
    if data.ncon:
        raise ValueError(f"generated state has {data.ncon} contacts")
    data.qacc[dofs] = ddq
    mujoco.mj_inverse(model, data)
    return data.qfrc_inverse[dofs].copy()


def _scene_models(scene: Path, source_repo: Path, config: dict) -> tuple[tuple, dict[str, tuple]]:
    if str(source_repo.resolve()) not in sys.path:
        sys.path.insert(0, str(source_repo.resolve()))
    from model.campaign_rbd import load_scene

    exact = load_scene(scene)
    qpos, dofs, limits = _model_ids(exact)
    if np.any(exact.dof_frictionloss[dofs] != 0):
        raise ValueError("source scene already contains joint frictionloss")
    dry = {}
    for level, scale in config["levels"].items():
        model = load_scene(scene)
        model.dof_frictionloss[dofs] = np.asarray(config["friction_base_nm"]) * scale
        dry[level] = (model, mujoco.MjData(model))
    return (exact, mujoco.MjData(exact), qpos, dofs, limits), dry


def _features(condition: str, q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> np.ndarray:
    onehot = np.zeros((len(q), 3), dtype=np.float32)
    index = 0 if condition == "bare_p0" else 1 if condition.startswith("ag145") else 2
    onehot[:, index] = 1
    mass = 0.0 if condition.endswith("_p0") else 0.5 if condition.endswith("_p05") else 1.0
    payload = np.full((len(q), 1), mass, dtype=np.float32)
    return np.concatenate((q, dq, ddq, onehot, payload), axis=1).astype(np.float32)


def _build_chunk(trajectory: str, condition: str, trial: int, plan: tuple[np.ndarray, ...],
                 models: tuple[tuple, dict], config: dict, max_rows: int | None) -> tuple[dict, dict]:
    exact, dry = models
    model, data, qpos, dofs, limits = exact
    t, q, dq, ddq, seed, amplitude_factor = _perturb_plan(plan, trajectory, trial, config, limits)
    trim = config["trim_seconds_each_end"]
    selected = np.flatnonzero((t >= trim) & (t <= t[-1] - trim))
    if max_rows is not None and len(selected) > max_rows:
        selected = selected[np.linspace(0, len(selected) - 1, max_rows, dtype=int)]
    if len(selected) < 10:
        raise ValueError("too few selected states")
    t_out, q_out, dq_out, ddq_out = (value[selected]
                                   for value in (t, q, dq, ddq))
    reference = np.empty((len(selected), 6), dtype=np.float64)
    coulomb = {level: np.empty_like(reference) for level in config["levels"]}
    for index, (position, velocity, acceleration) in enumerate(zip(q_out, dq_out, ddq_out)):
        reference[index] = _inverse(model, data, position, velocity, acceleration, qpos, dofs)
        for level, (dry_model, dry_data) in dry.items():
            coulomb[level][index] = _inverse(
                dry_model, dry_data, position, velocity, acceleration, qpos, dofs)
    chosen = scenarios(config)
    delta = np.empty((len(selected), len(chosen), 6), dtype=np.float32)
    fc = np.asarray(config["friction_base_nm"], dtype=np.float64)
    b = np.asarray(config["viscous_base_nm_s_rad"], dtype=np.float64)
    for column, scenario in enumerate(chosen):
        level, scale, kind = scenario["level"], scenario["scale"], scenario["model"]
        if kind == "coulomb":
            values = coulomb[level] - reference
            if np.any(np.abs(values) > fc * scale + 5e-5):
                raise ValueError(f"MuJoCo dry-friction bound violated: {condition}/{trajectory}")
        elif kind == "viscous":
            values = scale * b * dq_out
        elif kind == "stribeck":
            values = _smooth_friction(dq_out, fc * scale, b * scale, config)
        elif kind == "asymmetric":
            values = _asymmetric_friction(dq_out, fc * scale, b * scale, config)
        elif kind == "lugre":
            values = _lugre_friction(t, dq, fc * scale, b * scale, config)[selected]
        else:
            raise ValueError(kind)
        delta[:, column] = values
    features = _features(condition, q_out, dq_out, ddq_out)
    if not all(np.isfinite(value).all() for value in (features, reference, delta)):
        raise ValueError("nonfinite generated data")
    if features.shape != (len(selected), 22) or delta.shape != (len(selected), 15, 6):
        raise ValueError("generated array shape mismatch")
    state = np.ascontiguousarray(features[:, :18])
    report = {"trajectory": trajectory, "condition": condition, "trial": trial,
              "seed": seed, "amplitude_factor": amplitude_factor,
              "rows": len(selected), "time_start_s": float(t_out[0]),
              "time_end_s": float(t_out[-1]),
              "state_sha256": hashlib.sha256(state.tobytes()).hexdigest(),
              "max_abs_delta_nm": float(np.max(np.abs(delta)))}
    arrays = {"time_s": t_out.astype(np.float32), "features": features,
              "tau_reference_nm": reference.astype(np.float32),
              "delta_friction_nm": delta}
    return arrays, report


def _atomic_npz(path: Path, arrays: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".trial_", suffix=".npz",
                                     delete=False) as handle:
        temporary = Path(handle.name)
    try:
        np.savez_compressed(temporary, **arrays)
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _atomic_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".trial_", suffix=".json",
                                     mode="w", encoding="utf-8", delete=False) as handle:
        temporary = Path(handle.name)
        json.dump(payload, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
    try:
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def _chunk_paths(output: Path, trajectory: str, condition: str, trial: int) -> tuple[Path, Path]:
    root = output / "chunks" / trajectory / condition
    return root / f"trial_{trial:03d}.npz", root / f"trial_{trial:03d}.json"


def _audit_split_overlap(output: Path, records: list[dict]) -> dict:
    keys: dict[str, list[np.ndarray]] = {"train": [], "validation": [], "test": []}
    for item in records:
        with np.load(output / item["relative_npz"], allow_pickle=False) as saved:
            features = np.ascontiguousarray(saved["features"])
        dtype = np.dtype((np.void, features.dtype.itemsize * features.shape[1]))
        keys[item["split"]].append(features.view(dtype).ravel())
    distinct = {split: np.unique(np.concatenate(parts)) if parts else np.array([], dtype=np.void)
                for split, parts in keys.items()}
    overlap = {f"{a}_{b}": int(len(np.intersect1d(distinct[a], distinct[b])))
               for a, b in (("train", "validation"), ("train", "test"), ("validation", "test"))}
    if any(overlap.values()):
        raise ValueError(f"exact feature leakage across trajectory splits: {overlap}")
    return overlap


def collect(source_repo: Path, output: Path, config: dict, *,
            conditions: list[str], trajectories: list[str],
            max_rows_per_trial: int | None = None) -> dict:
    _validate_config(config)
    split = _split_lookup(config)
    if any(name not in CONDITIONS for name in conditions) or any(name not in split for name in trajectories):
        raise ValueError("unknown condition or trajectory")
    if max_rows_per_trial is not None and max_rows_per_trial < 10:
        raise ValueError("max rows per trial must be at least 10")
    source_repo, output = source_repo.resolve(), output.resolve()
    source_manifest = json.loads(SOURCE_MANIFEST.read_text(encoding="utf-8"))
    if tuple(source_manifest["conditions"]) != CONDITIONS:
        raise ValueError("wide16 source condition mismatch")
    source_archive = SOURCE_MANIFEST.parent / "dataset.npz"
    if sha256(source_archive) != source_manifest["dataset_sha256"]:
        raise ValueError("wide16 source archive digest mismatch")
    config_sha = config_digest(config)
    partial = (set(conditions) != set(CONDITIONS)
               or set(trajectories) != set(split)
               or max_rows_per_trial is not None)
    plan_record = {"config_sha256": config_sha, "config": config,
                   "conditions": conditions, "trajectories": trajectories,
                   "max_rows_per_trial": max_rows_per_trial, "partial": partial,
                   "source_dataset_sha256": source_manifest["dataset_sha256"],
                   "scenario_order": scenarios(config)}
    plan_path = output / "collection_plan.json"
    if output.exists():
        if not plan_path.is_file() or json.loads(plan_path.read_text(encoding="utf-8")) != plan_record:
            raise ValueError(f"existing output is a different collection: {output}")
    else:
        output.mkdir(parents=True)
        _atomic_json(plan_path, plan_record)
    plan_cache = {}
    for trajectory in trajectories:
        _, source_plan = _source_paths(source_repo, trajectory, "bare_p0")
        plan_cache[trajectory] = (_read_plan(source_plan, config["stride_from_100hz"]),
                                  sha256(source_plan))
    records = []
    start = time.perf_counter()
    for condition in conditions:
        scene, _ = _source_paths(source_repo, trajectories[0], condition)
        if sha256(scene) != source_manifest["scene_sha256"][condition]:
            raise ValueError(f"source scene digest mismatch: {scene}")
        models = _scene_models(scene, source_repo, config)
        for trajectory in trajectories:
            plan, plan_sha = plan_cache[trajectory]
            trial_states = set()
            for trial in range(1, config["trials"] + 1):
                npz_path, meta_path = _chunk_paths(output, trajectory, condition, trial)
                if npz_path.exists() or meta_path.exists():
                    if not npz_path.is_file() or not meta_path.is_file():
                        raise ValueError(f"incomplete chunk pair: {npz_path}")
                    item = json.loads(meta_path.read_text(encoding="utf-8"))
                    if (item["config_sha256"] != config_sha or item["source_scene_sha256"] != sha256(scene)
                            or item["source_plan_sha256"] != plan_sha
                            or item["npz_sha256"] != sha256(npz_path)):
                        raise ValueError(f"existing chunk verification failed: {npz_path}")
                    print(f"Verified existing {condition}/{trajectory}/trial_{trial:03d}", flush=True)
                else:
                    arrays, item = _build_chunk(trajectory, condition, trial, plan, models,
                                                config, max_rows_per_trial)
                    item.update({"split": split[trajectory], "config_sha256": config_sha,
                                 "source_scene_sha256": sha256(scene),
                                 "source_plan_sha256": plan_sha,
                                 "relative_npz": str(npz_path.relative_to(output)).replace("\\", "/")})
                    _atomic_npz(npz_path, arrays)
                    item["npz_sha256"] = sha256(npz_path)
                    _atomic_json(meta_path, item)
                    print(f"Generated {condition}/{trajectory}/trial_{trial:03d}: "
                          f"{item['rows']} rows", flush=True)
                if item["state_sha256"] in trial_states:
                    raise ValueError(f"three trials have identical states: {condition}/{trajectory}")
                trial_states.add(item["state_sha256"])
                records.append(item)
    overlaps = _audit_split_overlap(output, records) if not partial else None
    manifest = {"created_utc": datetime.now(timezone.utc).isoformat(),
                "dataset_kind": "synthetic offline MuJoCo inverse-dynamics friction labels",
                "real_robot_measurements": False, "closed_loop_rollouts": False,
                "source_repeats_001_to_003_identical_in_spot_checks": True,
                "new_trials": "three seeded smooth trajectory perturbations per condition/trajectory",
                "source_repo": str(source_repo), "source_dataset_sha256": source_manifest["dataset_sha256"],
                "config_sha256": config_sha, "config": config,
                "feature_order": "q1..6,dq1..6,ddq1..6,bare/ag145/mdpg one-hot,payload_mass_kg",
                "arrays": {"features": "[rows,22] float32",
                           "tau_reference_nm": "[rows,6] float32",
                           "delta_friction_nm": "[rows,15,6] float32",
                           "time_s": "[rows] float32"},
                "target_formula": "tau_with_friction = tau_reference_nm + delta_friction_nm[:, scenario, :]",
                "scenario_order": scenarios(config),
                "partial_collection": partial, "training_ready": not partial,
                "exact_feature_split_overlap": overlaps,
                "trials": records, "trial_chunks": len(records),
                "state_rows": sum(item["rows"] for item in records),
                "scenario_state_rows": sum(item["rows"] for item in records) * len(scenarios(config)),
                "elapsed_s": time.perf_counter() - start}
    _atomic_json(output / "manifest.json", manifest)
    return manifest


def plan(source_repo: Path, config: dict, conditions: list[str],
         trajectories: list[str], max_rows_per_trial: int | None) -> dict:
    _validate_config(config)
    counts = {}
    for trajectory in trajectories:
        _, path = _source_paths(source_repo, trajectory, "bare_p0")
        times = _read_plan(path, config["stride_from_100hz"])[0]
        trim = config["trim_seconds_each_end"]
        rows = int(np.sum((times >= trim) & (times <= times[-1] - trim)))
        counts[trajectory] = min(rows, max_rows_per_trial) if max_rows_per_trial else rows
    state_rows = len(conditions) * config["trials"] * sum(counts.values())
    return {"conditions": conditions, "trajectories": trajectories,
            "trials_per_condition_trajectory": config["trials"],
            "friction_models": config["models"], "severity_levels": config["levels"],
            "scenario_count": len(scenarios(config)),
            "trial_chunks": len(conditions) * len(trajectories) * config["trials"],
            "estimated_state_rows": state_rows,
            "estimated_scenario_state_rows": state_rows * len(scenarios(config)),
            "rows_per_trajectory_trial": counts,
            "sampling_hz": 100 / config["stride_from_100hz"],
            "label": "synthetic offline inverse dynamics; no physical robot measurements"}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("plan", "collect"))
    parser.add_argument("--source-repo", type=Path, default=SOURCE_REPO)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--config", type=Path, default=CONFIG_PATH)
    parser.add_argument("--condition", action="append")
    parser.add_argument("--trajectory", action="append")
    parser.add_argument("--max-rows-per-trial", type=int)
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8"))
    conditions = args.condition or list(CONDITIONS)
    trajectories = args.trajectory or [f"T{i}" for i in range(1, 17)]
    if args.command == "plan":
        result = plan(args.source_repo, config, conditions, trajectories,
                      args.max_rows_per_trial)
    else:
        result = collect(args.source_repo, args.output, config, conditions=conditions,
                         trajectories=trajectories,
                         max_rows_per_trial=args.max_rows_per_trial)
    print(json.dumps(result if args.command == "plan" else
                     {key: result[key] for key in ("training_ready", "trial_chunks",
                                                   "state_rows", "scenario_state_rows",
                                                   "exact_feature_split_overlap")},
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
