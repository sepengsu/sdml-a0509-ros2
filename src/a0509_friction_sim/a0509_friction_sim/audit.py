"""Read-only source audit and full legacy-history reproduction before engine use."""
import csv
from datetime import datetime, timezone
import json
from pathlib import Path

import mujoco as mj
import numpy as np

from .common import (build_baseline, checked_source, digest, original_functions,
                     reconstruct_legacy, save_npz, sha256, write_json)
from .laws import FrictionState, resistance, scenarios


def reproduce(source, output, config, run_identity):
    manifest = json.loads((source/"manifest.json").read_text(encoding="utf-8"))
    if manifest["config"] != config["legacy"] or manifest["config_sha256"] != config["original_config_sha256"]:
        raise ValueError("Source manifest does not match the pinned original configuration")
    model = build_baseline("bare_p0", output/"audit/models/bare_p0.xml", config)
    limits = model.jnt_range.copy()
    ref = original_functions()
    chosen = scenarios(config, baseline=False)
    original_order = manifest["scenario_order"]
    if [s["name"] for s in chosen] != [s["name"] for s in original_order][:len(chosen)]:
        raise ValueError("Legacy scenario order differs")
    cache, errors = {}, np.zeros((len(chosen), 6))
    count = 0
    for number, entry in enumerate(manifest["trials"], 1):
        path = checked_source(source, entry)
        key = (entry["trajectory"], entry["trial"])
        history = output/"audit/legacy_history"/entry["trajectory"]/f"trial_{entry['trial']:03d}.npz"
        if key not in cache:
            (t, q, v, a), selected = reconstruct_legacy(entry, config, limits)
            curves, z_curves = [], []
            for scenario in [s for s in chosen if s["model"] == "lugre"]:
                fc = np.asarray(config["legacy"]["friction_base_nm"])*scenario["scale"]
                b = np.asarray(config["legacy"]["viscous_base_nm_s_rad"])*scenario["scale"]
                expected = ref["_lugre_friction"](t, v, fc, b, config["legacy"])
                state = FrictionState(scenario, config)
                actual, states = [], []
                for stamp, velocity in zip(t, v):
                    actual.append(-state.advance(float(stamp), velocity))
                    states.append(state.z.copy())
                if not np.allclose(actual, expected, atol=1e-12, rtol=1e-12):
                    raise ValueError("New causal LuGre integrator differs from the original implementation")
                curves.append(np.asarray(actual)[selected])
                z_curves.append(np.asarray(states)[selected])
            arrays = {"time_s": t[selected], "q": q[selected], "dq": v[selected], "ddq": a[selected],
                      "lugre_nm": np.stack(curves, axis=1) if curves else np.zeros((len(selected),0,6)),
                      "lugre_z": np.stack(z_curves, axis=1) if curves else np.zeros((len(selected),0,6))}
            save_npz(history, arrays)
            write_json(history.with_suffix(".json"), {"identity": run_identity, "npz_sha256": sha256(history),
                       "initial_time_s": 0., "initial_z_rad": [0.]*6, "history_steps": len(t),
                       "history_dt_s": .1, "trim_after_full_history": True, "seed": entry["seed"]})
            cache[key] = arrays
        arrays = cache[key]
        with np.load(path, allow_pickle=False) as saved:
            state = np.concatenate((arrays["q"], arrays["dq"], arrays["ddq"]), axis=1).astype(np.float32)
            if not np.array_equal(state, saved["features"][:,:18]) or not np.array_equal(
                    arrays["time_s"].astype(np.float32), saved["time_s"]):
                raise ValueError(f"Original full trajectory/seed could not be reproduced: {path}")
            delta = saved["delta_friction_nm"]
            for k, scenario in enumerate(chosen):
                if scenario["model"] == "coulomb":
                    bound = np.asarray(config["legacy"]["friction_base_nm"])*scenario["scale"]
                    if np.any(np.abs(delta[:,k]) > bound+5e-5):
                        raise ValueError("Legacy Coulomb delta exceeds native frictionloss bounds")
                    continue
                if scenario["model"] == "lugre":
                    values = arrays["lugre_nm"][:, list(config["legacy"]["levels"]).index(scenario["level"])]
                else:
                    values = resistance(arrays["dq"], scenario, config)
                errors[k] = np.maximum(errors[k], np.max(np.abs(values-delta[:,k]), axis=0))
            count += len(state)
        if number % 48 == 0:
            print(f"Legacy reproduction {number}/336 chunks", flush=True)
    if errors.max() > config["offline"]["friction_tolerance_nm"]:
        raise ValueError("Reproduced legacy friction differs beyond tolerance")
    result = {"identity": run_identity, "all_chunks_verified": len(manifest["trials"])==336,
              "verified_chunks": len(manifest["trials"]), "verified_states": count,
              "coefficient_config_sha256": digest(config["legacy"]),
              "generator_sha256": config["original_generator_sha256"],
              "original_trajectory_features_float32_bit_exact": True,
              "lugre_verified": config["include_lugre_after_reproduction"], "initial_z_rad": [0.]*6,
              "history_source": "full 0..452 s trace at 0.1 s, advanced before trimming",
              "max_formula_error_nm_by_scenario_joint": errors.tolist(),
              "coulomb_note": "Bound checked here; independent MuJoCo inverse difference checked in Offline stage"}
    write_json(output/"audit/legacy_reproduction.json", result)
    return result


def damping_audit(legacy_speed_root, output, config, run_identity):
    d = np.asarray(config["background_damping_nm_s_rad"], dtype=float)
    limits = np.asarray(config["effort_limits_nm"], dtype=float)
    files = sorted((legacy_speed_root/"simulation").rglob("metadata.json"))
    if not files:
        raise ValueError("No existing completed speed rollouts found; provide --legacy-speed-root for the damping audit")
    peaks, sums = np.zeros((3,6)), np.zeros((3,6))
    near = np.zeros(6, dtype=np.int64)
    count, used, failed_attempts = 0, [], []
    for file in files:
        meta = json.loads(file.read_text(encoding="utf-8"))
        path = file.parent/"samples.npz"
        if meta.get("status") != "complete" or sha256(path) != meta["npz_sha256"]:
            raise ValueError("Existing accepted rollout checksum differs")
        with np.load(path, allow_pickle=False) as saved:
            x, actuator = saved["features"].astype(float), saved["tau_actuator_nm"].astype(float)
            damping = d*x[:,6:12]
            rigid = saved["tau_reference_nm"].astype(float)-damping
            for i, value in enumerate((damping, rigid, actuator)):
                peaks[i] = np.maximum(peaks[i], np.abs(value).max(axis=0))
                sums[i] += (value**2).sum(axis=0)
            near += (np.abs(actuator) >= .999*limits).sum(axis=0)
            count += len(x)
        used.append({"metadata": str(file), "npz_sha256": meta["npz_sha256"], "rows": meta["sample_count"]})
        failed_attempts.extend(meta.get("speed_selection_attempts", []))
    rms = np.sqrt(sums/max(count,1))
    rows = [{"joint": f"J{j+1}", "damping_nm_s_rad": d[j], "motor_limit_nm": limits[j],
             "damping_only_limit_speed_deg_s": float(np.rad2deg(limits[j]/d[j])),
             "damping_peak_nm": peaks[0,j], "rigid_peak_nm": peaks[1,j], "actuator_peak_nm": peaks[2,j],
             "damping_rms_nm": rms[0,j], "rigid_rms_nm": rms[1,j], "actuator_rms_nm": rms[2,j],
             "accepted_near_limit_ratio": near[j]/max(count,1)} for j in range(6)]
    path = output/"audit/damping_joint_summary.csv"
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    result = {"identity": run_identity, "snapshot_utc": datetime.now(timezone.utc).isoformat(),
              "accepted_rollouts": len(used), "state_rows": count, "joints": rows, "input_files": used,
              "speed_selection_attempts": failed_attempts, "provenance": config["source_provenance"],
              "assessment": "Damping dominates several axes and limits motion speed; no calibration evidence found. Retain unchanged, do not call it measured robot friction.",
              "saturation_interpretation": "Accepted v1 rollouts reject clipping. Zero observed ratio is selection-conditioned; rejected trajectories' full saturation ratios cannot be recovered.",
              "force_convention": "M*qacc+bias = actuator+passive+constraint+external; inverse=actuator+external; damping is passive=-D*dq; native frictionloss is a constraint."}
    write_json(output/"audit/model_analysis.json", result)
    lines = ["# A0509 model analysis before friction-engine implementation", "",
             "The original 800/600 damping is preserved. Source: Doosan commit 7e6d640e, 2025-07-04. No calibration evidence was found.",
             f"Read-only snapshot: {len(used)} accepted rollouts, {count:,} states.", "",
             "|Joint|D (Nm s/rad)|Limit (Nm)|Damping RMS|Rigid-body RMS|Damping-only speed limit (deg/s)|", "|---|---:|---:|---:|---:|---:|"]
    for row in rows:
        lines.append(f"|{row['joint']}|{row['damping_nm_s_rad']:.0f}|{row['motor_limit_nm']:.0f}|{row['damping_rms_nm']:.5f}|{row['rigid_rms_nm']:.5f}|{row['damping_only_limit_speed_deg_s']:.3f}|")
    lines += ["", result["assessment"], result["saturation_interpretation"], "",
              "Native damping is already included in inverse dynamics. Native dry friction is already included through constraints. External extra friction must be subtracted once from qfrc_inverse to obtain required motor torque.",
              "Original LuGre source and z(0)=0 were recovered in the sibling modeling project; full-history replay is checked before enabling it."]
    (output/"audit/ANALYSIS.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    return result


def analyze(source, legacy_speed_root, output, config, run_identity):
    physics = damping_audit(legacy_speed_root, output, config, run_identity)
    replay = reproduce(source, output, config, run_identity)
    from .plots import audit_plots
    audit_plots(output, config, physics)
    print(f"Analysis: {physics['accepted_rollouts']} accepted source rollouts; {replay['verified_chunks']} legacy chunks reproduced.", flush=True)
