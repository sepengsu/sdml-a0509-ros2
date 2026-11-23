"""Independent inverse-dynamics comparisons on exactly identical states."""
import json
import numpy as np
import mujoco as mj

from .common import (build_baseline, checked_source, marker_complete, save_npz, sha256, write_json)
from .laws import configure_model, inverse_at, resistance, scenarios


def condition_worker(condition, source, output, config, run_identity):
    manifest = json.loads((source/"manifest.json").read_text(encoding="utf-8"))
    model = build_baseline(condition, output/"models"/condition/"_offline_baseline.xml", config)
    chosen = scenarios(config, baseline=False)
    baseline = {"name": "baseline", "model": "baseline", "scale": 0., "level": "none"}
    background = config["background_damping_nm_s_rad"]
    for entry in [e for e in manifest["trials"] if e["condition"] == condition]:
        folder = output/"offline"/entry["trajectory"]/condition/f"trial_{entry['trial']:03d}"
        if marker_complete(folder, run_identity):
            continue
        path = checked_source(source, entry)
        history = output/"audit/legacy_history"/entry["trajectory"]/f"trial_{entry['trial']:03d}.npz"
        proof = json.loads(history.with_suffix(".json").read_text(encoding="utf-8"))
        if proof["identity"] != run_identity or sha256(history) != proof["npz_sha256"]:
            raise ValueError("Original full-history proof missing or changed; run -Analyze")
        with np.load(history, allow_pickle=False) as data:
            size = len(data["time_s"])
            limit = config["offline"]["samples_per_chunk"]
            indices = np.unique(np.linspace(0, size-1, min(size,limit), dtype=int)) if limit else np.arange(size)
            times, q, v, a, lugre = (data[k][indices] for k in ("time_s", "q", "dq", "ddq", "lugre_nm"))
        with np.load(path, allow_pickle=False) as old:
            features, old_reference, old_delta = (old[k][indices].copy() for k in ("features", "tau_reference_nm", "delta_friction_nm"))
        n, s = len(indices), len(chosen)
        base, rigid = np.empty((n,6)), np.empty((n,6))
        configure_model(model, baseline, config)
        data = mj.MjData(model)
        for i in range(n):
            base[i], parts = inverse_at(model, data, q[i], v[i], a[i], background=background)
            rigid[i] = parts["tau_rbd_nm"]
        if np.max(np.abs(base-old_reference)) > config["offline"]["torque_tolerance_nm"]:
            raise ValueError("Restored physical model does not reproduce legacy reference torque")
        motor, raw, passive, constraint, external, extra = [np.empty((n,s,6)) for _ in range(6)]
        for k, scenario in enumerate(chosen):
            configure_model(model, scenario, config)
            data = mj.MjData(model)
            if scenario["model"] == "lugre":
                force = -lugre[:,list(config["legacy"]["levels"]).index(scenario["level"])]
            elif scenario["model"] in ("stribeck", "asymmetric"):
                force = -resistance(v, scenario, config)
            else:
                force = np.zeros((n,6))
            for i in range(n):
                raw[i,k], parts = inverse_at(model, data, q[i], v[i], a[i], force[i], background)
                motor[i,k] = raw[i,k]-force[i]
                passive[i,k], constraint[i,k], external[i,k], extra[i,k] = (
                    parts["qfrc_passive"], parts["qfrc_constraint"], force[i], parts["qfrc_additional_friction"])
        delta = motor-base[:,None,:]
        identity_error = delta+extra
        difference = delta-old_delta[:,:s]
        error = float(np.max(np.abs(difference)))
        if error > config["offline"]["torque_tolerance_nm"] or np.max(np.abs(identity_error)) > 1e-6:
            raise ValueError(f"Friction duplication/sign/legacy comparison failed: {error} Nm")
        arrays = {"source_row_indices": indices, "time_s": times, "features": features,
                  "tau_rbd_nm": rigid, "tau_background_reference_nm": base,
                  "tau_required_by_scenario_nm": motor, "qfrc_inverse_native_nm": raw,
                  "qfrc_passive": passive, "qfrc_constraint": constraint,
                  "qfrc_external_friction": external, "qfrc_additional_friction": extra,
                  "legacy_delta_friction_nm": old_delta[:,:s], "difference_from_legacy_nm": difference}
        save_npz(folder/"comparison.npz", arrays)
        write_json(folder/"metadata.json", {"identity": run_identity, "status": "complete",
                   "condition": condition, "trajectory": entry["trajectory"], "trial": entry["trial"],
                   "split": entry["split"], "rows": n, "scenario_order": chosen,
                   "max_legacy_difference_nm": error, "max_force_identity_error_nm": float(np.max(np.abs(identity_error))),
                   "scenario_joint_rmse_vs_legacy_nm": np.sqrt(np.mean(difference**2,axis=0)).tolist(),
                   "source_npz_sha256": entry["npz_sha256"], "same_states_for_all_scenarios": True,
                   "closed_loop_rollouts": False, "files": {"comparison.npz": sha256(folder/"comparison.npz")}})
    print(f"Offline {condition}: validated all 48 source chunks against {len(chosen)} scenarios", flush=True)


def summarize(output, config, run_identity):
    records = []
    for path in sorted((output/"offline").rglob("metadata.json")):
        meta = marker_complete(path.parent, run_identity)
        if meta:
            records.append(meta)
    passed = len(records)==336 and all(r["max_legacy_difference_nm"] <= config["offline"]["torque_tolerance_nm"] for r in records)
    result = {"identity": run_identity, "all_scenario_labels_passed": passed, "completed_chunks": len(records),
              "states_checked": sum(r["rows"] for r in records), "scenarios": len(scenarios(config, False)),
              "max_legacy_difference_nm": max((r["max_legacy_difference_nm"] for r in records),default=0.),
              "max_force_identity_error_nm": max((r["max_force_identity_error_nm"] for r in records),default=0.),
              "native_coulomb_already_compensated": True, "external_friction_subtracted_once": True}
    write_json(output/"offline/summary.json", result)
    return result
