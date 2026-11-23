"""Fixed-controller, fixed-reference friction experiments using actual mj_step."""
import csv
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

from .common import build_baseline, marker_complete, save_npz, sha256, target_states, write_json
from .laws import FrictionState, configure_model, decompose, inverse_at, parameters, scenarios
from a0509_dynamics_experiment.dynamics10_safety import POSITION_LIMITS, VELOCITY_LIMITS, ACCELERATION_LIMITS


def reference(condition, trajectory, duration, output, config, run_identity):
    sim = config["simulation"]
    path = output/"reference"/condition/trajectory/f"duration_{duration:g}.npz"
    if path.is_file() and path.with_suffix(".json").is_file():
        proof = json.loads(path.with_suffix(".json").read_text(encoding="utf-8"))
        if proof["identity"] == run_identity and sha256(path) == proof["sha256"]:
            with np.load(path, allow_pickle=False) as d:
                return tuple(d[k].copy() for k in ("time_s", "q", "dq", "ddq", "feedforward"))
    model = build_baseline(condition, output/"models"/condition/"_controller_baseline.xml", config)
    times = np.arange(round(duration*sim["sample_hz"])+1)/sim["sample_hz"]
    q, v, a = target_states(trajectory, sim["speed_scale"], times)
    low, high = np.asarray(POSITION_LIMITS).T
    if (np.any((q < low) | (q > high)) or np.any(np.abs(v) > VELOCITY_LIMITS)
            or np.any(np.abs(a) > ACCELERATION_LIMITS)):
        raise ValueError("Fixed target trajectory violates reference planning limits")
    data = mj.MjData(model)
    ff = np.empty_like(q)
    for i in range(len(times)):
        ff[i], _ = inverse_at(model, data, q[i], v[i], a[i], background=config["background_damping_nm_s_rad"])
    save_npz(path, {"time_s": times, "q": q, "dq": v, "ddq": a, "feedforward": ff})
    write_json(path.with_suffix(".json"), {"identity": run_identity, "sha256": sha256(path),
               "all_scenarios_share_identical_target_and_controller": True,
               "feedforward_model": "baseline without added scenario friction"})
    return times, q, v, a, ff


def scenario_model(condition, scenario, output, config):
    baseline = output/"models"/condition/"_controller_baseline.xml"
    if not baseline.is_file():
        build_baseline(condition, baseline, config)
    p = parameters(scenario, config)
    tree = ET.parse(baseline)
    root = tree.getroot()
    for j in range(6):
        joint = root.find(f".//joint[@name='joint_{j+1}']")
        joint.set("damping", str(p["native_damping_nm_s_rad"][j]))
        joint.set("frictionloss", str(p["native_frictionloss_nm"][j]))
    path = baseline.with_name(scenario["name"]+".xml")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    model = mj.MjModel.from_xml_path(str(path))
    configure_model(model, scenario, config)
    return model, path, p


ARRAY_FIELDS = ("q", "dq", "ddq", "target_q", "tau_actuator_nm", "tau_rbd_nm", "qfrc_inverse",
                "required_motor_inverse_nm", "qfrc_passive", "qfrc_constraint", "qfrc_native_dry",
                "qfrc_external_friction", "qfrc_additional_friction", "qfrc_background_damping",
                "qfrc_additional_damping", "requested_control_nm", "lugre_z", "lugre_z_dot",
                "force_balance_residual_nm")


def folder_for(output, job):
    return output/job["phase"]/job["condition"]/job["trajectory"]/job["scenario"]["name"]


def run_trial(job, output, config, run_identity):
    folder = folder_for(output, job)
    folder.mkdir(parents=True, exist_ok=True)
    sim = config["simulation"]
    dt, hz = sim["timestep_s"], sim["sample_hz"]
    substeps = round(1./hz/dt)
    times, q_ref, v_ref, a_ref, ff = reference(job["condition"], job["trajectory"], job["duration_s"], output, config, run_identity)
    model, model_path, p = scenario_model(job["condition"], job["scenario"], output, config)
    data, invdata = mj.MjData(model), mj.MjData(model)
    data.qpos[:] = q_ref[0]
    state = FrictionState(job["scenario"], config)
    step_times = np.arange((len(times)-1)*substeps+1)*dt
    q_cmd, v_cmd, _ = target_states(job["trajectory"], sim["speed_scale"], step_times)
    arrays = {name: np.empty((len(times),6), dtype=np.float64) for name in ARRAY_FIELDS}
    arrays["time_s"] = times
    arrays["saturated"] = np.zeros((len(times),6), dtype=bool)
    saturation = np.zeros(6, dtype=np.int64)
    peak_acceleration = np.zeros(6)
    max_tracking, peak_balance = 0., 0.
    background, limits = np.asarray(config["background_damping_nm_s_rad"]), np.asarray(config["effort_limits_nm"])
    low, high = np.asarray(POSITION_LIMITS).T
    columns = ["time_s", *[f"{name}_{j+1}" for name in ARRAY_FIELDS for j in range(6)], *[f"saturated_{j+1}" for j in range(6)]]
    started = time.monotonic()
    rows_written = 0
    warning_before = np.array([w.number for w in data.warning])
    try:
        with (folder/"samples.csv.partial").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(columns)
            for step, stamp in enumerate(step_times):
                i = min(step//substeps, len(times)-1)
                # One causal history update per physical time, never in mj_forward/mj_inverse callbacks.
                external = state.advance(float(stamp), data.qvel)
                data.qfrc_applied[:] = external
                mj.mj_forward(model, data)
                if data.ncon:
                    raise ValueError(f"Contact at physics step {step}")
                alpha = (step%substeps)/substeps
                feedforward = (1-alpha)*ff[i]+alpha*ff[min(i+1,len(times)-1)]
                command = feedforward+sim["kp_nm_rad"]*(q_cmd[step]-data.qpos)+sim["kd_nm_s_rad"]*(v_cmd[step]-data.qvel)
                mask = np.abs(command)>limits
                if step+1 < len(step_times):
                    saturation += mask
                data.ctrl[:] = np.clip(command,-limits,limits)
                mj.mj_forward(model,data)
                if (not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or not np.isfinite(data.qacc).all()
                        or np.any((data.qpos<low)|(data.qpos>high)) or np.any(np.abs(data.qvel)>VELOCITY_LIMITS)):
                    raise ValueError(f"Invalid integrated joint state at physics step {step}")
                # Raw acceleration is recorded; reference planning bounds are reported separately.
                peak_acceleration = np.maximum(peak_acceleration,np.abs(data.qacc))
                error = float(np.max(np.abs(data.qpos-q_cmd[step])))
                max_tracking = max(max_tracking,error)
                if step%substeps==0:
                    i = step//substeps
                    parts = decompose(model,data,background)
                    raw_inverse, _ = inverse_at(model,invdata,data.qpos,data.qvel,data.qacc,external,background)
                    predicted_motor = raw_inverse-external
                    peak_balance = max(peak_balance,float(np.max(np.abs(parts["force_balance_residual_nm"]))),
                                       float(np.max(np.abs(predicted_motor-data.qfrc_actuator))))
                    values = {**parts,"q":data.qpos.copy(),"dq":data.qvel.copy(),"ddq":data.qacc.copy(),
                              "target_q":q_ref[i],"tau_actuator_nm":data.qfrc_actuator.copy(),
                              "qfrc_inverse":raw_inverse,"required_motor_inverse_nm":predicted_motor,
                              "requested_control_nm":command,"lugre_z":state.z.copy(),"lugre_z_dot":state.z_dot.copy()}
                    for name in ARRAY_FIELDS:
                        arrays[name][i] = values[name]
                    arrays["saturated"][i]=mask
                    writer.writerow([format(float(stamp),".6f"),
                                     *[format(float(v),".10g") for name in ARRAY_FIELDS for v in values[name]],
                                     *map(int,mask)])
                    rows_written+=1
                if step+1<len(step_times):
                    mj.mj_step(model,data)
            if np.any(np.array([w.number for w in data.warning])>warning_before):
                raise ValueError("MuJoCo emitted a numerical warning")
    except Exception as error:
        write_json(folder/"aborted.json", {"identity":run_identity,"job":job,"status":"aborted",
                   "reason":str(error),"rows_written":rows_written,"original_models_unchanged":True})
        raise
    if peak_balance>config["offline"]["torque_tolerance_nm"]:
        raise ValueError(f"Force balance or external-force double-counting error {peak_balance} Nm")
    q_error = arrays["q"]-q_ref
    rmse = np.sqrt(np.mean(q_error**2,axis=0))
    ratio = saturation/max(len(step_times)-1,1)
    quality = (max_tracking<=sim["max_tracking_error_rad"] and np.max(rmse)<=sim["max_tracking_rmse_rad"]
               and np.max(ratio)<=sim["max_saturation_ratio"])
    arrays["target_dq"], arrays["target_ddq"] = v_ref,a_ref
    tool = [1.,0.,0.] if job["condition"].startswith("bare") else [0.,1.,0.] if job["condition"].startswith("ag145") else [0.,0.,1.]
    mass = .5 if job["condition"].endswith("p05") else 1. if job["condition"].endswith("p10") else 0.
    arrays["features"] = np.concatenate((arrays["q"],arrays["dq"],arrays["ddq"],
                                         np.tile(tool,(len(times),1)),np.full((len(times),1),mass)),axis=1).astype(np.float32)
    save_npz(folder/"samples.npz",arrays)
    (folder/"samples.csv.partial").replace(folder/"samples.csv")
    meta = {"identity":run_identity,"status":"complete","job":job,"rows":len(times),"quality_pass":bool(quality),
            "closed_loop_rollouts":True,"real_robot_measurements":False,"friction_parameters":p,
            "model_sha256":sha256(model_path),"model_file":str(model_path),"simulation":sim,
            "split":job["split"],"random_seed":sim["random_seed"],"state_updates":len(step_times),
            "physics_steps":len(step_times)-1,"tracking_rmse_rad":rmse.tolist(),"max_tracking_error_rad":max_tracking,
            "saturation_ratio_by_joint":ratio.tolist(),"saturation_physics_steps_by_joint":saturation.tolist(),
            "max_force_balance_error_nm":peak_balance,"peak_observed_acceleration_rad_s2":peak_acceleration.tolist(),
            "reference_acceleration_guideline_exceeded":(peak_acceleration>ACCELERATION_LIMITS).tolist(),
            "wall_seconds":time.monotonic()-started,"is_full_trajectory":job["phase"]=="dynamic",
            "same_reference_controller_all_scenarios":True,"initial_q":q_ref[0].tolist(),"initial_dq":[0.]*6,
            "initial_lugre_z":[0.]*6,"coefficient_dt_note":"Original offline LuGre uses 0.1s; Dynamic advances actual velocity at physics timestep.",
            "force_convention":"RBD=actuator+native_passive+constraint+external; inverse=actuator+external; required motor=inverse-external",
            "files":{"samples.npz":sha256(folder/"samples.npz"),"samples.csv":sha256(folder/"samples.csv")}}
    write_json(folder/"metadata.json",meta)
    print(f"Dynamic {job['scenario']['name']} {job['condition']} {job['trajectory']}: quality={quality}, max saturation={ratio.max():.6g}, max balance={peak_balance:.3g} Nm",flush=True)
    return meta


def jobs(config, phase):
    from .common import CONDITIONS,SPLITS
    conditions = config["pilot"]["conditions"] if phase=="pilot" else CONDITIONS
    trajectories = config["pilot"]["trajectories"] if phase=="pilot" else [f"T{i}" for i in range(1,17)]
    duration = config["pilot"]["duration_s"] if phase=="pilot" else 2.+np.ceil(450./config["simulation"]["speed_scale"]*config["simulation"]["sample_hz"])/config["simulation"]["sample_hz"]
    return [{"condition":c,"trajectory":t,"scenario":s,"phase":phase,"duration_s":float(duration),"split":SPLITS[t]}
            for c in conditions for t in trajectories for s in scenarios(config)]


def summary(output, config, run_identity, phase):
    found=[]
    for job in jobs(config,phase):
        meta=marker_complete(folder_for(output,job),run_identity)
        if meta:
            found.append(meta)
    result={"identity":run_identity,"phase":phase,"completed":len(found),"planned":len(jobs(config,phase)),
            "quality_passed":sum(m["quality_pass"] for m in found),"sample_rows":sum(m["rows"] for m in found),
            "training_ready":phase=="dynamic" and len(found)==len(jobs(config,phase)) and all(m["quality_pass"] for m in found),
            "pass":len(found)==len(jobs(config,phase)) and all(m["quality_pass"] for m in found),
            "max_saturation_ratio":max((max(m["saturation_ratio_by_joint"]) for m in found),default=0.),
            "max_force_balance_error_nm":max((m["max_force_balance_error_nm"] for m in found),default=0.)}
    write_json(output/phase/"summary.json",result)
    return result
