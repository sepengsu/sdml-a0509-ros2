"""Static scientific figures, with numeric CSV/NPZ sources retained beside them."""
import csv
import json

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import mujoco as mj

from .common import build_baseline, marker_complete, save_npz
from .laws import FrictionState, configure_model, inverse_at, scenarios


def audit_plots(output, config, audit):
    folder=output/"figures"
    folder.mkdir(exist_ok=True)
    rows=audit["joints"]
    x=np.arange(6)
    fig,ax=plt.subplots(figsize=(10,4.8),layout="constrained")
    ax.bar(x-.2,[r["damping_rms_nm"] for r in rows],.4,label="Background damping RMS")
    ax.bar(x+.2,[r["rigid_rms_nm"] for r in rows],.4,label="Rigid-body RMS")
    ax.set(xticks=x,xticklabels=[r["joint"] for r in rows],ylabel="Torque (Nm)",title=f"Accepted source rollouts: {audit['accepted_rollouts']} (selection-conditioned)")
    ax.legend(); ax.grid(axis="y",alpha=.2)
    fig.savefig(folder/"background_damping_vs_rbd.png",dpi=160); plt.close(fig)
    # A causal triangular velocity sweep makes LuGre hysteresis visible.
    times=np.arange(401)*.01
    velocity=np.concatenate((np.linspace(-.2,.2,201),np.linspace(.2,-.2,201)[1:]))
    q=np.array((-.5,-.1,.3,0.,-.2,0.))
    model=build_baseline("bare_p0",output/"audit/models/curve_baseline.xml",config)
    baseline={"name":"baseline","model":"baseline","scale":0.,"level":"none"}
    configure_model(model,baseline,config)
    data=mj.MjData(model)
    base=np.array([inverse_at(model,data,q,np.full(6,v),np.zeros(6),background=config["background_damping_nm_s_rad"])[0] for v in velocity])
    curves=[]; names=[]
    selected_scenarios=scenarios(config,False)
    for scenario in selected_scenarios:
        configure_model(model,scenario,config); data=mj.MjData(model)
        state=FrictionState(scenario,config); values=[]
        for i,(t,v) in enumerate(zip(times,velocity)):
            ext=state.advance(float(t),np.full(6,v))
            native,_=inverse_at(model,data,q,np.full(6,v),np.zeros(6),ext,config["background_damping_nm_s_rad"])
            values.append(native-ext-base[i])
        curves.append(values); names.append(scenario["name"])
    curves=np.asarray(curves)
    save_npz(folder/"friction_velocity_curves.npz",{"time_s":times,"velocity_rad_s":velocity,"required_extra_torque_nm":curves})
    (folder/"friction_velocity_curves.json").write_text(json.dumps({"scenarios":selected_scenarios,
        "sweep_dt_s":.01,"lugre_initial_z":[0.]*6,"curve_type":"offline same-state velocity sweep, not a rollout"},indent=2),encoding="utf-8")
    fig,axes=plt.subplots(2,3,figsize=(12,7),layout="constrained")
    for j,ax in enumerate(axes.flat):
        for scenario,y in zip(selected_scenarios,curves):
            if scenario["level"]=="nominal": ax.plot(velocity,y[:,j],label=scenario["name"])
        ax.set(title=f"J{j+1}",xlabel="Velocity (rad/s)",ylabel="Additional required torque (Nm)")
        ax.grid(alpha=.2)
    axes.flat[0].legend(fontsize=7)
    fig.suptitle("Native Coulomb / viscous and external nonlinear friction; LuGre follows sweep history")
    fig.savefig(folder/"friction_velocity_curves.png",dpi=160); plt.close(fig)
    for family in config["legacy"]["models"]:
        selected=[(s,y) for s,y in zip(selected_scenarios,curves) if s["model"]==family]
        if not selected: continue
        fig,axes=plt.subplots(2,3,figsize=(11,6),layout="constrained")
        for j,ax in enumerate(axes.flat):
            for s,y in selected: ax.plot(velocity,y[:,j],label=s["level"])
            ax.set(title=f"J{j+1}",xlabel="Velocity (rad/s)",ylabel="Additional required torque (Nm)"); ax.grid(alpha=.2)
        axes.flat[0].legend(fontsize=8); fig.suptitle(f"{family}: all three strengths (LuGre uses causal sweep history)")
        fig.savefig(folder/f"velocity_{family}_three_strengths.png",dpi=150); plt.close(fig)


def report(output, config, run_identity):
    figure_dir=output/"figures"; figure_dir.mkdir(exist_ok=True)
    offline=[]
    for path in sorted((output/"offline").rglob("metadata.json")):
        m=marker_complete(path.parent,run_identity)
        if m: offline.append(m)
    if offline:
        errors=np.stack([m["scenario_joint_rmse_vs_legacy_nm"] for m in offline])
        rms=np.sqrt(np.mean(errors**2,axis=0))
        names=[s["name"] for s in scenarios(config,False)]
        with (figure_dir/"offline_vs_legacy.csv").open("w",newline="",encoding="utf-8") as stream:
            w=csv.writer(stream); w.writerow(["scenario",*[f"J{j+1}_rmse_nm" for j in range(6)]])
            for name,value in zip(names,rms): w.writerow([name,*value])
        fig,axes=plt.subplots(2,3,figsize=(13,7),layout="constrained")
        for j,ax in enumerate(axes.flat):
            ax.bar(np.arange(len(names)),rms[:,j]); ax.set(title=f"J{j+1}",ylabel="Offline vs legacy RMSE (Nm)")
            ax.set_xticks(np.arange(len(names)),names,rotation=90,fontsize=6); ax.grid(axis="y",alpha=.2)
        fig.savefig(figure_dir/"offline_vs_legacy.png",dpi=160); plt.close(fig)
    records=[]
    for phase in ("pilot","dynamic"):
        for path in sorted((output/phase).rglob("metadata.json")):
            meta=marker_complete(path.parent,run_identity)
            if meta: records.append((path.parent,meta))
    if not records:
        return
    rows=[]
    for folder,meta in records:
        row={"phase":meta["job"]["phase"],"condition":meta["job"]["condition"],"trajectory":meta["job"]["trajectory"],
             "scenario":meta["job"]["scenario"]["name"],"quality_pass":meta["quality_pass"],
             "max_saturation_ratio":max(meta["saturation_ratio_by_joint"]),"max_tracking_error_rad":meta["max_tracking_error_rad"],
             "max_force_balance_error_nm":meta["max_force_balance_error_nm"]}
        with np.load(folder/"samples.npz",allow_pickle=False) as d:
            for j in range(6):
                row[f"J{j+1}_tracking_rmse_rad"]=meta["tracking_rmse_rad"][j]
                row[f"J{j+1}_added_torque_rms_nm"]=float(np.sqrt(np.mean(d["qfrc_additional_friction"][:,j]**2)))
            baseline_folder=folder.with_name("baseline")
            if marker_complete(baseline_folder,run_identity):
                with np.load(baseline_folder/"samples.npz",allow_pickle=False) as base:
                    if not np.array_equal(d["time_s"],base["time_s"]): raise ValueError("Scenario target time axes differ")
                    row["motor_torque_change_rms_nm"]=float(np.sqrt(np.mean((d["tau_actuator_nm"]-base["tau_actuator_nm"])**2)))
                    row["actual_state_change_rms_rad"]=float(np.sqrt(np.mean((d["q"]-base["q"])**2)))
        rows.append(row)
    columns=sorted(set().union(*(r.keys() for r in rows)))
    with (figure_dir/"dynamic_scenario_summary.csv").open("w",newline="",encoding="utf-8") as stream:
        w=csv.DictWriter(stream,fieldnames=columns); w.writeheader(); w.writerows(rows)
    for phase in ("pilot","dynamic"):
        plot_rows=[r for r in rows if r["phase"]==phase]
        if not plot_rows: continue
        names=[s["name"] for s in scenarios(config)]
        fig,axes=plt.subplots(2,1,figsize=(12,7),layout="constrained")
        for ax,key,label in [(axes[0],"motor_torque_change_rms_nm","Actuator change vs baseline (Nm RMS)"),
                             (axes[1],"max_tracking_error_rad","Max tracking error (rad)")]:
            values=[np.mean([r.get(key,0.) for r in plot_rows if r["scenario"]==name]) if any(r["scenario"]==name for r in plot_rows) else np.nan for name in names]
            ax.bar(np.arange(len(names)),values); ax.set(ylabel=label); ax.grid(axis="y",alpha=.2)
            ax.set_xticks(np.arange(len(names)),names,rotation=75,fontsize=7)
        fig.suptitle(f"{phase}: Dynamic changes include changed states; isolate friction with Offline comparisons")
        fig.savefig(figure_dir/f"{phase}_torque_and_tracking.png",dpi=160); plt.close(fig)
        phase_records=[(p,m) for p,m in records if m["job"]["phase"]==phase]
        nonlinear=next(((p,m) for p,m in phase_records if m["job"]["scenario"]["model"]=="stribeck"),None)
        selected=nonlinear or next(((p,m) for p,m in phase_records if m["job"]["scenario"]["model"]!="baseline"),phase_records[0])
        with np.load(selected[0]/"samples.npz",allow_pickle=False) as d:
            step=max(1,len(d["time_s"])//3000); idx=slice(None,None,step)
            fig,axes=plt.subplots(2,3,figsize=(12,7),layout="constrained")
            for j,ax in enumerate(axes.flat):
                ax.plot(d["time_s"][idx],d["tau_actuator_nm"][idx,j],label="Actuator")
                ax.plot(d["time_s"][idx],d["tau_rbd_nm"][idx,j],label="Rigid-body")
                ax.plot(d["time_s"][idx],-d["qfrc_additional_friction"][idx,j],label="Extra friction compensation")
                ax.set(title=f"J{j+1}",xlabel="Simulation time (s)",ylabel="Torque (Nm)"); ax.grid(alpha=.2)
            axes.flat[0].legend(fontsize=7)
            fig.suptitle(f"{phase}: {selected[1]['job']['condition']} / {selected[1]['job']['trajectory']} / {selected[1]['job']['scenario']['name']}")
            fig.savefig(figure_dir/f"{phase}_force_components.png",dpi=160); plt.close(fig)
