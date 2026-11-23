import argparse
import json
from pathlib import Path
import shutil

from .common import (CONDITIONS, PACKAGE, available_ram, identity, load_config,
                     marker_complete, run_lock, run_worker, write_json)
from .laws import parameters, scenarios


def require_report(path, run_identity, flag):
    report=json.loads(path.read_text(encoding="utf-8")) if path.is_file() else {}
    if report.get("identity")!=run_identity or not report.get(flag):
        raise RuntimeError(f"Required validated stage missing: {path}. Run preceding stages first.")


def main(argv=None):
    p=argparse.ArgumentParser(description="A0509 MuJoCo simulation v4: T1-T16 with 15 engine-applied friction scenarios")
    p.add_argument("--mode",choices=("plan","analyze","offline","pilot","collect","report","status"),default="plan")
    p.add_argument("--config",type=Path,default=PACKAGE/"config.json")
    p.add_argument("--source",type=Path,required=True)
    p.add_argument("--legacy-speed-root",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--conditions",nargs="+",choices=CONDITIONS)
    p.add_argument("--tool",choices=("none","ag145","mdpg_1s1"),
                   help="Collect filter for the selected tool with zero payload; use --conditions for payload cases")
    p.add_argument("--trajectories",nargs="+",choices=[f"T{i}" for i in range(1,17)])
    p.add_argument("--max-jobs",type=int)
    p.add_argument("--worker",type=int)
    a=p.parse_args(argv)
    if a.tool:
        if a.conditions:
            p.error("Choose --tool or --conditions")
        if a.mode not in ("collect","status","plan"):
            p.error("--tool filters collect/status/plan only; validation stages cover the full source campaign")
        a.conditions=[{"none":"bare_p0","ag145":"ag145_p0","mdpg_1s1":"mdpg_p0"}[a.tool]]
    config=load_config(a.config)
    source,output=a.source.resolve(),a.output.resolve()
    old=a.legacy_speed_root.resolve()
    if (source.is_relative_to(output) or output.is_relative_to(source)
            or old.is_relative_to(output) or output.is_relative_to(old)):
        p.error("New output must be separate from both original datasets")
    if a.max_jobs is not None and a.max_jobs<1: p.error("MaxJobs must be positive")
    run_identity,signature=identity(config,source)
    print(f"MuJoCo friction simulation v4 | {a.mode} | {len(scenarios(config,False))} scenarios + baseline | {output}",flush=True)
    from . import dynamic,offline
    canonical=dynamic.jobs(config,"pilot" if a.mode=="pilot" else "dynamic")
    if a.mode=="plan":
        count=sum((not a.conditions or j["condition"] in a.conditions) and
                  (not a.trajectories or j["trajectory"] in a.trajectories) for j in canonical)
        print(f"Full Dynamic: {len(canonical)} executions; selected scope {count}; one worker, fixed speed {config['simulation']['speed_scale']}x; original damping retained.",flush=True)
        return 0
    if a.worker is not None:
        if a.mode=="offline": offline.condition_worker(CONDITIONS[a.worker],source,output,config,run_identity)
        else: dynamic.run_trial(canonical[a.worker],output,config,run_identity)
        return 0
    if a.mode=="status":
        for name in ("audit/legacy_reproduction.json","offline/summary.json","pilot/summary.json","dynamic/summary.json"):
            path=output/name
            if path.is_file():
                record=json.loads(path.read_text(encoding="utf-8"))
                if record.get("identity") != run_identity:
                    print(name,"STALE: stored code/config/source identity differs",flush=True)
                else:
                    print(name,{k:record[k] for k in ("verified_chunks","lugre_verified","all_scenario_labels_passed","completed","planned","quality_passed","pass","training_ready") if k in record},flush=True)
        scope=[j for j in canonical if (not a.conditions or j["condition"] in a.conditions) and
               (not a.trajectories or j["trajectory"] in a.trajectories)]
        count=sum(marker_complete(dynamic.folder_for(output,j),run_identity) is not None for j in scope)
        print(f"Validated Dynamic execution files in selected scope: {count}/{len(scope)}",flush=True)
        return 0
    output.mkdir(parents=True,exist_ok=True)
    with run_lock(output):
        settings=output/"experiment.json"
        if settings.is_file() and json.loads(settings.read_text(encoding="utf-8"))["identity"]!=run_identity:
            raise RuntimeError("Existing version has different code/config/source. Use another ResultsName.")
        write_json(settings,{"identity":run_identity,**signature})
        write_json(output/"friction_parameters.json",{"identity":run_identity,"scenarios":[parameters(s,config) for s in scenarios(config)]})
        if a.mode=="analyze":
            from .audit import analyze
            analyze(source,old,output,config,run_identity)
            return 0
        require_report(output/"audit/legacy_reproduction.json",run_identity,"all_chunks_verified")
        if config["include_lugre_after_reproduction"]:
            require_report(output/"audit/legacy_reproduction.json",run_identity,"lugre_verified")
        if a.mode=="report":
            from .plots import report
            report(output,config,run_identity)
            return 0
        if a.mode in ("pilot","collect"):
            require_report(output/"offline/summary.json",run_identity,"all_scenario_labels_passed")
        if a.mode=="collect": require_report(output/"pilot/summary.json",run_identity,"pass")
        if a.mode=="offline":
            selected=[i for i,c in enumerate(CONDITIONS) if not a.conditions or c in a.conditions]
        else:
            selected=[i for i,j in enumerate(canonical) if (not a.conditions or j["condition"] in a.conditions)
                      and (not a.trajectories or j["trajectory"] in a.trajectories)]
        if a.max_jobs is not None: selected=selected[:a.max_jobs]
        logdir=output/"logs"/a.mode; logdir.mkdir(parents=True,exist_ok=True)
        errors=[]; consecutive=0; peak=0.; processed=0
        progress_path=output/"progress.json"
        if progress_path.is_file():
            previous=json.loads(progress_path.read_text(encoding="utf-8"))
            if previous.get("identity")==run_identity:
                peak=previous.get("peak_worker_mib",0.)
        forward=["--mode",a.mode,"--config",str(a.config.resolve()),"--source",str(source),
                 "--legacy-speed-root",str(old),"--output",str(output)]
        for index in selected:
            if a.mode!="offline" and marker_complete(dynamic.folder_for(output,canonical[index]),run_identity):
                processed+=1; continue
            if available_ram()<2*1024**3 or shutil.disk_usage(output).free<10*1024**3:
                raise RuntimeError("Need 2 GiB free host RAM and 10 GiB free output storage")
            log=logdir/f"worker_{index:04d}.log"
            label=CONDITIONS[index] if a.mode=="offline" else str(canonical[index])
            print(f"[{processed+1}/{len(selected)}] {label}",flush=True)
            code,memory=run_worker(forward,index,log,timeout_s=900)
            peak=max(peak,memory); processed+=1
            if code:
                consecutive+=1; errors.append({"index":index,"exit_code":code,"log":str(log)})
                print(log.read_text(encoding="utf-8",errors="replace")[-2400:],flush=True)
            else:
                consecutive=0
                if a.mode!="offline":
                    m=marker_complete(dynamic.folder_for(output,canonical[index]),run_identity)
                    if m is None: raise RuntimeError("Worker exited without verified completion files")
                    print(f"  quality={m['quality_pass']} saturation={max(m['saturation_ratio_by_joint']):.6g} balance={m['max_force_balance_error_nm']:.3g} Nm",flush=True)
            write_json(output/"progress.json",{"identity":run_identity,"stage":a.mode,"processed":processed,
                       "selected":len(selected),"errors":errors,"peak_worker_mib":peak})
            if consecutive>=3:
                print("Stopped after three consecutive runtime errors.",flush=True); break
        if a.mode=="offline": result=offline.summarize(output,config,run_identity)
        else: result=dynamic.summary(output,config,run_identity,"pilot" if a.mode=="pilot" else "dynamic")
        from .plots import report
        report(output,config,run_identity)
        print(json.dumps(result),flush=True)
        if a.mode=="pilot": return 0 if result["pass"] and not errors else 2
        if a.mode=="offline" and not a.conditions and a.max_jobs is None and not result["all_scenario_labels_passed"]:
            return 2
        if a.mode=="collect" and any(
                not marker_complete(dynamic.folder_for(output,canonical[i]),run_identity).get("quality_pass",False)
                for i in selected if marker_complete(dynamic.folder_for(output,canonical[i]),run_identity)):
            print("Completed diagnostic data include failed quality gates; inspect metadata before training.",flush=True)
            return 2
        return 0 if not errors and processed==len(selected) else 2
