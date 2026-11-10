"""Reproducible T16 and T1–T16 research figures from generated references.

The legacy chirp is read from its archived coefficients only for comparison.
Figure 3 Cartesian T1–T3 paths are reference TCP paths, not IK-validated runs.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from .simulation_executor import MujocoObserver
from .t16_fourier import load_spec
from .trajectory_generator import (EXCITATIONS, TRAJECTORY_IDS, TrajectoryConfig,
                                   _window, generate, is_joint_trajectory)


def _legacy_path():
    local = Path(__file__).resolve().parents[1] / "config" / "t16_legacy_chirp.json"
    if local.is_file():
        return local
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("a0509_dynamics_experiment")) / "config" / local.name


def legacy_states():
    spec = json.loads(_legacy_path().read_text(encoding="utf-8"))
    t = np.linspace(0, spec["duration_s"], round(spec["duration_s"] * 100) + 1)
    q = np.tile(spec["home_rad"], (len(t), 1)).astype(float)
    dq = np.zeros_like(q)
    ddq = np.zeros_like(q)
    for joint, amplitude_deg, f0, f1, phase, start, end in spec["components"]:
        amplitude = math.radians(amplitude_deg)
        chirp_rate = (f1 - f0) / (end - start)
        for index, time_s in enumerate(t):
            envelope, de, dde = _window(float(time_s), start, end)
            elapsed = time_s - start
            angle = 2 * np.pi * (f0 * elapsed + .5 * chirp_rate * elapsed**2) + phase
            omega = 2 * np.pi * (f0 + chirp_rate * elapsed)
            alpha = 2 * np.pi * chirp_rate
            sine, cosine = math.sin(angle), math.cos(angle)
            q[index, joint] += amplitude * envelope * sine
            dq[index, joint] += amplitude * (de * sine + envelope * omega * cosine)
            ddq[index, joint] += amplitude * (dde * sine + 2 * de * omega * cosine +
                                                envelope * (alpha * cosine - omega**2 * sine))
    return t, q, dq, ddq


def _joint_arrays(plan):
    motion = [p for p in plan.points if p.phase.startswith("motion_")]
    return (np.asarray([p.time_s - plan.config.pre_hold_seconds for p in motion]),
            np.asarray([p.q_rad for p in motion]),
            np.asarray([p.dq_rad_s for p in motion]),
            np.asarray([p.ddq_rad_s2 for p in motion]))


def _tcp(observer, q, max_points=400):
    step = max(1, len(q) // max_points)
    indices = list(range(0, len(q), step))
    if indices[-1] != len(q)-1:
        indices.append(len(q)-1)
    return np.asarray([observer.tcp_mm(q[i]) for i in indices])


def _set_global_axes(ax, paths):
    merged = np.concatenate(paths)
    center = (merged.min(axis=0) + merged.max(axis=0)) / 2
    radius = max(40.0, float(np.ptp(merged, axis=0).max()) / 2 * 1.08)
    for setter, c in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), center):
        setter(c - radius, c + radius)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")


def _interactive_html(path: Path, xyz: np.ndarray, links: np.ndarray):
    # Self-contained interactive canvas: drag to rotate, wheel to zoom.
    trajectory = json.dumps(xyz[::max(1, len(xyz)//500)].round(3).tolist())
    robot = json.dumps(links.round(3).tolist())
    html = '''<!doctype html><html><head><meta charset="utf-8"><title>T16 AG145 TCP</title>
<style>body{font:15px system-ui;margin:18px;background:#f8fafc;color:#172033}canvas{width:100%;height:75vh;border:1px solid #ccd5df;background:white;touch-action:none}p{margin:8px 0}</style></head><body>
<h1>T16 Optimized Fourier Excitation — AG145 TCP</h1><p>Drag to rotate; mouse wheel to zoom. Blue: planned TCP; gray: A0509 links; green/red: start/end. Coordinates in mm.</p><button onclick="setFocus(true)">TCP detail</button> <button onclick="setFocus(false)">Full robot</button>
<canvas id="v"></canvas><script>
const trajectory=TRAJECTORY, links=LINKS, canvas=document.getElementById('v'),ctx=canvas.getContext('2d');
let yaw=-0.7,pitch=0.4,zoom=1,drag=false,last=[0,0],focus=true;
const all=trajectory.concat(links);
function extent(points){let lo=[0,1,2].map(i=>Math.min(...points.map(p=>p[i]))),hi=[0,1,2].map(i=>Math.max(...points.map(p=>p[i])));return [lo.map((x,i)=>(x+hi[i])/2),Math.max(40,...lo.map((x,i)=>hi[i]-x))]}
let [mid,span]=extent(trajectory);
function setFocus(value){focus=value;[mid,span]=extent(focus?trajectory:all);zoom=1;draw()}
function project(p){let x=p[0]-mid[0],y=p[1]-mid[1],z=p[2]-mid[2],cy=Math.cos(yaw),sy=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch),u=cy*x-sy*y,v=sp*(sy*x+cy*y)+cp*z;return [canvas.width/2+u*zoom*canvas.width/(span*1.7),canvas.height/2-v*zoom*canvas.height/(span*1.7)]}
function line(points,color,width){ctx.strokeStyle=color;ctx.lineWidth=width;ctx.beginPath();points.forEach((p,i)=>{let [x,y]=project(p);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke()}
function dot(p,color,r){let [x,y]=project(p);ctx.fillStyle=color;ctx.beginPath();ctx.arc(x,y,r,0,2*Math.PI);ctx.fill()}
function draw(){canvas.width=canvas.clientWidth*devicePixelRatio;canvas.height=canvas.clientHeight*devicePixelRatio;ctx.clearRect(0,0,canvas.width,canvas.height);if(!focus){line(links,'#777',3);links.forEach(p=>dot(p,'#777',4))}line(trajectory,'#2274c9',3);dot(trajectory[0],'#198754',8);dot(trajectory[trajectory.length-1],'#d33',7);for(let i=20;i<trajectory.length-1;i+=Math.max(20,Math.floor(trajectory.length/10))){let a=project(trajectory[i-1]),b=project(trajectory[i+1]),ang=Math.atan2(b[1]-a[1],b[0]-a[0]);ctx.save();ctx.translate(...project(trajectory[i]));ctx.rotate(ang);ctx.fillStyle='#2274c9';ctx.beginPath();ctx.moveTo(8,0);ctx.lineTo(-6,-5);ctx.lineTo(-6,5);ctx.fill();ctx.restore()}}
canvas.onpointerdown=e=>{drag=true;last=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)};
canvas.onpointermove=e=>{if(drag){yaw+=(e.clientX-last[0])*.008;pitch=Math.max(-1.5,Math.min(1.5,pitch+(e.clientY-last[1])*.008));last=[e.clientX,e.clientY];draw()}};
canvas.onpointerup=()=>drag=false;canvas.onwheel=e=>{e.preventDefault();zoom=Math.max(.25,Math.min(5,zoom*(e.deltaY<0?1.1:.9)));draw()};window.onresize=draw;draw();
</script></body></html>'''
    path.write_text(html.replace("TRAJECTORY", trajectory).replace("LINKS", robot), encoding="utf-8")


def save_t16_figures(folder: Path, scene: Path, plan=None) -> dict:
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    spec = load_spec()
    if plan is None:
        plan = generate(TrajectoryConfig("T16"))
    observer = MujocoObserver(scene, tcp_site="ag145_tcp")
    t, q, dq, ddq = _joint_arrays(plan)
    old_t, old_q, old_dq, old_ddq = legacy_states()

    fig, axes = plt.subplots(6, 3, figsize=(16, 15), sharex="col")
    for j in range(6):
        for col, (new, old, label, scale) in enumerate(((q, old_q, "q (deg)", 180/math.pi),
                                                        (dq, old_dq, "dq (deg/s)", 180/math.pi),
                                                        (ddq, old_ddq, "ddq (deg/s²)", 180/math.pi))):
            ax = axes[j, col]
            ax.plot(t, new[:, j] * scale, color="#1565c0", lw=1.4, label="Fourier 10 s")
            ax.plot(old_t, old[:, j] * scale, color="#ef6c00", lw=1, alpha=.7, label="legacy chirp 18 s")
            ax.set_ylabel(f"J{j+1} {label}")
            ax.grid(alpha=.25)
            if j == 0:
                ax.legend(fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("Time in motion (s)")
    fig.suptitle("T16 joint excitation: optimized Fourier vs archived chirp")
    fig.tight_layout()
    fig.savefig(folder / "t16_joint_profiles.png", dpi=170)
    plt.close(fig)

    xyz = _tcp(observer, q, 1000)
    links = np.asarray(observer.links_mm(q[0]))
    fig = plt.figure(figsize=(14, 7))
    ax = fig.add_subplot(121, projection="3d")
    ax.plot(*xyz.T, color="#1565c0", lw=1.8, label="planned AG145 TCP")
    ax.plot(*links.T, color="0.55", marker="o", lw=1, label="A0509 links at start")
    ax.scatter(*xyz[0], color="green", s=65, label="start")
    ax.scatter(*xyz[-1], color="red", marker="x", s=75, label="end")
    for i in range(80, len(xyz)-1, max(80, len(xyz)//8)):
        d = xyz[i+1] - xyz[i-1]
        ax.quiver(*xyz[i], *d, normalize=True, length=8, color="#1565c0")
    _set_global_axes(ax, [xyz, links])
    ax.set_title("A0509 + AG145 at start", pad=15)
    ax.legend(fontsize=8)
    detail = fig.add_subplot(122, projection="3d")
    detail.plot(*xyz.T, color="#1565c0", lw=1.8)
    detail.scatter(*xyz[0], color="green", s=65, label="start")
    detail.scatter(*xyz[-1], color="red", marker="x", s=75, label="end")
    for i in range(80, len(xyz)-1, max(80, len(xyz)//8)):
        d = xyz[i+1] - xyz[i-1]
        detail.quiver(*xyz[i], *d, normalize=True, length=5, color="#1565c0")
    _set_global_axes(detail, [xyz])
    detail.set_title("T16 TCP path detail", pad=15)
    detail.legend(fontsize=8)
    fig.subplots_adjust(left=.02, right=.98, bottom=.05, top=.89, wspace=.02)
    fig.savefig(folder / "t16_tcp_3d.png", dpi=170)
    plt.close(fig)
    _interactive_html(folder / "t16_tcp_3d.html", xyz, links)

    paths = {}
    reference = np.asarray(observer.tcp_mm(tuple(spec["home_rad"])))
    for identifier in TRAJECTORY_IDS:
        if identifier in ("T1", "T2", "T3"):
            item = generate(TrajectoryConfig(identifier, reference_tcp_mm=tuple(reference)))
            path = np.asarray([point.xyz_mm for point in item.points])
            path = path[::max(1, len(path)//250)]
        else:
            item = plan if identifier == "T16" else generate(TrajectoryConfig(identifier))
            states_q = np.asarray([p.q_rad for p in item.points])
            path = _tcp(observer, states_q, 250)
        paths[identifier] = path
    fig = plt.figure(figsize=(18, 19))
    for index, identifier in enumerate(TRAJECTORY_IDS, 1):
        ax = fig.add_subplot(4, 4, index, projection="3d")
        path = paths[identifier]
        ax.plot(*path.T, color="#1565c0", lw=1.15)
        ax.scatter(*path[0], color="green", s=15)
        ax.scatter(*path[-1], color="red", marker="x", s=18)
        _set_global_axes(ax, list(paths.values()))
        name = EXCITATIONS[identifier]["name"] if identifier in EXCITATIONS else {
            "T1": "Linear Reciprocation", "T2": "Spatial Diagonal",
            "T3": "3D Box", "T4": "Joint Excitation"}[identifier]
        ax.set_title(f"{identifier} {name}", fontsize=9)
        ax.tick_params(labelsize=6)
    fig.suptitle("A0509 + AG145 TCP trajectory gallery (T1–T3: reference paths)", y=.995)
    fig.subplots_adjust(left=.02, right=.98, bottom=.03, top=.95, wspace=.14, hspace=.35)
    fig.savefig(folder / "t1_t16_tcp_gallery.png", dpi=170)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    before, after = spec["initial"], spec["optimized"]
    axes[0, 0].bar(["seeded baseline", "optimized"],
                   [before["condition_number"], after["condition_number"]],
                   color=["#ef6c00", "#1565c0"])
    axes[0, 0].set_yscale("log"); axes[0, 0].set_ylabel("Condition number (fixed normalized proxy)")
    axes[0, 0].set_title("Identification objective")
    for label, info, color in (("baseline", before, "#ef6c00"), ("optimized", after, "#1565c0")):
        axes[0, 1].semilogy(np.arange(1, len(info["singular_values"])+1),
                            info["singular_values"], marker="o", ms=3, label=label, color=color)
    axes[0, 1].set_xlabel("Singular value index"); axes[0, 1].set_ylabel("Normalized singular value")
    axes[0, 1].legend(); axes[0, 1].set_title("Selected identifiable parameter subset")
    a, b = np.asarray(spec["a_rad_s"]), np.asarray(spec["b_rad_s"])
    harmonics = np.hypot(a, b)
    for k in range(3):
        axes[1, 0].bar(np.arange(6) + (k-1)*.23, harmonics[:, k], width=.23,
                       label=f"{(k+1)*.1:.1f} Hz")
    axes[1, 0].set_xticks(np.arange(6), [f"J{i}" for i in range(1,7)])
    axes[1, 0].set_ylabel("Velocity harmonic amplitude (rad/s)")
    axes[1, 0].set_title("Optimized independent joint spectra"); axes[1, 0].legend()
    t10 = generate(TrajectoryConfig("T10"))
    _, q10, dq10, ddq10 = _joint_arrays(t10)
    ranges = []
    for name, values in (("T10", (q10, dq10, ddq10)),
                         ("legacy T16", (old_q, old_dq, old_ddq)),
                         ("Fourier T16", (q, dq, ddq))):
        ranges.append([name, *[f"{np.sum(np.ptp(array, axis=0)):.4f}" for array in values]])
    axes[1, 1].axis("off")
    axes[1, 1].table(cellText=ranges, colLabels=["trajectory", "Σq range (rad)",
                                                   "Σdq range (rad/s)", "Σddq range (rad/s²)"],
                     loc="center", cellLoc="center")
    axes[1, 1].set_title("Six-joint state-space span")
    fig.suptitle("T16 parameter-sensitivity proxy optimization (not paper regressor)")
    fig.tight_layout()
    fig.savefig(folder / "t16_optimization_results.png", dpi=170)
    plt.close(fig)

    fig = plt.figure(figsize=(12, 5))
    ax3 = fig.add_subplot(121, projection="3d")
    ax3.plot(*paths["T10"].T, label="T10 multi-sine", color="#ef6c00")
    ax3.plot(*paths["T16"].T, label="T16 Fourier", color="#1565c0")
    _set_global_axes(ax3, [paths["T10"], paths["T16"]]); ax3.legend()
    ax3.set_title("TCP paths")
    axf = fig.add_subplot(122)
    for name, time, velocity, color in (("T10", _joint_arrays(t10)[0], dq10, "#ef6c00"),
                                        ("T16", t, dq, "#1565c0")):
        dt = float(np.median(np.diff(time)))
        spectrum = np.mean(np.abs(np.fft.rfft(velocity, axis=0)), axis=1) / len(time)
        frequency = np.fft.rfftfreq(len(time), dt)
        axf.plot(frequency, spectrum, label=name, color=color)
    axf.set_xlim(0, .6); axf.set_xlabel("Frequency (Hz)")
    axf.set_ylabel("Mean |FFT(dq)| across joints (rad/s)")
    axf.grid(alpha=.25); axf.legend(); axf.set_title("Joint velocity spectra")
    fig.tight_layout()
    fig.savefig(folder / "t10_t16_comparison.png", dpi=170)
    plt.close(fig)

    summary = {"scene": str(scene), "AG145": "fixed approximate scene",
               "gallery_paths": list(paths), "condition_before": before["condition_number"],
               "condition_after": after["condition_number"], "effective_rank": after["effective_rank"],
               "t10_legacy_t16_fourier_ranges": ranges,
               "caveat": "T1-T3 gallery paths are Cartesian references; figure is not evidence of executable IK."}
    (folder / "t16_figure_metadata.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary
