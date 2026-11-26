"""Compare TCC1 nominal inertials on one archived T5 joint trajectory."""

import argparse
import csv
from pathlib import Path
from tempfile import TemporaryDirectory

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from a0509_dynamics_experiment.ag145_model import build_ag145_scene
from mdpg_1s1_description.model import build_mdpg_scene
from tcc1_tool_changer_description.model import write_scene


ROOT = Path(__file__).resolve().parents[1]
TRIAL = ROOT / "data/grid_16_trajectories/simulation/T5/ag145_p0/trial_001/dynamics.csv"
OUTPUT = ROOT / "output/tcc1_nominal"
CHANNELS = ("inverse", "gravity", "coriolis", "inertial")
GRIPPERS = {
    "ag145": ("AG145", "ag145_tcp"),
    "mdpg_1s1": ("mDPG", "mdpg_tcp"),
}


def read_t5(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) < 2 or any(row["trajectory_id"] != "T5" or row["mode"] != "simulation"
                            or float(row["payload_mass"]) != 0.0 for row in rows):
        raise ValueError("Expected one archived T5, 0 kg simulation")
    times = np.array([float(row["ros_time_s"]) for row in rows])
    states = np.array([[[float(row[f"{channel}{i}"]) for i in range(1, 7)]
                        for channel in ("q", "dq", "ddq")] for row in rows])
    if not np.isfinite(states).all() or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError("Invalid T5 timestamps or joint states")
    return times - times[0], states


def tool_properties(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[float, np.ndarray, np.ndarray]:
    flange = model.body("link_6").id
    descendants = {flange}
    for body in range(flange + 1, model.nbody):
        if model.body_parentid[body] in descendants:
            descendants.add(body)
    ids = sorted(descendants - {flange})
    mass = model.body_mass[ids]
    rotation = data.xmat[flange].reshape(3, 3)
    coms = (data.xipos[ids] - data.xpos[flange]) @ rotation
    com = np.average(coms, weights=mass, axis=0)
    inertia = np.zeros((3, 3))
    for body, submass, subcom in zip(ids, mass, coms):
        axes = rotation.T @ data.ximat[body].reshape(3, 3)
        offset = subcom - com
        inertia += axes @ np.diag(model.body_inertia[body]) @ axes.T
        inertia += submass * ((offset @ offset) * np.eye(3) - np.outer(offset, offset))
    return float(mass.sum()), com, inertia


def evaluate(scene: Path, tcp_site: str, states: np.ndarray) -> dict:
    model = mujoco.MjModel.from_xml_path(str(scene))
    if (model.nq, model.nv) != (6, 6) or [
        model.joint(i).name for i in range(model.njnt)
    ] != [f"joint_{i}" for i in range(1, 7)]:
        raise ValueError(f"Unexpected joint order in {scene}")
    data, gravity_data = mujoco.MjData(model), mujoco.MjData(model)
    data.qpos[:] = states[0, 0]
    mujoco.mj_forward(model, data)
    mass, com, inertia = tool_properties(model, data)
    torque = {name: np.empty((len(states), 6)) for name in CHANNELS}
    tcp = np.empty((len(states), 3))
    contacts = 0
    for index, (q, dq, ddq) in enumerate(states):
        data.qpos[:], data.qvel[:] = q, dq
        mujoco.mj_forward(model, data)
        tcp[index] = data.site(tcp_site).xpos
        contacts += int(data.ncon > 0)
        gravity_data.qpos[:] = q
        mujoco.mj_forward(model, gravity_data)
        torque["gravity"][index] = gravity_data.qfrc_bias
        torque["coriolis"][index] = data.qfrc_bias - gravity_data.qfrc_bias
        mujoco.mj_mulM(model, data, torque["inertial"][index], ddq)
        data.qacc[:] = ddq
        mujoco.mj_inverse(model, data)
        torque["inverse"][index] = data.qfrc_inverse
    return {"robot_mass": float(model.body_mass.sum()), "tool_mass": mass, "com": com, "inertia": inertia,
            "tcp": tcp, "torque": torque, "contact_samples": contacts}


def plot(times: np.ndarray, comparisons: dict, output: Path) -> None:
    fig, axes = plt.subplots(2, 2, figsize=(12, 6.8), sharey="row", layout="constrained")
    fig.suptitle("A0509 T5 · Effect of the nominal TCC1 on joint torque · 0 kg payload")
    colors = ("#415a77", "#b84d37", "#177a65", "#7660a5", "#cd901d", "#657984")
    for column, (name, (off, on)) in enumerate(comparisons.items()):
        delta = {k: on["torque"][k] - off["torque"][k] for k in CHANNELS}
        ax = axes[0, column]
        ax.set_title(name, pad=31)
        for joint, color in enumerate(colors, start=1):
            ax.plot(times, delta["inverse"][:, joint - 1], color=color, label=f"J{joint}", lw=1.35)
        ax.axhline(0, color="#606060", lw=0.6)
        ax.set(xlabel="Time (s)", ylabel="Inverse torque change (N·m)")
        ax.legend(ncol=6, loc="lower center", bbox_to_anchor=(0.5, 1.0), frameon=False)
        ax = axes[1, column]
        joints = np.arange(1, 7)
        for i, (key, label) in enumerate(zip(CHANNELS[1:], ("Gravity", "Coriolis / centrifugal", "Inertial"))):
            ax.bar(joints + (i - 1) * 0.23, np.max(np.abs(delta[key]), axis=0), width=0.21,
                   color=colors[i], label=label)
        ax.set(xticks=joints, xticklabels=[f"J{i}" for i in joints], ylabel="Peak absolute change (N·m)")
        ax.legend(frameon=False)
        for panel in axes[:, column]:
            panel.grid(axis="y", alpha=0.22)
            panel.set_axisbelow(True)
    fig.savefig(output, dpi=160)
    plt.close(fig)


def write_report(path: Path, trial: Path, times: np.ndarray, comparisons: dict, config: Path | None) -> None:
    try:
        source = trial.resolve().relative_to(ROOT).as_posix()
    except ValueError:
        source = trial.resolve().as_posix()
    lines = [
        "# TCC1 장착 전후 동역학 비교", "",
        f"기존 T5 기록 `{source}`의 q, dq, ddq {len(times):,}개를 네 모델에 동일하게 입력했다. "
        f"기록 구간은 {times[-1]:.2f}초이며 페이로드는 0 kg이다.", "",
        f"MuJoCo {mujoco.__version__} / 설정 `{config or '패키지 기본 tcc1_parameters.yaml'}`. "
        "OFF와 ON은 같은 그리퍼 물성 모델을 사용한다. mDPG OFF는 기존 동심 장착 및 box_approx를 사용한다.", "",
        "질량은 로봇 전체와 J6 하위 조립체 합으로 표기한다. CoM과 관성은 J6 좌표축 기준이며, "
        "관성의 기준점은 각 조립체 CoM이다. 관성 성분 순서는 Ixx, Iyy, Izz, Ixy, Ixz, Iyz이다.", "",
        "토크 변화는 ON − OFF이다. 역동역학은 MuJoCo inverse force, 중력은 속도 0에서의 bias, "
        "코리올리·원심력은 bias − 중력, 관성 토크는 M(q)·ddq로 계산했다.", "",
        "![관절 토크 변화](T5_torque_delta.png)", "",
    ]
    for name, (off, on) in comparisons.items():
        delta = on["torque"]["inverse"] - off["torque"]["inverse"]
        tcp_mm = np.linalg.norm(on["tcp"] - off["tcp"], axis=1) * 1000
        lines += [f"## {name}", "",
                  "| 구성 | 로봇 총질량 (kg) | J6 하위 질량 (kg) | J6 하위 CoM (mm) | J6 하위 관성 (kg·m²) |",
                  "| --- | ---: | ---: | --- | --- |"]
        for status, result in (("OFF", off), ("ON", on)):
            com = ", ".join(f"{value:.3f}" for value in result["com"] * 1000)
            matrix = result["inertia"]
            inertia = ", ".join(f"{value:.6e}" for value in matrix[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]])
            lines.append(f"| {status} | {result['robot_mass']:.3f} | {result['tool_mass']:.3f} | {com} | {inertia} |")
        lines += ["", f"TCP 위치 차이는 {tcp_mm.min():.3f}–{tcp_mm.max():.3f} mm이다. "
                  f"접촉이 보고된 샘플은 OFF {off['contact_samples']}개, ON {on['contact_samples']}개이다.", "",
                  "| 관절 | OFF 최대 절댓값 | ON 최대 절댓값 | Δ 최대 절댓값 | Δ RMS |",
                  "| --- | ---: | ---: | ---: | ---: |"]
        for i in range(6):
            values = (np.max(np.abs(off["torque"]["inverse"][:, i])),
                      np.max(np.abs(on["torque"]["inverse"][:, i])),
                      np.max(np.abs(delta[:, i])), np.sqrt(np.mean(delta[:, i] ** 2)))
            lines.append(f"| J{i+1} | " + " | ".join(f"{v:.4f}" for v in values) + " |")
        lines += ["", "토크 단위는 N·m이다.", ""]
    lines += ["이 결과는 기록을 사용한 역동역학 비교다. 실제 장비와의 검증 및 닫힌 루프 추종 시험은 별도 단계다.", ""]
    path.write_text("\n".join(lines), encoding="utf-8")


def compare(trial: Path, output: Path, config: Path | None) -> None:
    times, states = read_t5(trial)
    comparisons = {}
    with TemporaryDirectory(prefix="tcc1_dynamics_") as directory:
        scratch = Path(directory)
        for gripper, (name, tcp) in GRIPPERS.items():
            base = scratch / f"{gripper}_off.xml"
            if gripper == "ag145":
                build_ag145_scene(base)
            else:
                build_mdpg_scene(base, (0, 0, 0), (0, 0, 0), dynamics_model="box_approx")
            mounted = write_scene(base, scratch / f"{gripper}_on.xml", gripper, tool_changer_type="tcc1",
                                  mode="dynamics", config_path=config)
            comparisons[name] = evaluate(base, tcp, states), evaluate(mounted, tcp, states)
    output.mkdir(parents=True, exist_ok=True)
    plot(times, comparisons, output / "T5_torque_delta.png")
    write_report(output / "T5_dynamics_report.md", trial, times, comparisons, config)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trial", type=Path, default=TRIAL)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    compare(args.trial, args.output, args.config)
