"""Extract the MuJoCo mass matrix at every recorded AG145 state."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np

from .data import JOINTS, REPO_ROOT, TRIAL_IDS, load_trials


def load_scene_model(scene: Path):
    import mujoco

    scene_text = scene.read_text(encoding="utf-8")
    # Trial XMLs were generated in /ros2_ws. Resolve those asset paths against
    # this checkout when preprocessing on Windows or another host.
    if not Path("/ros2_ws").exists():
        scene_text = scene_text.replace("/ros2_ws/", REPO_ROOT.as_posix().rstrip("/") + "/")
    return mujoco.MjModel.from_xml_string(scene_text)


def build_teacher(output: str | Path, stride: int = 5) -> Path:
    import mujoco

    output = Path(output)
    trials = load_trials(stride=stride)
    scene = trials["T1"].folder / "ag145_scene.xml"
    model = load_scene_model(scene)
    if model.nq != JOINTS or model.nv != JOINTS:
        raise ValueError("expected a six-axis MuJoCo scene")
    qpos_addresses = []
    for index in range(1, JOINTS + 1):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{index}")
        if joint_id < 0:
            raise ValueError(f"joint_{index} missing from MuJoCo scene")
        qpos_addresses.append(int(model.jnt_qposadr[joint_id]))
    data = mujoco.MjData(model)
    arrays = {}
    for task, trial in trials.items():
        matrices = np.empty((trial.size, JOINTS, JOINTS), dtype=np.float32)
        dense = np.empty((JOINTS, JOINTS), dtype=np.float64)
        for index, q in enumerate(trial.q):
            data.qpos[:] = model.qpos0
            for address, value in zip(qpos_addresses, q):
                data.qpos[address] = value
            data.qvel[:] = 0
            mujoco.mj_forward(model, data)
            mujoco.mj_fullM(model, dense, data.qM)
            matrices[index] = dense
        if not np.allclose(matrices, matrices.transpose(0, 2, 1), atol=1e-5):
            raise ValueError(f"nonsymmetric mass matrix for {task}")
        if np.min(np.linalg.eigvalsh(matrices)) <= 0:
            raise ValueError(f"non-positive mass matrix for {task}")
        arrays[f"{task}_q"] = trial.q.astype(np.float32)
        arrays[f"{task}_mass_matrix"] = matrices
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(output, **arrays)
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "purpose": "MuJoCo mass-matrix teacher for an offline inverse-dynamics surrogate",
        "mujoco_version": mujoco.__version__, "stride": stride,
        "source_scene": str(scene.relative_to(REPO_ROOT)).replace("\\", "/"),
        "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
        "trials": {
            task: {
                "trial_id": TRIAL_IDS[task], "samples": trial.size,
                "dynamics_csv_sha256": hashlib.sha256((trial.folder / "dynamics.csv").read_bytes()).hexdigest(),
            } for task, trial in trials.items()
        },
        "artifact": output.name,
    }
    output.with_suffix(".json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return output


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Save MuJoCo mass matrices for the AG145 trials")
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parent / "data" / "ag145_mass_matrices.npz")
    parser.add_argument("--stride", type=int, default=5)
    args = parser.parse_args(argv)
    print(build_teacher(args.output, args.stride))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
