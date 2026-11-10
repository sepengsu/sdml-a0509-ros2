"""Compare learned zero-velocity and velocity bias to MuJoCo on held-out rows."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np
import torch

from .data import JOINTS, TRIAL_IDS, load_trials, time_split
from .predict import load_model
from .teacher import load_scene_model


def validate(checkpoint: str | Path) -> dict:
    trained = load_model(checkpoint)
    trials = load_trials()
    scene = trials["T1"].folder / "ag145_scene.xml"
    physical = load_scene_model(scene)
    data = mujoco.MjData(physical)
    positions = []
    dofs = []
    for index in range(1, JOINTS + 1):
        joint = mujoco.mj_name2id(physical, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{index}")
        positions.append(int(physical.jnt_qposadr[joint]))
        dofs.append(int(physical.jnt_dofadr[joint]))
    report = {}
    for task in TRIAL_IDS:
        trial = trials[task]
        _, held_out = time_split(trial)
        static_bias = np.empty((len(held_out), JOINTS), dtype=np.float64)
        for offset, row in enumerate(held_out):
            data.qpos[:] = physical.qpos0
            data.qvel[:] = 0
            for address, value in zip(positions, trial.q[row]):
                data.qpos[address] = value
            mujoco.mj_forward(physical, data)
            static_bias[offset] = data.qfrc_bias[dofs]
        with torch.no_grad():
            output = trained(
                torch.as_tensor(trial.q[held_out], dtype=torch.float32),
                torch.as_tensor(trial.dq[held_out], dtype=torch.float32),
                torch.as_tensor(trial.ddq[held_out], dtype=torch.float32),
            )
        zero = output["gravity"].numpy()
        velocity = output["velocity_bias"].numpy()
        report[task] = {
            "held_out_rows": len(held_out),
            "zero_velocity_bias_rmse_nm": float(np.sqrt(np.mean((zero - static_bias) ** 2))),
            "velocity_bias_rmse_nm": float(np.sqrt(np.mean((velocity - (
                trial.bias[held_out] - static_bias)) ** 2))),
            "total_bias_rmse_nm": float(np.sqrt(np.mean((zero + velocity - trial.bias[held_out]) ** 2))),
        }
    return report


def main(argv: list[str] | None = None) -> int:
    folder = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Validate learned PKAN bias decomposition")
    parser.add_argument("--checkpoint", default=str(folder / "runs" / "ag145_pkan" / "final.pt"))
    parser.add_argument("--output", default=str(folder / "exports" / "ag145_pkan" / "decomposition_validation.json"))
    args = parser.parse_args(argv)
    report = validate(args.checkpoint)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
