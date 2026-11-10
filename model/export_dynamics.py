"""Export PKAN inverse-dynamics terms along recorded AG145 trajectories."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from .data import TRIAL_IDS, Trial, load_trials
from .predict import load_model


TERMS = ("zero_velocity_bias", "velocity_bias", "inertia_torque",
         "damping_torque", "bias", "tau_pred", "tau_mujoco", "tau_error")


def _columns() -> list[str]:
    columns = ["trajectory_id", "trial_id", "ros_time_s"]
    for stem in ("q_rad", "dq_rad_s", "ddq_rad_s2"):
        columns.extend(f"{stem}_{joint}" for joint in range(1, 7))
    columns.extend(f"M_{row}{column}_kg_m2" for row in range(1, 7) for column in range(1, 7))
    for stem in TERMS:
        columns.extend(f"{stem}_{joint}_nm" for joint in range(1, 7))
    return columns


def _predict_trial(model, trial: Trial) -> dict[str, np.ndarray]:
    chunks = {key: [] for key in ("mass_matrix", "gravity", "velocity_bias", "bias", "tau")}
    model.eval()
    with torch.no_grad():
        for start in range(0, trial.size, 256):
            stop = min(trial.size, start + 256)
            output = model(
                torch.as_tensor(trial.q[start:stop], dtype=torch.float32),
                torch.as_tensor(trial.dq[start:stop], dtype=torch.float32),
                torch.as_tensor(trial.ddq[start:stop], dtype=torch.float32),
            )
            for key in chunks:
                chunks[key].append(output[key].cpu().numpy().astype(np.float64))
    return {key: np.concatenate(values) for key, values in chunks.items()}


def export(checkpoint: str | Path, output_dir: str | Path,
           trajectory: str = "all", stride: int = 5) -> dict:
    if trajectory not in (*TRIAL_IDS, "all"):
        raise ValueError("trajectory must be T1, T2, T3, T4 or all")
    if stride < 1:
        raise ValueError("stride must be at least one")
    checkpoint = Path(checkpoint).resolve()
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    model = load_model(checkpoint)
    trials = load_trials(stride=stride)
    selected = TRIAL_IDS if trajectory == "all" else (trajectory,)
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "checkpoint": checkpoint.name,
        "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
        "stride": stride,
        "formula": "tau = M(q) ddq + zero_velocity_bias(q) + velocity_bias(q,dq) + D dq",
        "joint_order": [f"joint_{i}" for i in range(1, 7)],
        "units": {"q": "rad", "dq": "rad/s", "ddq": "rad/s^2", "M": "kg m^2",
                  "torque_terms": "Nm", "ros_time_s": "simulation seconds"},
        "interpretation": {
            "zero_velocity_bias": "b(q,0); gravity-like, not separately supervised as gravity",
            "velocity_bias": "b(q,dq)-b(q,0); not an identified Coriolis matrix",
            "inertia_torque": "M(q)ddq",
            "damping_torque": "fixed D dq from the MuJoCo arm model",
            "M": "configuration-dependent generalized inertia, not individual link inertial parameters",
        },
        "scope": "MuJoCo A0509 with fixed-open AG145; no real-robot validation or payload variation",
        "trials": {},
    }
    columns = _columns()
    for task in selected:
        trial = trials[task]
        prediction = _predict_trial(model, trial)
        inertia = np.einsum("bij,bj->bi", prediction["mass_matrix"], trial.ddq)
        damping = trial.dq * trial.damping
        reconstructed = (inertia + prediction["gravity"] + prediction["velocity_bias"] + damping)
        if not np.allclose(reconstructed, prediction["tau"], rtol=1e-5, atol=2e-4):
            raise RuntimeError(f"PKAN torque decomposition failed for {task}")
        error = prediction["tau"] - trial.tau_inverse
        path = output_dir / f"{task}_dynamics.csv"
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(columns)
            for index in range(trial.size):
                values = [task, trial.folder.name, trial.time_s[index],
                          *trial.q[index], *trial.dq[index], *trial.ddq[index],
                          *prediction["mass_matrix"][index].reshape(-1),
                          *prediction["gravity"][index], *prediction["velocity_bias"][index],
                          *inertia[index], *damping[index], *prediction["bias"][index],
                          *prediction["tau"][index], *trial.tau_inverse[index], *error[index]]
                writer.writerow(values)
        manifest["trials"][task] = {
            "source_trial": trial.folder.name,
            "rows": trial.size,
            "csv": path.name,
            "torque_rmse_nm": float(np.sqrt(np.mean(error**2))),
            "minimum_M_eigenvalue": float(np.linalg.eigvalsh(prediction["mass_matrix"]).min()),
        }
    (output_dir / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return manifest


def main(argv: list[str] | None = None) -> int:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Export A0509+AG145 PKAN mass and torque terms")
    parser.add_argument("--checkpoint", default=str(root / "runs" / "ag145_pkan" / "final.pt"))
    parser.add_argument("--output-dir", default=str(root / "exports" / "ag145_pkan"))
    parser.add_argument("--trajectory", choices=(*TRIAL_IDS, "all"), default="all")
    parser.add_argument("--stride", type=int, default=5)
    args = parser.parse_args(argv)
    manifest = export(args.checkpoint, args.output_dir, args.trajectory, args.stride)
    print(json.dumps({task: details for task, details in manifest["trials"].items()}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
