"""Verify the MuJoCo inverse-dynamics identity in the recorded samples."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .data import load_trials


def check(teacher_path: str | Path, stride: int = 5) -> dict:
    trials = load_trials(stride=stride)
    summary = {}
    with np.load(teacher_path) as teacher:
        for task, trial in trials.items():
            mass = teacher[f"{task}_mass_matrix"]
            if mass.shape != (trial.size, 6, 6) or not np.allclose(teacher[f"{task}_q"], trial.q, atol=1e-6):
                raise ValueError(f"mass teacher does not match {task} rows")
            predicted = (np.einsum("bij,bj->bi", mass, trial.ddq) + trial.bias
                         + trial.damping * trial.dq)
            error = predicted - trial.tau_inverse
            summary[task] = {
                "samples": trial.size,
                "equation_rmse_nm": float(np.sqrt(np.mean(error**2))),
                "equation_max_abs_nm": float(np.max(np.abs(error))),
                "joint_rmse_nm": np.sqrt(np.mean(error**2, axis=0)).tolist(),
            }
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check tau = M ddq + bias + D dq for AG145 data")
    parser.add_argument("--teacher", type=Path, default=Path(__file__).resolve().parent / "data" / "ag145_mass_matrices.npz")
    parser.add_argument("--stride", type=int, default=5)
    args = parser.parse_args(argv)
    print(json.dumps(check(args.teacher, args.stride), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
