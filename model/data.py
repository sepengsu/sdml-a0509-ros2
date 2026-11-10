"""Read the four validated AG145 MuJoCo trials without ROS dependencies."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np


TRIAL_IDS = {"T1": "trial_005", "T2": "trial_001", "T3": "trial_001", "T4": "trial_001"}
JOINTS = 6
REPO_ROOT = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Trial:
    trajectory_id: str
    folder: Path
    time_s: np.ndarray
    q: np.ndarray
    dq: np.ndarray
    ddq: np.ndarray
    tau_inverse: np.ndarray
    bias: np.ndarray
    damping: np.ndarray
    frictionloss: np.ndarray
    source_rows: int

    @property
    def size(self) -> int:
        return len(self.q)


def _vector(value: str, name: str) -> np.ndarray:
    try:
        array = np.asarray(json.loads(value), dtype=np.float64)
    except (ValueError, TypeError) as error:
        raise ValueError(f"invalid {name}: {value[:60]}") from error
    if array.shape != (JOINTS,) or not np.isfinite(array).all():
        raise ValueError(f"{name} needs six finite numbers")
    return array


def load_trial(folder: str | Path, stride: int = 5) -> Trial:
    folder = Path(folder).resolve()
    if stride < 1:
        raise ValueError("stride must be at least one")
    metadata = json.loads((folder / "metadata.json").read_text(encoding="utf-8"))
    trajectory_id = metadata["trajectory_id"]
    if (metadata["mode"] != "simulation" or metadata.get("gripper_type") != "ag145"
            or trajectory_id not in TRIAL_IDS):
        raise ValueError(f"expected an AG145 simulation trial: {folder}")
    if metadata["trial_id"] != folder.name:
        raise ValueError(f"trial ID does not match its folder: {folder}")
    if trajectory_id != folder.parent.name:
        raise ValueError(f"trajectory ID does not match its folder: {folder}")
    if metadata["joint_order"] != [f"joint_{i}" for i in range(1, JOINTS + 1)]:
        raise ValueError("unexpected joint order")
    if metadata["units"]["q"] != "rad" or metadata["units"]["torque"] != "Nm":
        raise ValueError("unexpected joint or torque units")
    rows = []
    source_rows = 0
    with (folder / "dynamics.csv").open(newline="", encoding="utf-8") as stream:
        for source_rows, row in enumerate(csv.DictReader(stream), start=1):
            if (row["trajectory_id"] != trajectory_id or row["trial_id"] != metadata["trial_id"]
                    or row["mode"] != "simulation" or row["gripper_type"] != "ag145"):
                raise ValueError(f"CSV identifiers do not match metadata at row {source_rows}")
            if (source_rows - 1) % stride:
                continue
            try:
                values = tuple(np.asarray([float(row[f"{stem}{i}"]) for i in range(1, JOINTS + 1)],
                                          dtype=np.float64) for stem in ("q", "dq", "ddq"))
                inverse = _vector(row["qfrc_inverse"], "qfrc_inverse")
                bias = _vector(row["qfrc_bias"], "qfrc_bias")
                damping = _vector(row["dof_damping"], "dof_damping")
                friction = _vector(row["dof_frictionloss"], "dof_frictionloss")
                stamp = float(row["ros_time_s"])
            except (KeyError, TypeError, ValueError):
                continue  # blank first derivative or stale measurements
            if all(np.isfinite(value).all() for value in (*values, inverse, bias, damping, friction)) and math.isfinite(stamp):
                rows.append((stamp, *values, inverse, bias, damping, friction))
    if len(rows) < 20:
        raise ValueError(f"too few valid AG145 samples in {folder}")
    time_s, q, dq, ddq, inverse, bias, damping, friction = (
        np.stack([row[i] for row in rows]) for i in range(8)
    )
    if np.any(np.diff(time_s) <= 0):
        raise ValueError("simulation time is not strictly increasing")
    if not np.allclose(damping, damping[0], rtol=0, atol=1e-9):
        raise ValueError("joint damping changes within a trial")
    if not np.allclose(friction, friction[0], rtol=0, atol=1e-9):
        raise ValueError("joint friction changes within a trial")
    return Trial(trajectory_id, folder, time_s, q, dq, ddq, inverse, bias,
                 damping[0], friction[0], source_rows)


def load_trials(root: str | Path | None = None, stride: int = 5) -> dict[str, Trial]:
    root = Path(root) if root is not None else REPO_ROOT / "results" / "ag145" / "simulation"
    trials = {trajectory_id: load_trial(root / trajectory_id / trial_id, stride)
              for trajectory_id, trial_id in TRIAL_IDS.items()}
    reference_damping = trials["T1"].damping
    for trial in trials.values():
        if not np.array_equal(trial.damping, reference_damping):
            raise ValueError("trials use different arm damping parameters")
        if np.any(trial.frictionloss != 0):
            raise ValueError("nonzero Coulomb friction is not modeled by this PKAN")
    return trials


def time_split(trial: Trial, validation_fraction: float = 0.2) -> tuple[np.ndarray, np.ndarray]:
    if not 0 < validation_fraction < 0.5:
        raise ValueError("validation_fraction must be between zero and one half")
    cut = int(trial.size * (1 - validation_fraction))
    if cut < 10 or trial.size - cut < 5:
        raise ValueError("trial too small for temporal holdout")
    return np.arange(cut), np.arange(cut, trial.size)


def data_summary(trials: dict[str, Trial]) -> dict:
    summary = {}
    for task, trial in trials.items():
        train, val = time_split(trial)
        summary[task] = {
            "source_rows": trial.source_rows, "valid_subsampled_rows": trial.size,
            "train_rows": len(train), "validation_rows": len(val),
            "max_abs_q_rad": np.max(np.abs(trial.q), axis=0).tolist(),
            "max_abs_dq_rad_s": np.max(np.abs(trial.dq), axis=0).tolist(),
            "max_abs_ddq_rad_s2": np.max(np.abs(trial.ddq), axis=0).tolist(),
            "max_abs_inverse_nm": np.max(np.abs(trial.tau_inverse), axis=0).tolist(),
            "damping_nm_s_rad": trial.damping.tolist(),
        }
    return summary
