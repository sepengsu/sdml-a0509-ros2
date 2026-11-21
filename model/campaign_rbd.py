"""Evaluate rigid-body inverse dynamics for the seven-condition grid campaign.

The saved condition XMLs are the simulator's physical models.  The exact-scene
result is an internal reconstruction check, not an independent torque baseline.
The optional nominal result deliberately scales tool and payload inertials to
80% so that its model error is explicit and reproducible.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .rbd import RBDModel


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CAMPAIGN = ROOT / "data/grid_16_fourier_t16_20261111"
DEFAULT_DATA = ROOT / "data/grid16_fourier_pkan_csv"
DEFAULT_OUTPUT = ROOT / "data/grid16_fourier_rbd"
SPLITS = ("validation", "test")


def _local_asset(virtual: str) -> Path:
    prefixes = {
        "/ros2_ws/install/mdpg_1s1_description/share/mdpg_1s1_description/":
            ROOT / "src/mdpg_1s1_description",
        "/ros2_ws/src/": ROOT / "src",
        "/ros2_ws/data/": ROOT / "data",
    }
    for prefix, base in prefixes.items():
        if virtual.startswith(prefix):
            path = base / virtual[len(prefix):]
            if not path.is_file():
                raise FileNotFoundError(f"campaign asset missing: {path}")
            return path
    raise ValueError(f"unknown campaign asset path: {virtual}")


def load_scene(path: Path, inertial_scale: float = 1.0) -> mujoco.MjModel:
    """Load a saved condition scene without changing any arm inertial.

    `inertial_scale` affects only the attached tool and experiment payload.
    COM locations and the arm remain as recorded in the campaign.
    """
    if not 0 < inertial_scale <= 1:
        raise ValueError("inertial_scale must be in (0, 1]")
    root = ET.parse(path).getroot()
    for mesh in root.iter("mesh"):
        source = mesh.get("file")
        if source and source.startswith("/ros2_ws/"):
            mesh.set("file", _local_asset(source).as_posix())
    if inertial_scale != 1:
        for name in ("ag145_base_link", "mdpg_base_link", "experiment_payload"):
            body = root.find(f".//body[@name='{name}']")
            if body is None:
                continue
            inertial = body.find("inertial")
            if inertial is None:
                raise ValueError(f"{name} has no inertial in {path}")
            for key in ("mass", "diaginertia", "fullinertia"):
                value = inertial.get(key)
                if value is not None:
                    inertial.set(key, " ".join(f"{float(x) * inertial_scale:.16g}"
                                                for x in value.split()))
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def _read_states(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    q, dq, ddq, tau, ids = [], [], [], [], []
    with path.open(encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            q.append([float(row[f"q{i}"]) for i in range(1, 7)])
            dq.append([float(row[f"dq{i}"]) for i in range(1, 7)])
            ddq.append([float(row[f"ddq{i}"]) for i in range(1, 7)])
            tau.append([float(row[f"tau_inverse{i}"]) for i in range(1, 7)])
            ids.append(row["trajectory_id"])
    arrays = tuple(np.asarray(values, dtype=np.float64) for values in (q, dq, ddq, tau))
    if not ids or any(not np.isfinite(values).all() for values in arrays):
        raise ValueError(f"empty or nonfinite split: {path}")
    return *arrays, np.asarray(ids)


def _score(predicted: np.ndarray, actual: np.ndarray, ids: np.ndarray) -> dict:
    def one(mask: np.ndarray) -> dict:
        error = predicted[mask] - actual[mask]
        return {"samples": int(mask.sum()),
                "torque_rmse_nm": float(np.sqrt(np.mean(error**2))),
                "joint_rmse_nm": np.sqrt(np.mean(error**2, axis=0)).tolist(),
                "max_abs_error_nm": float(np.max(np.abs(error)))}
    return {"all": one(np.ones(len(ids), dtype=bool)),
            "by_trajectory": {name: one(ids == name) for name in sorted(set(ids))}}


def evaluate(campaign: Path, data_root: Path, output: Path, *, nominal_scale: float = 0.8,
             conditions: list[str] | None = None) -> dict:
    campaign, data_root, output = campaign.resolve(), data_root.resolve(), output.resolve()
    manifest_path = data_root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if Path(manifest["source"]).resolve() != campaign:
        raise ValueError("export manifest points to a different campaign")
    available = sorted(manifest["conditions"])
    chosen = conditions or available
    if not chosen or any(name not in available for name in chosen):
        raise ValueError(f"condition must be one of {available}")
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "campaign": str(campaign), "data_root": str(data_root),
        "export_manifest_sha256": hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        "nominal_inertial_scale": nominal_scale,
        "reference_definition": "MuJoCo inverse dynamics from the saved condition XML; same simulator as the labels",
        "nominal_definition": "saved condition XML with tool and payload mass/inertia scaled; arm and COM unchanged",
        "label": manifest["torque_label"],
        "conditions": {},
        "software": {"python": sys.version.split()[0], "numpy": np.__version__,
                     "mujoco": mujoco.__version__},
    }
    for condition in chosen:
        scene = campaign / "models/conditions" / f"{condition}.xml"
        if not scene.is_file():
            raise FileNotFoundError(scene)
        models = {"scene_rbd": RBDModel(load_scene(scene)),
                  "nominal_rbd": RBDModel(load_scene(scene, nominal_scale))}
        scores = {}
        for split in SPLITS:
            q, dq, ddq, tau, ids = _read_states(data_root / condition / f"{split}.csv")
            if set(ids) != set(manifest["split_policy"][split]):
                raise ValueError(f"trajectory split mismatch: {condition}/{split}")
            scores[split] = {}
            for name, model in models.items():
                prediction = model.evaluate_many(q, dq, ddq)["torque"]
                scores[split][name] = _score(prediction, tau, ids)
            # An exact-scene comparison verifies the asset mapping, state units,
            # and torque definition before any model result is accepted.
            if scores[split]["scene_rbd"]["all"]["max_abs_error_nm"] > 2e-5:
                raise ValueError(f"scene RBD does not reproduce {condition}/{split} labels")
        report["conditions"][condition] = {
            "scene": str(scene), "scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest(),
            "metrics": scores,
        }
    output.mkdir(parents=True, exist_ok=True)
    (output / "metrics.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                         encoding="utf-8")
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--campaign", type=Path, default=DEFAULT_CAMPAIGN)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--nominal-scale", type=float, default=0.8)
    parser.add_argument("--condition", action="append")
    args = parser.parse_args()
    report = evaluate(args.campaign, args.data, args.output,
                      nominal_scale=args.nominal_scale, conditions=args.condition)
    print(json.dumps({name: {split: {model: scores[model]["all"]["torque_rmse_nm"]
                                           for model in scores}
                                   for split, scores in item["metrics"].items()}
                      for name, item in report["conditions"].items()}, indent=2))


if __name__ == "__main__":
    main()
