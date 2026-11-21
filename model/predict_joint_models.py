"""Offline M0-M4 torque inference for one six-joint state and tool condition."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch

from .campaign_rbd import load_scene
from .prepare_joint_dataset import CONDITIONS
from .rbd import RBDModel
from .residual_models import build_features, condition_features
from .train_joint_models import MODELS, load_dataset


def predict(data_root: Path, model_root: Path, condition: str,
            q: np.ndarray, dq: np.ndarray, ddq: np.ndarray) -> dict[str, list[float]]:
    manifest, _ = load_dataset(data_root)
    if condition not in CONDITIONS:
        raise ValueError(f"condition must be one of {CONDITIONS}")
    q, dq, ddq = (np.asarray(values, dtype=np.float64) for values in (q, dq, ddq))
    if any(values.shape != (6,) or not np.isfinite(values).all() for values in (q, dq, ddq)):
        raise ValueError("q, dq, ddq must each contain six finite values")
    source = Path(manifest["source"])
    scene = source / "models" / "conditions" / f"{condition}.xml"
    rbd = RBDModel(load_scene(scene, manifest["nominal_inertial_scale"]))
    nominal = rbd.evaluate(q, dq, ddq)["torque"]
    gripper = "bare" if condition == "bare_p0" else condition.split("_")[0]
    payload_kg = {"p0": 0.0, "p05": 0.5, "p10": 1.0}[condition.split("_")[-1]]
    with torch.no_grad():
        feature = build_features(
            torch.from_numpy(q[None].astype(np.float32)),
            torch.from_numpy(dq[None].astype(np.float32)),
            torch.from_numpy(ddq[None].astype(np.float32)),
            condition_features([gripper], torch.tensor([payload_kg], dtype=torch.float32)),
        )
        outputs = {"M0": nominal.tolist()}
        for model_id, (model_type, expected_name, target_kind) in MODELS.items():
            checkpoint = torch.load(model_root / model_id / "best.pt",
                                    map_location="cpu", weights_only=True)
            if (checkpoint.get("model_id") != model_id or checkpoint.get("model_name") != expected_name
                    or checkpoint.get("target_kind") != target_kind
                    or checkpoint.get("dataset_sha256") != manifest["dataset_sha256"]
                    or checkpoint.get("source") != manifest["source"]
                    or checkpoint.get("profile") != manifest["profile"]
                    or checkpoint.get("nominal_inertial_scale") != manifest["nominal_inertial_scale"]):
                raise ValueError(f"checkpoint provenance mismatch: {model_id}")
            model = model_type()
            model.load_state_dict(checkpoint["model_state"])
            model.eval()
            torque = model(feature).numpy()[0].astype(np.float64)
            if target_kind == "residual":
                torque += nominal
            outputs[model_id] = torque.tolist()
    return outputs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--condition", choices=CONDITIONS, required=True)
    parser.add_argument("--q", nargs=6, type=float, required=True, metavar="RAD")
    parser.add_argument("--dq", nargs=6, type=float, required=True, metavar="RAD_PER_S")
    parser.add_argument("--ddq", nargs=6, type=float, required=True, metavar="RAD_PER_S2")
    args = parser.parse_args()
    outputs = predict(args.data, args.models, args.condition,
                      np.array(args.q), np.array(args.dq), np.array(args.ddq))
    print(json.dumps({"condition": args.condition, "unit": "Nm",
                      "predictions": outputs,
                      "note": "offline MuJoCo-trained predictions, not measured torque"}, indent=2))


if __name__ == "__main__":
    main()
