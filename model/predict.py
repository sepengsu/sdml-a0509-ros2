"""Offline torque inference from a trained AG145 PKAN checkpoint."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import torch

from .pkan import PhysicsKAN


def load_model(checkpoint_path: str | Path) -> PhysicsKAN:
    payload = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    config = payload["config"]
    model = PhysicsKAN(torch.eye(6), torch.zeros(6), torch.zeros(6),
                       hidden_dim=config["hidden_dim"], grid_intervals=config["grid_intervals"])
    model.load_state_dict(payload["model_state"])
    model.eval()
    return model


def predict(model: PhysicsKAN, q: list[float], dq: list[float],
            ddq: list[float]) -> dict:
    if any(len(value) != 6 or not all(math.isfinite(x) for x in value)
           for value in (q, dq, ddq)):
        raise ValueError("q, dq, ddq must each contain six finite values")
    with torch.no_grad():
        output = model(torch.tensor([q], dtype=torch.float32),
                       torch.tensor([dq], dtype=torch.float32),
                       torch.tensor([ddq], dtype=torch.float32))
    return {"torque_nm": output["tau"][0].tolist(),
            "bias_nm": output["bias"][0].tolist(),
            "mass_matrix_kg_m2": output["mass_matrix"][0].tolist(),
            "damping_nm_s_rad": model.damping.tolist(),
            "provenance": "MuJoCo AG145 surrogate, not real-robot torque measurement"}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Predict offline A0509+AG145 inverse dynamics")
    parser.add_argument("--checkpoint", default=str(Path(__file__).resolve().parent / "runs" / "ag145_pkan" / "final.pt"))
    parser.add_argument("--q", type=float, nargs=6, required=True, metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6"))
    parser.add_argument("--dq", type=float, nargs=6, required=True, metavar=("V1", "V2", "V3", "V4", "V5", "V6"))
    parser.add_argument("--ddq", type=float, nargs=6, required=True, metavar=("A1", "A2", "A3", "A4", "A5", "A6"))
    args = parser.parse_args(argv)
    print(json.dumps(predict(load_model(args.checkpoint), args.q, args.dq, args.ddq), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
