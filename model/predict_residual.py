"""Single-state offline inference for the nominal RBD plus residual PKAN."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import torch

from .rbd import RBDModel, flange_parameters, make_model, tool_specs
from .train_residual import load_trained


def predict(checkpoint: str | Path, tool_condition: str,
            q: list[float], dq: list[float], ddq: list[float]) -> dict:
    specs = tool_specs()
    if tool_condition not in specs:
        raise ValueError(f"unknown configured tool condition: {tool_condition}")
    if any(len(values) != 6 or not all(math.isfinite(value) for value in values)
           for values in (q, dq, ddq)):
        raise ValueError("q, dq and ddq must each have six finite values")
    nominal_spec = specs[tool_condition].scaled_nominal()
    nominal = RBDModel(make_model(nominal_spec)).evaluate(np.asarray(q), np.asarray(dq), np.asarray(ddq))
    model, payload = load_trained(checkpoint)
    with torch.no_grad():
        residual = model(*(torch.as_tensor([values], dtype=torch.float32)
                           for values in (q, dq, ddq, flange_parameters(nominal_spec).tolist())))[0].numpy()
    return {"tool_condition": tool_condition,
            "nominal_tool_parameters_flange": flange_parameters(nominal_spec).tolist(),
            "nominal_rbd_torque_nm": nominal["torque"].tolist(),
            "kan_residual_torque_nm": residual.tolist(),
            "compensated_torque_nm": (nominal["torque"] + residual).tolist(),
            "checkpoint_best_epoch": payload["best_epoch"],
            "provenance": "offline MuJoCo study; no real torque sensing or control command"}


def main(argv: list[str] | None = None) -> int:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="Predict offline RBD plus residual PKAN torque")
    parser.add_argument("--checkpoint", default=str(root / "residual_runs/rbd_v1/best_pkan.pt"))
    parser.add_argument("--tool-condition", choices=tuple(tool_specs()), default="ag145")
    parser.add_argument("--q", type=float, nargs=6, required=True)
    parser.add_argument("--dq", type=float, nargs=6, required=True)
    parser.add_argument("--ddq", type=float, nargs=6, required=True)
    args = parser.parse_args(argv)
    print(json.dumps(predict(args.checkpoint, args.tool_condition, args.q, args.dq, args.ddq), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
