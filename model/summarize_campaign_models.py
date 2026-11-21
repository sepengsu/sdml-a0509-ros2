"""Combine seven-condition RBD and PhysicsKAN metrics without mixing campaigns."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATA = ROOT / "data/grid16_fourier_pkan_csv"
DEFAULT_RBD = ROOT / "data/grid16_fourier_rbd/metrics.json"
DEFAULT_PKAN = ROOT / "data/grid16_fourier_pkan_models"
DEFAULT_RESIDUAL = ROOT / "data/grid16_fourier_residual_kan/metrics.json"
DEFAULT_OUTPUT = ROOT / "data/grid16_fourier_model_comparison"


def summarize(data_root: Path, rbd_path: Path, pkan_root: Path,
              residual_path: Path, output: Path) -> dict:
    data_root, rbd_path, pkan_root, residual_path, output = (path.resolve() for path in
                                                            (data_root, rbd_path, pkan_root,
                                                             residual_path, output))
    manifest_path = data_root / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    rbd = json.loads(rbd_path.read_text(encoding="utf-8"))
    residual = json.loads(residual_path.read_text(encoding="utf-8"))
    digest = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    if any(item["export_manifest_sha256"] != digest or Path(item["data_root"]).resolve() != data_root
           for item in (rbd, residual)):
        raise ValueError("RBD or residual KAN results were evaluated on another export")
    if residual["nominal_inertial_scale"] != rbd["nominal_inertial_scale"]:
        raise ValueError("nominal RBD definitions differ")
    conditions = sorted(manifest["conditions"])
    if set(rbd["conditions"]) != set(conditions) or residual["conditions"] != conditions:
        raise ValueError("model results do not cover every condition")
    rows = []
    trajectories = []
    for condition in conditions:
        pkan = json.loads((pkan_root / condition / "metrics.json").read_text(encoding="utf-8"))
        if pkan["condition"] != condition or Path(pkan["data_root"]).resolve() != data_root:
            raise ValueError(f"PKAN result is from another condition or export: {condition}")
        if pkan["split_policy"] != manifest["split_policy"]:
            raise ValueError(f"PKAN split policy mismatch: {condition}")
        for split in ("validation", "test"):
            scores = rbd["conditions"][condition]["metrics"][split]
            learned = pkan["metrics"][split]
            corrected = residual["metrics"][split]["rbd_plus_residual_kan"]
            expected = manifest["conditions"][condition]["rows"][split]
            if any(item != expected for item in (
                    scores["scene_rbd"]["all"]["samples"],
                    scores["nominal_rbd"]["all"]["samples"],
                    learned["all"]["samples"],
                    corrected["by_condition"][condition]["samples"])):
                raise ValueError(f"sample count mismatch: {condition}/{split}")
            rows.append({
                "condition": condition, "split": split, "samples": expected,
                "scene_rbd_rmse_nm": scores["scene_rbd"]["all"]["torque_rmse_nm"],
                "nominal_rbd_rmse_nm": scores["nominal_rbd"]["all"]["torque_rmse_nm"],
                "pkan_rmse_nm": learned["all"]["torque_rmse_nm"],
                "rbd_plus_residual_kan_rmse_nm": corrected["by_condition"][condition]["torque_rmse_nm"],
                "fixed_mass_bias_rmse_nm": pkan["fixed_mass_bias_baseline"][split]["all"]["torque_rmse_nm"],
                "pkan_best_epoch": pkan["best_epoch"],
            })
            for trajectory in manifest["split_policy"][split]:
                trajectories.append({
                    "condition": condition, "split": split, "trajectory": trajectory,
                    "samples": learned["by_trajectory"][trajectory]["samples"],
                    "scene_rbd_rmse_nm": scores["scene_rbd"]["by_trajectory"][trajectory]["torque_rmse_nm"],
                    "nominal_rbd_rmse_nm": scores["nominal_rbd"]["by_trajectory"][trajectory]["torque_rmse_nm"],
                    "pkan_rmse_nm": learned["by_trajectory"][trajectory]["torque_rmse_nm"],
                    "rbd_plus_residual_kan_rmse_nm": corrected["by_trajectory"][condition][trajectory]["torque_rmse_nm"],
                })
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "data_root": str(data_root), "campaign": manifest["source"],
        "rbd_metrics": str(rbd_path), "pkan_root": str(pkan_root),
        "residual_metrics": str(residual_path),
        "nominal_inertial_scale": rbd["nominal_inertial_scale"],
        "rows": rows, "trajectory_rows": trajectories,
        "interpretation": [
            "scene_rbd uses the same saved MuJoCo physical parameters as the label and is an internal oracle check",
            "nominal_rbd has deliberately reduced tool/payload inertials; the arm is unchanged",
            "PKAN is trained independently for each condition using training torque, mass matrix and bias labels",
            "the residual KAN is one shared model trained on all seven conditions with a condition one-hot input",
            "T12 selects the PKAN checkpoint; T13/T14/Fourier T16 are held-out trajectories",
            "three deterministic repeats are not independent state coverage; the export uses the first completed trial",
            "all torques are simulated inverse dynamics, not measured robot motor torques",
        ],
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "summary.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                         encoding="utf-8")
    for name, items in (("comparison.csv", rows), ("by_trajectory.csv", trajectories)):
        with (output / name).open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(items[0]))
            writer.writeheader()
            writer.writerows(items)
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--rbd", type=Path, default=DEFAULT_RBD)
    parser.add_argument("--pkan-root", type=Path, default=DEFAULT_PKAN)
    parser.add_argument("--residual", type=Path, default=DEFAULT_RESIDUAL)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    report = summarize(args.data, args.rbd, args.pkan_root, args.residual, args.output)
    print(json.dumps({"output": str(args.output.resolve()), "conditions": len(report["rows"]) // 2,
                      "splits": ["validation", "test"]}, indent=2))


if __name__ == "__main__":
    main()
