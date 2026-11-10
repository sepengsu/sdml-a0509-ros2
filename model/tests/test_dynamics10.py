"""Check source provenance, held-out splits, and saved residual predictions."""

import csv
import hashlib
import json
from pathlib import Path
import unittest

import numpy as np
import torch

from model.residual_data import load_dataset
from model.train_residual import load_trained


ROOT = Path(__file__).resolve().parents[2]
DATASET = ROOT / "model" / "residual_data" / "dynamics10_v1"
RUN = ROOT / "model" / "residual_runs" / "dynamics10_v1"


class Dynamics10StudyTests(unittest.TestCase):
    def test_trials_are_disjoint_and_source_files_match_manifest(self):
        manifest = json.loads((DATASET / "manifest.json").read_text(encoding="utf-8"))
        data = load_dataset(DATASET)
        expected = {**{f"T{i}": "train" for i in range(5, 12)},
                    "T12": "validation", "T13": "test", "T14": "test",
                    "T4": "diagnostic"}
        for trajectory, split in expected.items():
            mask = data["trajectory_id"] == trajectory
            self.assertEqual(set(data["split"][mask]), {split})
            source = manifest["sources"][trajectory]
            path = ROOT / source["folder"] / "dynamics.csv"
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),
                             source["dynamics_sha256"])
            self.assertEqual(int(mask.sum()), source["selected_rows"])
        self.assertEqual(len(data["q"]), sum(manifest["counts"].values()))
        self.assertEqual(set(data["tool_condition"]), {"ag145"})
        np.testing.assert_allclose(data["tau_reference"] - data["tau_nominal"],
                                   data["delta_tau_target"], atol=1e-10)
        np.testing.assert_allclose(data["tau_aware"], data["tau_reference"], atol=2e-5)
        self.assertFalse(np.allclose(data["tool_nominal"], data["tool_true"]))

    def test_checkpoint_is_bound_to_new_dataset_and_predicts_finite_torque(self):
        model, checkpoint = load_trained(RUN / "best_pkan.pt", DATASET)
        self.assertEqual(checkpoint["dataset_sha256"],
                         hashlib.sha256((DATASET / "dataset.npz").read_bytes()).hexdigest())
        data = load_dataset(DATASET)
        indices = np.flatnonzero(data["trajectory_id"] == "T13")[:8]
        with torch.no_grad():
            correction = model(*(torch.as_tensor(data[key][indices], dtype=torch.float32)
                                 for key in ("q", "dq", "ddq", "tool_nominal")))
        self.assertEqual(correction.shape, (8, 6))
        self.assertTrue(torch.isfinite(correction).all())

    def test_saved_metrics_recompute_from_prediction_csv(self):
        report = json.loads((RUN / "metrics.json").read_text(encoding="utf-8"))
        with (RUN / "predictions.csv").open(newline="", encoding="utf-8") as stream:
            rows = list(csv.DictReader(stream))
        self.assertEqual(len(rows), sum(part["samples"] for part in report["metrics"].values()))
        for trajectory in ("T12", "T13", "T14"):
            selected = [row for row in rows if row["trajectory_id"] == trajectory]
            self.assertEqual(len(selected), report["trajectory_metrics"][trajectory]["samples"])
            reference = np.array([[float(row[f"tau_reference_j{j}_nm"]) for j in range(1, 7)]
                                  for row in selected])
            for name in ("nominal_rbd", "previous_pkan", "dynamics10_pkan", "dynamics10_mlp"):
                prediction = np.array([[float(row[f"{name}_j{j}_nm"]) for j in range(1, 7)]
                                       for row in selected])
                rmse = float(np.sqrt(np.mean((prediction - reference)**2)))
                self.assertAlmostEqual(rmse, report["trajectory_metrics"][trajectory]
                                       ["models"][name]["rmse_nm"], places=10)
        self.assertGreater(report["trajectory_metrics"]["T13"]["models"]
                           ["dynamics10_pkan"]["rmse_nm"],
                           report["trajectory_metrics"]["T12"]["models"]
                           ["dynamics10_pkan"]["rmse_nm"])


if __name__ == "__main__":
    unittest.main()
