import json
from pathlib import Path
import unittest

import numpy as np
import torch

from model.data import load_trials
from model.rbd import RBDModel, flange_parameters, make_model, tool_specs
from model.residual_data import load_dataset
from model.residual_pkan import FEATURE_DIM, ResidualPKAN, feature_array
from model.train_residual import load_trained


ROOT = Path(__file__).resolve().parents[1]
DATASET = ROOT / "residual_data" / "rbd_v1"
RUN = ROOT / "residual_runs" / "rbd_v1"


class ResidualStudyTests(unittest.TestCase):
    def test_rbd_matches_recorded_inverse_without_double_damping(self):
        trial = load_trials()["T1"]
        spec = tool_specs()["ag145"]
        reference = RBDModel(make_model(spec)).evaluate(trial.q[50], trial.dq[50], trial.ddq[50])
        nominal = RBDModel(make_model(spec.scaled_nominal())).evaluate(
            trial.q[50], trial.dq[50], trial.ddq[50])
        reconstructed = (reference["mass_matrix"] @ trial.ddq[50]
                         + reference["bias"] + reference["damping"])
        np.testing.assert_allclose(reconstructed, reference["torque"], atol=2e-5)
        np.testing.assert_allclose(reference["torque"], trial.tau_inverse[50], atol=1e-6)
        self.assertGreater(np.linalg.norm(reference["torque"] - nominal["torque"]), 0.1)
        self.assertEqual(flange_parameters(spec).shape, (10,))

    def test_splits_and_features_exclude_true_tool_parameters(self):
        data = load_dataset(DATASET)
        training = data["split"] == "train"
        self.assertFalse(np.any(data["trajectory_id"][training] == "T4"))
        self.assertFalse(np.any(data["tool_condition"][training] == "virtual_unseen"))
        self.assertTrue(np.any(data["split"] == "test_new_trial"))
        np.testing.assert_allclose(data["tau_reference"] - data["tau_nominal"],
                                   data["delta_tau_target"], atol=1e-10)
        np.testing.assert_allclose(data["tau_aware"], data["tau_reference"], atol=2e-5)
        selected = np.flatnonzero((data["tool_condition"] == "ag145") & training)[:4]
        self.assertFalse(np.allclose(data["tool_nominal"][selected], data["tool_true"][selected]))
        features = feature_array(*(torch.as_tensor(data[key][selected], dtype=torch.float32)
                                   for key in ("q", "dq", "ddq", "tool_nominal")))
        self.assertEqual(features.shape, (4, FEATURE_DIM))
        np.testing.assert_allclose(features[:, -10:].numpy(), data["tool_nominal"][selected], atol=1e-6)

    def test_checkpoint_and_heldout_metrics(self):
        model, checkpoint = load_trained(RUN / "best_pkan.pt", DATASET)
        self.assertIsInstance(model, ResidualPKAN)
        self.assertGreater(checkpoint["best_epoch"], 0)
        data = load_dataset(DATASET)
        index = np.flatnonzero(data["split"] == "test_new_trial")[:5]
        with torch.no_grad():
            residual = model(*(torch.as_tensor(data[key][index], dtype=torch.float32)
                               for key in ("q", "dq", "ddq", "tool_nominal")))
        self.assertEqual(tuple(residual.shape), (5, 6))
        self.assertTrue(torch.isfinite(residual).all())
        metrics = json.loads((RUN / "metrics.json").read_text(encoding="utf-8"))
        for split in ("test_new_trial", "test_unseen_trajectory", "test_unseen_tool"):
            self.assertGreater(metrics["splits"][split]["all"]["samples"], 0)
            self.assertIn("joint_rmse_nm", metrics["splits"][split]["all"]["models"]["residual_pkan"])


if __name__ == "__main__":
    unittest.main()
