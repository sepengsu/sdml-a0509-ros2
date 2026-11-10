import math
import csv
from pathlib import Path
import tempfile
import unittest

import numpy as np
import torch

from model.data import load_trials, time_split
from model.export_dynamics import export
from model.physics_check import check
from model.pkan import PhysicsKAN, SplineKANLayer
from model.predict import load_model, predict
from model.train import Config, train


MODEL_DIR = Path(__file__).resolve().parents[1]
TEACHER = MODEL_DIR / "data" / "ag145_mass_matrices.npz"


class ModelTests(unittest.TestCase):
    def test_spline_edges_have_trainable_gradients(self):
        layer = SplineKANLayer(3, 2)
        x = torch.tensor([[0.1, -0.2, 0.3], [0.6, 0.4, -0.1]], requires_grad=True)
        output = layer(x)
        self.assertEqual(tuple(output.shape), (2, 2))
        output.square().sum().backward()
        self.assertTrue(torch.isfinite(layer.spline_coeff.grad).all())
        self.assertGreater(layer.spline_coeff.grad.abs().sum().item(), 0)
        self.assertTrue(torch.isfinite(x.grad).all())

    def test_mass_is_spd_and_torque_is_affine_in_acceleration(self):
        model = PhysicsKAN(torch.diag(torch.tensor([0.6, 3.0, 1.0, 0.08, 0.09, 0.01])),
                           torch.zeros(6), torch.tensor([800., 800., 800., 600., 600., 600.]))
        q = torch.randn(5, 6) * 0.2
        dq = torch.randn(5, 6) * 0.1
        ddq_a = torch.randn(5, 6)
        ddq_b = torch.randn(5, 6)
        a = model(q, dq, ddq_a)
        b = model(q, dq, ddq_b)
        expected = torch.bmm(a["mass_matrix"], (ddq_b - ddq_a).unsqueeze(-1)).squeeze(-1)
        self.assertTrue(torch.allclose(b["tau"] - a["tau"], expected, atol=2e-5))
        self.assertGreater(torch.linalg.eigvalsh(a["mass_matrix"]).min().item(), 0)
        zero_velocity = model(q, torch.zeros_like(dq), torch.zeros_like(ddq_a))
        self.assertTrue(torch.allclose(zero_velocity["velocity_bias"], torch.zeros_like(dq)))

    def test_recorded_trials_and_physics_identity(self):
        trials = load_trials()
        self.assertEqual(set(trials), {"T1", "T2", "T3", "T4"})
        for trial in trials.values():
            training, validation = time_split(trial)
            self.assertLess(training[-1], validation[0])
            self.assertEqual(trial.q.shape[1], 6)
            self.assertTrue(np.isfinite(trial.tau_inverse).all())
        diagnostics = check(TEACHER)
        self.assertLess(max(item["equation_rmse_nm"] for item in diagnostics.values()), 1e-3)

    def test_checkpoint_predicts_finite_offline_torque(self):
        model = load_model(MODEL_DIR / "runs" / "ag145_pkan" / "final.pt")
        result = predict(model, [-0.5, -0.5, 1.6, 0, -0.6, 0], [0] * 6, [0] * 6)
        self.assertEqual(len(result["torque_nm"]), 6)
        self.assertTrue(all(math.isfinite(value) for value in result["torque_nm"]))
        self.assertTrue(np.allclose(result["torque_nm"], result["zero_velocity_bias_nm"], atol=1e-5))
        self.assertEqual(result["kinetic_energy_j"], 0)

    def test_exported_dynamics_terms_reconstruct_torque(self):
        with tempfile.TemporaryDirectory(dir=MODEL_DIR / "runs") as folder:
            report = export(MODEL_DIR / "runs" / "ag145_pkan" / "final.pt",
                            folder, trajectory="T1")
            self.assertEqual(report["trials"]["T1"]["rows"], 200)
            with (Path(folder) / "T1_dynamics.csv").open(newline="", encoding="utf-8") as stream:
                row = next(csv.DictReader(stream))
            mass = np.asarray([[float(row[f"M_{i}{j}_kg_m2"]) for j in range(1, 7)]
                               for i in range(1, 7)])
            self.assertGreater(np.linalg.eigvalsh(mass).min(), 0)
            for joint in range(1, 7):
                reconstructed = sum(float(row[f"{term}_{joint}_nm"]) for term in (
                    "zero_velocity_bias", "velocity_bias", "inertia_torque", "damping_torque"))
                self.assertAlmostEqual(reconstructed, float(row[f"tau_pred_{joint}_nm"]), places=4)

    def test_one_epoch_sequential_training_writes_checkpoints(self):
        with tempfile.TemporaryDirectory(dir=MODEL_DIR / "runs") as folder:
            report = train(Config(str(TEACHER), folder, epochs_per_task=1,
                                  replay_per_task=16, fisher_batches=1))
            self.assertTrue((Path(folder) / "final.pt").is_file())
            self.assertEqual(set(report["final_validation"]), {"T1", "T2", "T3", "T4"})


if __name__ == "__main__":
    unittest.main()
