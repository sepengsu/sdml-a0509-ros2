"""Check the new residual model contracts without fitting either network."""

import unittest

import torch

from model.residual_models import (
    ResidualKAN, ResidualMLP, SplineKANLayer, build_features, condition_features,
)


class ResidualModelTests(unittest.TestCase):
    def test_feature_order_and_conditions(self):
        q = torch.arange(12, dtype=torch.float32).reshape(2, 6)
        dq = q + 20
        ddq = q + 40
        conditions = condition_features(["bare", "mdpg"], torch.tensor([0.0, 1.0]))
        features = build_features(q, dq, ddq, conditions)
        self.assertEqual(tuple(features.shape), (2, 22))
        torch.testing.assert_close(features[:, :6], q)
        torch.testing.assert_close(features[:, 6:12], dq)
        torch.testing.assert_close(features[:, 12:18], ddq)
        torch.testing.assert_close(features[:, 18:], conditions)

    def test_architecture_and_gradients(self):
        torch.manual_seed(7)
        features = torch.randn(4, 22)
        for model, widths in ((ResidualMLP(), [(22, 96), (96, 96), (96, 6)]),
                              (ResidualKAN(), [(22, 32), (32, 16), (16, 6)])):
            layers = [layer for layer in model.network if isinstance(layer, (torch.nn.Linear, SplineKANLayer))]
            self.assertEqual([(layer.in_features, layer.out_features) for layer in layers], widths)
            residual = model(features)
            self.assertEqual(tuple(residual.shape), (4, 6))
            self.assertTrue(torch.isfinite(residual).all())
            residual.square().mean().backward()
            self.assertTrue(all(parameter.grad is not None and torch.isfinite(parameter.grad).all()
                                for parameter in model.parameters()))
            torch.testing.assert_close(model.corrected_torque(features, torch.ones(4, 6)), residual + 1)

    def test_spline_spec_and_input_validation(self):
        kan = ResidualKAN()
        self.assertTrue(all(layer.grid_intervals == 5 and layer.spline_order == 3
                            for layer in kan.network))
        with self.assertRaises(ValueError):
            kan(torch.zeros(1, 18))
        with self.assertRaises(ValueError):
            condition_features(["unknown"], torch.zeros(1))
        with self.assertRaises(ValueError):
            build_features(torch.zeros(1, 6), torch.zeros(1, 6), torch.zeros(1, 6),
                           torch.zeros(1, 4))


if __name__ == "__main__":
    unittest.main()
