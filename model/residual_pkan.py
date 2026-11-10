"""Physics-guided residual torque predictors built on the existing spline KAN."""

from __future__ import annotations

import torch
from torch import nn

from .pkan import KAN


FEATURE_DIM = 34
JOINTS = 6


def feature_array(q: torch.Tensor, dq: torch.Tensor, ddq: torch.Tensor,
                  nominal_tool: torch.Tensor) -> torch.Tensor:
    if (q.ndim != 2 or q.shape[1] != JOINTS or dq.shape != q.shape
            or ddq.shape != q.shape or nominal_tool.shape != (len(q), 10)):
        raise ValueError("expected q,dq,ddq [batch,6] and nominal_tool [batch,10]")
    return torch.cat((torch.sin(q), torch.cos(q), dq, ddq, nominal_tool), dim=1)


class ResidualPKAN(nn.Module):
    """Predict six residual torques; corrected torque is nominal + residual."""

    def __init__(self, feature_mean: torch.Tensor, feature_scale: torch.Tensor,
                 target_mean: torch.Tensor, target_scale: torch.Tensor,
                 hidden_dim: int = 16, grid_intervals: int = 8):
        super().__init__()
        if (feature_mean.shape != (FEATURE_DIM,) or feature_scale.shape != (FEATURE_DIM,)
                or target_mean.shape != (JOINTS,) or target_scale.shape != (JOINTS,)):
            raise ValueError("normalization shapes do not match the 34-to-6 model")
        if torch.any(feature_scale <= 0) or torch.any(target_scale <= 0):
            raise ValueError("normalization scales must be positive")
        self.register_buffer("feature_mean", feature_mean.clone())
        self.register_buffer("feature_scale", feature_scale.clone())
        self.register_buffer("target_mean", target_mean.clone())
        self.register_buffer("target_scale", target_scale.clone())
        self.net = KAN(FEATURE_DIM, hidden_dim, JOINTS, grid_intervals)

    def forward(self, q: torch.Tensor, dq: torch.Tensor, ddq: torch.Tensor,
                nominal_tool: torch.Tensor) -> torch.Tensor:
        features = feature_array(q, dq, ddq, nominal_tool)
        normalized = (features - self.feature_mean) / self.feature_scale
        return self.net(normalized) * self.target_scale + self.target_mean

    def corrected(self, q: torch.Tensor, dq: torch.Tensor, ddq: torch.Tensor,
                  nominal_tool: torch.Tensor, tau_nominal: torch.Tensor) -> dict[str, torch.Tensor]:
        residual = self(q, dq, ddq, nominal_tool)
        if tau_nominal.shape != residual.shape:
            raise ValueError("nominal torque must be [batch,6]")
        return {"residual": residual, "torque": tau_nominal + residual}


class ResidualMLP(ResidualPKAN):
    """Optional capacity-matched conventional baseline with the same features."""

    def __init__(self, feature_mean: torch.Tensor, feature_scale: torch.Tensor,
                 target_mean: torch.Tensor, target_scale: torch.Tensor,
                 hidden_dim: int = 64, grid_intervals: int = 8):
        super().__init__(feature_mean, feature_scale, target_mean, target_scale,
                         hidden_dim=16, grid_intervals=grid_intervals)
        self.net = nn.Sequential(nn.Linear(FEATURE_DIM, hidden_dim), nn.SiLU(),
                                 nn.Linear(hidden_dim, hidden_dim), nn.SiLU(),
                                 nn.Linear(hidden_dim, JOINTS))


def normalization(q, dq, ddq, nominal_tool, target) -> dict[str, torch.Tensor]:
    features = feature_array(torch.as_tensor(q, dtype=torch.float32),
                             torch.as_tensor(dq, dtype=torch.float32),
                             torch.as_tensor(ddq, dtype=torch.float32),
                             torch.as_tensor(nominal_tool, dtype=torch.float32))
    target = torch.as_tensor(target, dtype=torch.float32)
    feature_mean = features.mean(0)
    feature_scale = features.std(0, unbiased=False)
    feature_scale = torch.where(feature_scale < 1e-5, torch.ones_like(feature_scale), feature_scale)
    target_mean = target.mean(0)
    target_scale = target.std(0, unbiased=False).clamp_min(0.02)
    return {"feature_mean": feature_mean, "feature_scale": feature_scale,
            "target_mean": target_mean, "target_scale": target_scale}
