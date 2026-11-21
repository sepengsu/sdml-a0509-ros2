"""22-input direct and residual torque networks for the seven-condition campaign.

The direct target is reference torque. The residual target is reference torque
minus nominal RBD torque. All outputs are in Nm.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import torch
from torch import nn
from torch.nn import functional as F


JOINTS = 6
INPUT_DIM = 22
GRIPPER_TYPES = ("bare", "ag145", "mdpg")


def condition_features(grippers: Sequence[str], payload_mass_kg: torch.Tensor) -> torch.Tensor:
    """Return [bare, AG145, mDPG, payload mass in kg] for each sample."""
    if payload_mass_kg.ndim == 2 and payload_mass_kg.shape[1] == 1:
        payload_mass_kg = payload_mass_kg[:, 0]
    if payload_mass_kg.ndim != 1 or len(grippers) != len(payload_mass_kg):
        raise ValueError("expected one gripper and one payload mass per sample")
    if not torch.isfinite(payload_mass_kg).all() or (payload_mass_kg < 0).any():
        raise ValueError("payload masses must be finite and nonnegative")
    try:
        indices = [GRIPPER_TYPES.index(name) for name in grippers]
    except ValueError as exc:
        raise ValueError(f"gripper must be one of {GRIPPER_TYPES}") from exc
    one_hot = F.one_hot(torch.tensor(indices, device=payload_mass_kg.device), 3)
    return torch.cat((one_hot.to(payload_mass_kg.dtype), payload_mass_kg[:, None]), dim=1)


def build_features(q: torch.Tensor, dq: torch.Tensor, ddq: torch.Tensor,
                   conditions: torch.Tensor) -> torch.Tensor:
    """Concatenate q[6], dq[6], ddq[6], gripper one-hot[3], payload kg[1]."""
    if q.ndim != 2 or q.shape[1] != JOINTS or dq.shape != q.shape or ddq.shape != q.shape:
        raise ValueError("q, dq and ddq must each have shape [batch, 6]")
    if conditions.shape != (len(q), 4):
        raise ValueError("conditions must have shape [batch, 4]")
    if any(value.device != q.device or value.dtype != q.dtype for value in (dq, ddq, conditions)):
        raise ValueError("all feature tensors must have the same dtype and device")
    if not all(torch.isfinite(value).all() for value in (q, dq, ddq, conditions)):
        raise ValueError("features must be finite")
    if not torch.all((conditions[:, :3] == 0) | (conditions[:, :3] == 1)):
        raise ValueError("gripper encoding must be one-hot")
    if not torch.all(conditions[:, :3].sum(dim=1) == 1) or (conditions[:, 3] < 0).any():
        raise ValueError("invalid gripper one-hot or payload mass")
    return torch.cat((q, dq, ddq, conditions), dim=1)


class TorqueModel(nn.Module):
    """Normalize 22 inputs and return a six-joint torque target in Nm."""

    def __init__(self, input_mean: torch.Tensor | None = None,
                 input_scale: torch.Tensor | None = None,
                 output_mean: torch.Tensor | None = None,
                 output_scale: torch.Tensor | None = None):
        super().__init__()
        mean = torch.zeros(INPUT_DIM) if input_mean is None else torch.as_tensor(input_mean).clone()
        scale = torch.ones(INPUT_DIM) if input_scale is None else torch.as_tensor(input_scale).clone()
        target_mean = torch.zeros(JOINTS) if output_mean is None else torch.as_tensor(output_mean).clone()
        target_scale = torch.ones(JOINTS) if output_scale is None else torch.as_tensor(output_scale).clone()
        if mean.shape != (INPUT_DIM,) or scale.shape != (INPUT_DIM,):
            raise ValueError("input_mean and input_scale must have 22 values")
        if target_mean.shape != (JOINTS,) or target_scale.shape != (JOINTS,):
            raise ValueError("output_mean and output_scale must have 6 values")
        if (not torch.isfinite(mean).all() or not torch.isfinite(scale).all() or (scale <= 0).any()
                or not torch.isfinite(target_mean).all() or not torch.isfinite(target_scale).all()
                or (target_scale <= 0).any()):
            raise ValueError("normalization values must be finite and scales positive")
        self.register_buffer("input_mean", mean.float())
        self.register_buffer("input_scale", scale.float())
        self.register_buffer("output_mean", target_mean.float())
        self.register_buffer("output_scale", target_scale.float())

    def _normalized(self, features: torch.Tensor) -> torch.Tensor:
        if features.ndim != 2 or features.shape[1] != INPUT_DIM:
            raise ValueError("features must have shape [batch, 22]")
        if not torch.isfinite(features).all():
            raise ValueError("features must be finite")
        if features.device != self.input_mean.device:
            raise ValueError("features and model must be on the same device")
        return (features.to(dtype=self.input_mean.dtype) - self.input_mean) / self.input_scale

    def _physical_output(self, normalized_output: torch.Tensor) -> torch.Tensor:
        return normalized_output * self.output_scale + self.output_mean


class ResidualCorrection:
    """Add a residual network output to an external nominal RBD torque."""

    def corrected_torque(self, features: torch.Tensor, nominal_torque: torch.Tensor) -> torch.Tensor:
        if nominal_torque.shape != (len(features), JOINTS):
            raise ValueError("nominal_torque must have shape [batch, 6]")
        if not torch.isfinite(nominal_torque).all():
            raise ValueError("nominal_torque must be finite")
        return nominal_torque + self(features)


class DirectMLP(TorqueModel):
    """22 -> 96 SiLU -> 96 SiLU -> 6 linear, direct torque in Nm."""

    def __init__(self, input_mean: torch.Tensor | None = None,
                 input_scale: torch.Tensor | None = None,
                 output_mean: torch.Tensor | None = None,
                 output_scale: torch.Tensor | None = None):
        super().__init__(input_mean, input_scale, output_mean, output_scale)
        self.network = nn.Sequential(
            nn.Linear(INPUT_DIM, 96), nn.SiLU(),
            nn.Linear(96, 96), nn.SiLU(),
            nn.Linear(96, JOINTS),
        )

    def forward(self, features: torch.Tensor) -> torch.Tensor:
        return self._physical_output(self.network(self._normalized(features)))


class ResidualMLP(ResidualCorrection, DirectMLP):
    """Same MLP architecture, trained on residual torque in Nm."""


class SplineKANLayer(nn.Module):
    """One KAN layer with cubic B-spline functions on every edge."""

    def __init__(self, in_features: int, out_features: int, grid_intervals: int = 5,
                 spline_order: int = 3):
        super().__init__()
        if in_features < 1 or out_features < 1 or grid_intervals < 2 or spline_order < 1:
            raise ValueError("invalid KAN dimensions, grid or spline order")
        self.in_features = in_features
        self.out_features = out_features
        self.grid_intervals = grid_intervals
        self.spline_order = spline_order
        spacing = 2.0 / grid_intervals
        knots = torch.linspace(-1 - spline_order * spacing, 1 + spline_order * spacing,
                                grid_intervals + 2 * spline_order + 1)
        self.register_buffer("knots", knots)
        basis_count = len(knots) - spline_order - 1
        self.spline_coeff = nn.Parameter(torch.empty(out_features, in_features, basis_count))
        self.bias = nn.Parameter(torch.zeros(out_features))
        nn.init.normal_(self.spline_coeff, std=0.03 / math.sqrt(in_features))

    def _basis(self, x: torch.Tensor) -> torch.Tensor:
        # tanh keeps both raw and hidden features inside the spline grid.
        x = torch.tanh(x).unsqueeze(-1)
        knots = self.knots.to(dtype=x.dtype)
        basis = ((x >= knots[:-1]) & (x < knots[1:])).to(x.dtype)
        for order in range(1, self.spline_order + 1):
            count = len(knots) - order - 1
            left = (x - knots[:count]) / (knots[order:order + count] - knots[:count])
            right = (knots[order + 1:order + count + 1] - x) / (
                knots[order + 1:order + count + 1] - knots[1:count + 1]
            )
            basis = left * basis[..., :count] + right * basis[..., 1:count + 1]
        return basis

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        if x.ndim != 2 or x.shape[1] != self.in_features:
            raise ValueError(f"expected [batch, {self.in_features}] inputs")
        return torch.einsum("bin,oin->bo", self._basis(x), self.spline_coeff) + self.bias


class DirectKAN(TorqueModel):
    """22 -> 32 KAN -> 16 KAN -> 6 KAN, direct torque in Nm."""

    def __init__(self, input_mean: torch.Tensor | None = None,
                 input_scale: torch.Tensor | None = None,
                 output_mean: torch.Tensor | None = None,
                 output_scale: torch.Tensor | None = None):
        super().__init__(input_mean, input_scale, output_mean, output_scale)
        self.network = nn.Sequential(
            SplineKANLayer(INPUT_DIM, 32),
            SplineKANLayer(32, 16),
            SplineKANLayer(16, JOINTS),
        )

    def forward(self, features: torch.Tensor) -> torch.Tensor:
        return self._physical_output(self.network(self._normalized(features)))


class ResidualKAN(ResidualCorrection, DirectKAN):
    """Same KAN architecture, trained on residual torque in Nm."""
