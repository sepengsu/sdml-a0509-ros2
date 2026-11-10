"""Physics-structured spline KAN surrogate for A0509 + fixed AG145 dynamics.

The learned map satisfies tau = M(q) ddq + b(q, dq) + D dq. M is symmetric
positive definite by a Cholesky parameterization. Every KAN edge has a
trainable cubic B-spline function and a trainable smooth base function.
"""

from __future__ import annotations

import math

import torch
from torch import nn
from torch.nn import functional as F


class SplineKANLayer(nn.Module):
    def __init__(self, in_features: int, out_features: int, grid_intervals: int = 8,
                 degree: int = 3):
        super().__init__()
        if in_features < 1 or out_features < 1 or grid_intervals < 2 or degree < 1:
            raise ValueError("invalid KAN layer dimensions or spline grid")
        self.in_features = in_features
        self.out_features = out_features
        self.degree = degree
        spacing = 2.0 / grid_intervals
        knots = torch.linspace(-1.0 - degree * spacing, 1.0 + degree * spacing,
                               grid_intervals + 2 * degree + 1)
        self.register_buffer("knots", knots)
        bases = len(knots) - degree - 1
        self.spline_coeff = nn.Parameter(torch.empty(out_features, in_features, bases))
        self.base_weight = nn.Parameter(torch.empty(out_features, in_features))
        self.bias = nn.Parameter(torch.zeros(out_features))
        nn.init.normal_(self.spline_coeff, std=0.03 / math.sqrt(in_features))
        nn.init.normal_(self.base_weight, std=0.08 / math.sqrt(in_features))

    def _basis(self, x: torch.Tensor) -> torch.Tensor:
        x = torch.tanh(x).unsqueeze(-1)
        knots = self.knots.to(dtype=x.dtype)
        basis = ((x >= knots[:-1]) & (x < knots[1:])).to(x.dtype)
        for order in range(1, self.degree + 1):
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
        spline = torch.einsum("bin,oin->bo", self._basis(x), self.spline_coeff)
        base = F.linear(F.silu(x), self.base_weight)
        return spline + base + self.bias


class KAN(nn.Module):
    def __init__(self, input_dim: int, hidden_dim: int, output_dim: int,
                 grid_intervals: int = 8):
        super().__init__()
        self.layers = nn.Sequential(
            SplineKANLayer(input_dim, hidden_dim, grid_intervals),
            SplineKANLayer(hidden_dim, output_dim, grid_intervals),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.layers(x)


class PhysicsKAN(nn.Module):
    def __init__(self, initial_mass: torch.Tensor, initial_bias: torch.Tensor,
                 damping: torch.Tensor, hidden_dim: int = 16, grid_intervals: int = 8):
        super().__init__()
        if initial_mass.shape != (6, 6) or initial_bias.shape != (6,) or damping.shape != (6,):
            raise ValueError("initial dynamics must have six joints")
        if not torch.allclose(initial_mass, initial_mass.T, atol=1e-4):
            raise ValueError("initial mass matrix must be symmetric")
        self.hidden_dim = hidden_dim
        self.grid_intervals = grid_intervals
        self.mass_net = KAN(6, hidden_dim, 21, grid_intervals)
        self.gravity_net = KAN(6, hidden_dim, 6, grid_intervals)
        self.velocity_net = KAN(12, hidden_dim, 6, grid_intervals)
        self.register_buffer("base_cholesky", torch.linalg.cholesky(initial_mass))
        self.register_buffer("bias_anchor", initial_bias.clone())
        self.register_buffer("damping", damping.clone())
        indices = torch.tril_indices(6, 6)
        self.register_buffer("lower_rows", indices[0])
        self.register_buffer("lower_cols", indices[1])
        self.register_buffer("diagonal_in_lower", torch.nonzero(indices[0] == indices[1]).flatten())

    def forward(self, q: torch.Tensor, dq: torch.Tensor,
                ddq: torch.Tensor) -> dict[str, torch.Tensor]:
        if q.ndim != 2 or q.shape[1] != 6 or dq.shape != q.shape or ddq.shape != q.shape:
            raise ValueError("q, dq and ddq must all be [batch, 6]")
        q_scaled = q / 2.0
        dq_scaled = dq / 0.5
        raw = self.mass_net(q_scaled)
        lower = torch.zeros((len(q), 6, 6), dtype=q.dtype, device=q.device)
        lower[:, self.lower_rows, self.lower_cols] = (
            self.base_cholesky[self.lower_rows, self.lower_cols] + 0.2 * raw
        )
        diagonal = self.diagonal_in_lower
        lower[:, self.lower_rows[diagonal], self.lower_cols[diagonal]] = (
            self.base_cholesky.diagonal() * torch.exp(torch.clamp(0.2 * raw[:, diagonal], -2, 2))
        )
        mass = lower @ lower.transpose(1, 2) + 1e-6 * torch.eye(6, dtype=q.dtype, device=q.device)
        velocity_input = torch.cat((q_scaled, dq_scaled), dim=1)
        zero_velocity_input = torch.cat((q_scaled, torch.zeros_like(dq_scaled)), dim=1)
        gravity = self.bias_anchor + 10.0 * self.gravity_net(q_scaled)
        velocity_bias = 2.0 * (self.velocity_net(velocity_input)
                               - self.velocity_net(zero_velocity_input))
        bias = gravity + velocity_bias
        torque = torch.bmm(mass, ddq.unsqueeze(-1)).squeeze(-1) + bias + self.damping * dq
        return {"tau": torque, "mass_matrix": mass, "bias": bias,
                "gravity": gravity, "velocity_bias": velocity_bias}
