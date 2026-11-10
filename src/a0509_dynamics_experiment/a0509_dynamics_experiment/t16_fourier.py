"""Frozen, offline-optimized T16 Fourier reference. No robot interfaces here.

The four free coefficients per joint enforce q(0)=q(T)=home and zero endpoint
velocity and acceleration. The JSON is produced by t16_optimize.py; a missing
or unvalidated result is deliberately an error rather than a fallback path.
"""

from __future__ import annotations

import json
import math
from pathlib import Path


PERIOD_S = 10.0
FUNDAMENTAL_HZ = 0.1
HARMONICS = 3


def spec_path() -> Path:
    local = Path(__file__).resolve().parents[1] / "config" / "t16_fourier_optimized.json"
    if local.is_file():
        return local
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("a0509_dynamics_experiment")) / "config" / local.name
    except (ImportError, LookupError, OSError):
        return local


def load_spec() -> dict:
    path = spec_path()
    if not path.is_file():
        raise RuntimeError(f"validated T16 Fourier coefficients are missing: {path}")
    spec = json.loads(path.read_text(encoding="utf-8"))
    if (spec.get("status") != "validated_simulation_only" or
            spec.get("objective_kind") != "parameter-sensitivity proxy" or
            spec.get("period_s") != PERIOD_S or
            spec.get("fundamental_hz") != FUNDAMENTAL_HZ or
            len(spec.get("free_coefficients_rad_s", [])) != 6):
        raise RuntimeError("T16 Fourier coefficient file lacks validated provenance")
    if any(len(row) != 4 or not all(math.isfinite(float(x)) for x in row)
           for row in spec["free_coefficients_rad_s"]):
        raise RuntimeError("invalid T16 Fourier coefficients")
    return spec


def coefficients(free_coefficients):
    """Return (a[6,3], b[6,3]) in rad/s with endpoint C2 constraints."""
    a, b = [], []
    for a1, a2, b1, b2 in free_coefficients:
        a.append((a1, a2, -a1 - a2))
        b.append((b1, b2, -(b1 + 2.0 * b2) / 3.0))
    return tuple(a), tuple(b)


def evaluate(free_coefficients, home_rad, time_s: float, duration_s: float = PERIOD_S):
    """Analytic (q, dq, ddq); duration scaling preserves the optimized shape."""
    if duration_s <= 0 or not 0 <= time_s <= duration_s:
        raise ValueError("time must lie within the positive Fourier period")
    home = tuple(float(x) for x in home_rad)
    if len(home) != 6:
        raise ValueError("T16 requires six home joint angles")
    if time_s == 0 or time_s == duration_s:
        zero = (0.0,) * 6
        return home, zero, zero
    scale = PERIOD_S / duration_s
    t = time_s * scale
    a, b = coefficients(free_coefficients)
    q, dq, ddq = list(home), [0.0] * 6, [0.0] * 6
    for joint in range(6):
        for k in range(HARMONICS):
            omega = 2 * math.pi * FUNDAMENTAL_HZ * (k + 1)
            sine, cosine = math.sin(omega * t), math.cos(omega * t)
            q[joint] += (a[joint][k] * sine + b[joint][k] * (1 - cosine)) / omega
            dq[joint] += scale * (a[joint][k] * cosine + b[joint][k] * sine)
            ddq[joint] += scale * scale * omega * (-a[joint][k] * sine + b[joint][k] * cosine)
    return tuple(q), tuple(dq), tuple(ddq)
