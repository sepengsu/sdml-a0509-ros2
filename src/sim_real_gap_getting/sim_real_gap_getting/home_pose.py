"""Practice Home shared by the example trajectory and simulation homing action."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .trajectory import Point, _check_limits, _six


HOME_JOINT_DEG = (0.0, -32.01, 84.97994995117188, -3.99, 115.02, 45.0)
HOME_JOINT_RAD = tuple(math.radians(value) for value in HOME_JOINT_DEG)


@dataclass(frozen=True)
class HomePlan:
    name: str
    samples: tuple[Point, ...]


def plan_sim_home(current_rad, duration_s: float = 8.0) -> HomePlan:
    """Build a smooth 100 Hz simulation path from the measured pose to Home."""
    start = _six(current_rad, "current joint position")
    if (not math.isfinite(duration_s) or duration_s <= 0 or
            abs(round(duration_s * 100) - duration_s * 100) > 1e-9):
        raise ValueError("home duration must be a positive multiple of 0.01 s")
    _check_limits(Point(0.0, start, (0.0,) * 6, (0.0,) * 6), "current pose")
    count = round(duration_s * 100)
    delta = tuple(target - actual for actual, target in zip(start, HOME_JOINT_RAD))
    samples = []
    for index in range(count + 1):
        u = index / count
        blend = 10*u**3 - 15*u**4 + 6*u**5
        velocity_scale = (30*u**2 - 60*u**3 + 30*u**4) / duration_s
        acceleration_scale = (60*u - 180*u**2 + 120*u**3) / duration_s**2
        point = Point(
            index / 100,
            tuple(start[j] + delta[j] * blend for j in range(6)),
            tuple(delta[j] * velocity_scale for j in range(6)),
            tuple(delta[j] * acceleration_scale for j in range(6)),
        )
        _check_limits(point, f"home at {point.time_s:.2f}s")
        samples.append(point)
    return HomePlan("Practice Home", tuple(samples))
