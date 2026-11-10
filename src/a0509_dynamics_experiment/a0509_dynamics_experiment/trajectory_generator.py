"""Deterministic, robot-independent reference trajectories.

Cartesian coordinates are base-frame millimetres. Joint coordinates are radians.
The suggested dimensions are research examples, not validated robot paths.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Iterable

JOINT_NAMES = tuple(f"joint_{i}" for i in range(1, 7))
DEFAULT_SIZES_MM = {"T1": (120.0, 0.0, 0.0), "T2": (0.0, 80.0, 60.0), "T3": (120.0, 80.0, 60.0)}


@dataclass(frozen=True)
class TrajectoryConfig:
    trajectory_id: str
    reference_tcp_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    start_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    end_mm: tuple[float, float, float] | None = None
    joint_start_rad: tuple[float, ...] = (0.0,) * 6
    joint_indices: tuple[int, ...] = (0, 1, 2, 3, 4, 5)
    joint_mode: str = "sequential"
    joint_amplitude_deg: float = 5.0
    speed: str = "low"
    segment_seconds: float | None = None
    repeats: int = 1
    pre_hold_seconds: float = 1.0
    post_hold_seconds: float = 1.0
    sample_hz: int = 100

    def validate(self) -> None:
        if self.trajectory_id not in ("T1", "T2", "T3", "T4"):
            raise ValueError("trajectory_id must be T1, T2, T3, or T4")
        if self.speed not in ("low", "medium"):
            raise ValueError("speed must be low or medium")
        if self.repeats < 1 or self.sample_hz < 2:
            raise ValueError("repeats must be positive and sample_hz must be at least 2")
        if self.pre_hold_seconds < 0 or self.post_hold_seconds < 0:
            raise ValueError("hold times cannot be negative")
        if self.segment_seconds is not None and self.segment_seconds <= 0:
            raise ValueError("segment_seconds must be positive")
        if len(self.reference_tcp_mm) != 3 or len(self.start_mm) != 3 or (self.end_mm is not None and len(self.end_mm) != 3):
            raise ValueError("Cartesian coordinates must have three values")
        if len(self.joint_start_rad) != 6 or not self.joint_indices or any(i not in range(6) for i in self.joint_indices):
            raise ValueError("joint_start_rad needs six values and joint_indices must select joints 1..6")
        if self.joint_amplitude_deg <= 0:
            raise ValueError("joint_amplitude_deg must be positive")
        if not math.isfinite(self.joint_amplitude_deg):
            raise ValueError("joint_amplitude_deg must be finite")
        if self.joint_mode not in ("sequential", "simultaneous"):
            raise ValueError("joint_mode must be sequential or simultaneous")
        optional_duration = (self.segment_seconds,) if self.segment_seconds is not None else ()
        values = (*self.reference_tcp_mm, *self.start_mm, *self.joint_start_rad,
                  *(self.end_mm or ()), *optional_duration)
        if not all(math.isfinite(x) for x in values):
            raise ValueError("trajectory inputs must be finite")


@dataclass(frozen=True)
class TrajectoryPoint:
    time_s: float
    phase: str
    waypoint: bool
    xyz_mm: tuple[float, float, float] | None
    velocity_mm_s: tuple[float, float, float] | None
    acceleration_mm_s2: tuple[float, float, float] | None
    q_rad: tuple[float, ...] | None
    dq_rad_s: tuple[float, ...] | None
    ddq_rad_s2: tuple[float, ...] | None


@dataclass(frozen=True)
class TrajectoryPlan:
    config: TrajectoryConfig
    points: tuple[TrajectoryPoint, ...]
    waypoint_indices: tuple[int, ...]

    def as_dict(self) -> dict:
        return {"config": asdict(self.config), "waypoint_indices": self.waypoint_indices,
                "points": [asdict(point) for point in self.points]}


def _quintic(u: float) -> tuple[float, float, float]:
    return (10 * u**3 - 15 * u**4 + 6 * u**5,
            30 * u**2 - 60 * u**3 + 30 * u**4,
            60 * u - 180 * u**2 + 120 * u**3)


def _cartesian_waypoints(config: TrajectoryConfig) -> list[tuple[float, float, float]]:
    start = config.start_mm
    if config.end_mm is not None:
        end = config.end_mm
    else:
        size = DEFAULT_SIZES_MM[config.trajectory_id]
        end = tuple(start[i] + size[i] for i in range(3))
    if config.trajectory_id in ("T1", "T2"):
        offsets = [start, end, start]
    else:
        x, y, z = (end[i] - start[i] for i in range(3))
        offsets = [start, (start[0] + x, start[1], start[2]),
                   (start[0] + x, start[1] + y, start[2]), (start[0], start[1] + y, start[2]),
                   (start[0], start[1] + y, start[2] + z),
                   (start[0] + x, start[1] + y, start[2] + z),
                   (start[0] + x, start[1], start[2] + z), (start[0], start[1], start[2] + z), start]
    return [tuple(config.reference_tcp_mm[i] + p[i] for i in range(3)) for p in offsets]


def _joint_waypoints(config: TrajectoryConfig) -> list[tuple[float, ...]]:
    centre = config.joint_start_rad
    amplitude = math.radians(config.joint_amplitude_deg)
    indices = (config.joint_indices,) if config.joint_mode == "simultaneous" else tuple((i,) for i in config.joint_indices)
    result = [centre]
    for active in indices:
        for sign in (1, -1, 0):
            result.append(tuple(centre[i] + (sign * amplitude if i in active else 0.0) for i in range(6)))
    return result


def generate(config: TrajectoryConfig) -> TrajectoryPlan:
    config.validate()
    joint = config.trajectory_id == "T4"
    waypoints = _joint_waypoints(config) if joint else _cartesian_waypoints(config)
    segment_seconds = config.segment_seconds or (4.0 if config.speed == "low" else 2.0)
    n = max(2, round(segment_seconds * config.sample_hz))
    dt = segment_seconds / n
    points: list[TrajectoryPoint] = []
    waypoint_indices: list[int] = []
    zero = (0.0,) * (6 if joint else 3)

    def append(t: float, phase: str, value: tuple[float, ...], velocity: tuple[float, ...],
               acceleration: tuple[float, ...], waypoint: bool = False) -> None:
        points.append(TrajectoryPoint(round(t, 9), phase, waypoint,
                                      None if joint else value, None if joint else velocity,
                                      None if joint else acceleration, value if joint else None,
                                      velocity if joint else None, acceleration if joint else None))
        if waypoint:
            waypoint_indices.append(len(points) - 1)

    pre_n = round(config.pre_hold_seconds * config.sample_hz)
    for k in range(pre_n + 1):
        append(k / config.sample_hz, "pre_hold", waypoints[0], zero, zero, k == 0)
    t0 = pre_n / config.sample_hz
    for repeat in range(config.repeats):
        for a, b in zip(waypoints, waypoints[1:]):
            delta = tuple(b[i] - a[i] for i in range(len(a)))
            for k in range(1, n + 1):
                u = k / n
                s, ds, dds = _quintic(u)
                append(t0 + k * dt, f"motion_{repeat + 1}",
                       tuple(a[i] + delta[i] * s for i in range(len(a))),
                       tuple(d * ds / segment_seconds for d in delta),
                       tuple(d * dds / segment_seconds**2 for d in delta), k == n)
            t0 += segment_seconds
    post_n = round(config.post_hold_seconds * config.sample_hz)
    for k in range(1, post_n + 1):
        append(t0 + k / config.sample_hz, "post_hold", waypoints[-1], zero, zero, k == post_n)
    return TrajectoryPlan(config, tuple(points), tuple(waypoint_indices))


def generate_many(config: TrajectoryConfig, trajectory_ids: Iterable[str]) -> dict[str, TrajectoryPlan]:
    from dataclasses import replace
    return {trajectory_id: generate(replace(config, trajectory_id=trajectory_id)) for trajectory_id in trajectory_ids}
