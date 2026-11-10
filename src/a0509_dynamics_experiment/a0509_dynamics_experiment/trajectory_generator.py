"""Deterministic, robot-independent reference trajectories.

Cartesian coordinates are base-frame millimetres. Joint coordinates are radians.
The suggested dimensions are research examples, not validated robot paths.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import math
from typing import Iterable

JOINT_NAMES = tuple(f"joint_{i}" for i in range(1, 7))
DEFAULT_SIZES_MM = {"T1": (120.0, 0.0, 0.0), "T2": (0.0, 80.0, 60.0), "T3": (120.0, 80.0, 60.0)}
CARTESIAN_IDS = ("T1", "T2", "T3")
DYNAMICS10_IDS = tuple(f"T{i}" for i in range(5, 15))
EXTENDED_IDS = ("T15", "T16")
EXCITATION_IDS = (*DYNAMICS10_IDS, *EXTENDED_IDS)
JOINT_IDS = ("T4", *EXCITATION_IDS)
TRAJECTORY_IDS = (*CARTESIAN_IDS, *JOINT_IDS)
BASE_HOME = (-0.5, -0.5, 1.6, 0.0, -0.6, 0.0)

# (joint index, amplitude deg, f0 Hz, f1 Hz, phase rad, window start/end s).
# A differing f1 selects a linear chirp. These are research starting values.
EXCITATIONS = {
    "T5": {"name": "J2/J3 alternating sine", "duration": 12.0, "split": "train",
           "components": ((1, 5.0, .17, .17, .2, 0., 6.), (2, 5.0, .23, .23, .8, 6., 12.))},
    "T6": {"name": "J1/J4 coupled sine", "duration": 12.0, "split": "train",
           "components": ((0, 5.0, .14, .14, .1, 0., 12.), (3, 5.0, .26, .26, .9, 0., 12.))},
    "T7": {"name": "J5/J6 wrist excitation", "duration": 12.0, "split": "train",
           "components": ((4, 5.0, .32, .32, .4, 0., 12.), (5, 4.0, .38, .38, 1.2, 0., 12.))},
    "T8": {"name": "J1/J3/J5 multi-sine", "duration": 14.0, "split": "train",
           "components": ((0, 3.5, .12, .12, .1, 0., 14.), (0, 1.5, .27, .27, 1.1, 0., 14.),
                          (2, 3.5, .16, .16, .7, 0., 14.), (2, 1.5, .25, .25, 1.8, 0., 14.),
                          (4, 3.5, .21, .21, .4, 0., 14.), (4, 1.5, .34, .34, 2.1, 0., 14.))},
    "T9": {"name": "J2/J4/J6 multi-sine", "duration": 14.0, "split": "train",
           "components": ((1, 3.5, .13, .13, .5, 0., 14.), (1, 1.5, .24, .24, 1.6, 0., 14.),
                          (3, 3.5, .18, .18, .2, 0., 14.), (3, 1.5, .31, .31, 1.4, 0., 14.),
                          (5, 3.5, .22, .22, .9, 0., 14.), (5, 1.5, .36, .36, 2.3, 0., 14.))},
    "T10": {"name": "six-joint multi-sine", "duration": 16.0, "split": "train",
            "components": tuple((i, 4.0, .11 + .035*i, .11 + .035*i, .31*i, 0., 16.)
                                for i in range(6))},
    "T11": {"name": "J2/J3/J5 frequency sweep", "duration": 14.0, "split": "train",
            "components": ((1, 4.0, .10, .28, .1, 0., 14.), (2, 4.0, .13, .31, .8, 0., 14.),
                           (4, 3.0, .15, .36, 1.3, 0., 14.))},
    "T12": {"name": "J1/J4/J6 validation chirp", "duration": 14.0, "split": "validation",
            "components": ((0, 4.0, .12, .30, .3, 0., 14.), (3, 4.0, .14, .35, 1.1, 0., 14.),
                           (5, 3.0, .16, .39, 1.8, 0., 14.))},
    "T13": {"name": "shifted-posture multi-sine", "duration": 14.0, "split": "test",
            "home": (-.35, -.65, 1.35, .12, -.50, .08),
            "components": ((0, 4.0, .15, .15, .4, 0., 14.), (1, 3.0, .22, .22, 1.4, 0., 14.),
                           (2, 4.0, .18, .18, .8, 0., 14.), (4, 4.0, .29, .29, 1.9, 0., 14.))},
    "T14": {"name": "shifted-posture mixed excitation", "duration": 15.0, "split": "test",
            "home": (-.65, -.35, 1.45, -.16, -.45, -.10),
            "components": ((1, 5.0, .18, .18, .3, 0., 5.), (3, 5.0, .24, .24, .9, 5., 10.),
                           (0, 4.0, .16, .16, .2, 10., 15.), (4, 4.0, .28, .28, 1.1, 10., 15.),
                           (5, 3.0, .35, .35, 1.6, 10., 15.))},
    # New A0509 coverage designs inspired by the paper's state-space argument;
    # these are not trajectories specified or validated by Zhang et al.
    "T15": {"name": "low-speed reversals", "duration": 16.0, "split": "train",
            "components": ((0, 2.0, .050, .050, .1, 0., 8.),
                           (1, 2.0, .060, .060, .8, 0., 8.),
                           (2, 2.0, .070, .070, .4, 8., 16.),
                           (3, 1.5, .055, .055, 1.1, 8., 16.),
                           (4, 1.5, .080, .080, .3, 0., 16.),
                           (5, 1.5, .065, .065, 1.4, 0., 16.))},
    "T16": {"name": "held-out posture coupled chirp", "duration": 18.0, "split": "test",
            "home": (-.25, -.72, 1.25, .18, -.40, .12),
            "components": ((0, 4.0, .11, .25, .2, 0., 18.),
                           (1, 4.0, .08, .22, .9, 0., 18.),
                           (2, 3.0, .14, .28, 1.5, 0., 18.),
                           (3, 3.0, .12, .30, .4, 0., 18.),
                           (4, 3.0, .16, .32, 1.0, 0., 18.),
                           (5, 2.0, .18, .34, 1.8, 0., 18.))},
}
EXCITATION_SEED = 20261110  # no stochastic sample is drawn in this version


def is_joint_trajectory(trajectory_id: str) -> bool:
    return trajectory_id in JOINT_IDS


def excitation_metadata(trajectory_id: str) -> dict:
    spec = EXCITATIONS[trajectory_id]
    return {"name": spec["name"], "duration_s": spec["duration"], "split": spec["split"],
            "home_rad": spec.get("home", BASE_HOME), "seed": EXCITATION_SEED,
            "components": [{"joint": item[0]+1, "amplitude_deg": item[1], "f0_hz": item[2],
                            "f1_hz": item[3], "phase_rad": item[4], "window_s": item[5:]}
                           for item in spec["components"]]}


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
    time_scale: float = 1.0

    def __post_init__(self) -> None:
        if self.trajectory_id in EXCITATIONS and self.joint_start_rad == (0.0,) * 6:
            object.__setattr__(self, "joint_start_rad", tuple(EXCITATIONS[self.trajectory_id].get("home", BASE_HOME)))

    def validate(self) -> None:
        if self.trajectory_id not in TRAJECTORY_IDS:
            raise ValueError("unsupported trajectory_id")
        if self.speed not in ("low", "medium"):
            raise ValueError("speed must be low or medium")
        if self.repeats < 1 or self.sample_hz < 2:
            raise ValueError("repeats must be positive and sample_hz must be at least 2")
        if not math.isfinite(self.time_scale) or self.time_scale <= 0:
            raise ValueError("time_scale must be positive and finite")
        if abs(self.sample_hz * self.time_scale - round(self.sample_hz * self.time_scale)) > 1e-9:
            raise ValueError("sample_hz * time_scale must be an integer")
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


def _window(t: float, start: float, end: float) -> tuple[float, float, float]:
    """C2 finite-duration envelope and its first two time derivatives."""
    if t <= start or t >= end:
        return 0.0, 0.0, 0.0
    ramp = min(1.0, (end - start) / 3.0)
    if t < start + ramp:
        s, ds, dds = _quintic((t - start) / ramp)
        return s, ds / ramp, dds / ramp**2
    if t > end - ramp:
        s, ds, dds = _quintic((end - t) / ramp)
        return s, -ds / ramp, dds / ramp**2
    return 1.0, 0.0, 0.0


def _excitation_point(config: TrajectoryConfig, t: float, duration: float):
    q = list(config.joint_start_rad)
    dq, ddq = [0.0] * 6, [0.0] * 6
    for joint, amplitude_deg, f0, f1, phase, start, end in EXCITATIONS[config.trajectory_id]["components"]:
        start, end = start * duration / EXCITATIONS[config.trajectory_id]["duration"], end * duration / EXCITATIONS[config.trajectory_id]["duration"]
        envelope, de, dde = _window(t, start, end)
        elapsed = t - start
        chirp_rate = (f1 - f0) / (end - start)
        angle = 2 * math.pi * (f0 * elapsed + .5 * chirp_rate * elapsed**2) + phase
        omega = 2 * math.pi * (f0 + chirp_rate * elapsed)
        alpha = 2 * math.pi * chirp_rate
        amplitude = math.radians(amplitude_deg * config.joint_amplitude_deg / 5.0)
        sine, cosine = math.sin(angle), math.cos(angle)
        q[joint] += amplitude * envelope * sine
        dq[joint] += amplitude * (de * sine + envelope * omega * cosine)
        ddq[joint] += amplitude * (dde * sine + 2 * de * omega * cosine
                                   + envelope * (alpha * cosine - omega**2 * sine))
    return tuple(q), tuple(dq), tuple(ddq)


def _generate_excitation(config: TrajectoryConfig) -> TrajectoryPlan:
    duration = config.segment_seconds or EXCITATIONS[config.trajectory_id]["duration"]
    if config.speed == "medium" and config.segment_seconds is None:
        duration *= .8
    n = max(2, round(duration * config.sample_hz))
    duration = n / config.sample_hz
    points: list[TrajectoryPoint] = []
    waypoint_indices: list[int] = []

    def append(t, phase, q, dq, ddq, waypoint=False):
        points.append(TrajectoryPoint(round(t, 9), phase, waypoint, None, None, None, q, dq, ddq))
        if waypoint:
            waypoint_indices.append(len(points)-1)

    zero = (0.0,) * 6
    pre_n = round(config.pre_hold_seconds * config.sample_hz)
    for k in range(pre_n + 1):
        append(k / config.sample_hz, "pre_hold", config.joint_start_rad, zero, zero, k == 0)
    t0 = pre_n / config.sample_hz
    for repeat in range(config.repeats):
        for k in range(1, n + 1):
            t = duration * k / n
            q, dq, ddq = _excitation_point(config, t, duration)
            append(t0 + t, f"motion_{repeat+1}", q, dq, ddq, k == n)
        t0 += duration
    post_n = round(config.post_hold_seconds * config.sample_hz)
    for k in range(1, post_n + 1):
        append(t0 + k / config.sample_hz, "post_hold", config.joint_start_rad, zero, zero, k == post_n)
    return TrajectoryPlan(config, tuple(points), tuple(waypoint_indices))


def generate(config: TrajectoryConfig) -> TrajectoryPlan:
    config.validate()
    if config.time_scale != 1.0:
        factor = config.time_scale
        base = replace(config, time_scale=1.0,
                       sample_hz=round(config.sample_hz * factor),
                       pre_hold_seconds=config.pre_hold_seconds / factor,
                       post_hold_seconds=config.post_hold_seconds / factor)
        unscaled = generate(base)
        def scaled(values, divisor):
            return None if values is None else tuple(value / divisor for value in values)
        points = tuple(replace(point, time_s=round(index / config.sample_hz, 9),
                               velocity_mm_s=scaled(point.velocity_mm_s, factor),
                               acceleration_mm_s2=scaled(point.acceleration_mm_s2, factor * factor),
                               dq_rad_s=scaled(point.dq_rad_s, factor),
                               ddq_rad_s2=scaled(point.ddq_rad_s2, factor * factor))
                       for index, point in enumerate(unscaled.points))
        return TrajectoryPlan(config, points, unscaled.waypoint_indices)
    if config.trajectory_id in EXCITATION_IDS:
        return _generate_excitation(config)
    joint = is_joint_trajectory(config.trajectory_id)
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
