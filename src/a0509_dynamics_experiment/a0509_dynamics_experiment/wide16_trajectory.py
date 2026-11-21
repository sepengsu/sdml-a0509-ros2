"""Wide, repeated joint excitation for a new simulation-only 16-lane campaign.

The profile is independent of the original T1-T16 campaign. Every path makes
at least five complete back-and-forth cycles and has a smooth, stationary start/end.
The grid collector validates joint limits, contacts, torque saturation and
tracking before a scene is accepted. This is not a real-robot motion plan.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .trajectory_generator import TrajectoryPlan, TrajectoryPoint


BASE_HOME = (-0.5, -0.1, 0.3, 0.0, -0.2, 0.0)
SHIFT_A = (-0.35, -0.25, 0.2, 0.12, -0.1, 0.08)
SHIFT_B = (-0.65, 0.1, 0.4, -0.16, -0.1, -0.1)
FOURIER_HOME = (-0.25, -0.2, 0.25, 0.18, -0.1, 0.12)
SAMPLE_HZ = 100
DURATION_S = 450.0
CYCLES = 5
RAMP_S = 60.0
AMPLITUDE_SCALE = (3.0, 2.3, 2.3, 3.5, 3.5, 3.5)

# Components are (zero-based joint, amplitude in degrees, cycles, phase in rad).
# At the base posture, J3 is limited to 20 degrees to stay below its upper limit.
DESIGNS = {
    "T1": (BASE_HOME, ((0, 28, 5, 0.0), (1, 20, 6, 1.7))),
    "T2": (BASE_HOME, ((1, 25, 5, 0.0),)),
    "T3": (BASE_HOME, ((2, 20, 5, 0.0),)),
    "T4": (BASE_HOME, ((0, 22, 5, 0.0), (1, 14, 6, 1.8), (3, 14, 5, 1.2))),
    "T5": (BASE_HOME, ((1, 22, 5, 0.0), (4, 13, 6, 1.4))),
    "T6": (BASE_HOME, ((2, 18, 5, 0.0), (5, 14, 6, 1.6))),
    "T7": (BASE_HOME, ((0, 22, 5, 0.0), (1, 20, 6, 1.2))),
    "T8": (BASE_HOME, ((1, 21, 5, 0.0), (2, 17, 6, 2.0))),
    "T9": (BASE_HOME, ((0, 23, 5, 0.0), (1, 10, 6, 2.5), (2, 17, 5, 1.8))),
    "T10": (BASE_HOME, ((0, 19, 5, 0.0), (1, 18, 6, 1.4), (3, 12, 5, 2.2))),
    "T11": (BASE_HOME, ((1, 18, 5, 0.0), (2, 16, 6, 1.6), (4, 11, 5, 2.4))),
    "T12": (BASE_HOME, ((0, 20, 5, 0.0), (1, 10, 6, 2.5),
                         (2, 16, 5, 1.3), (5, 11, 6, 2.1))),
    "T13": (SHIFT_A, ((0, 20, 5, 0.0), (1, 18, 6, 1.1), (4, 10, 5, 2.0))),
    "T14": (SHIFT_B, ((1, 20, 5, 0.0), (2, 18, 6, 1.4), (3, 11, 5, 2.2))),
    "T15": (BASE_HOME, ((0, 18, 5, 0.0), (1, 17, 6, 1.0), (2, 14, 5, 2.0),
                        (3, 9, 6, 2.6), (4, 9, 5, 1.7), (5, 9, 6, 0.7))),
    "T16": (FOURIER_HOME, ((0, 18, 5, 0.0), (0, 4, 10, 1.0),
                           (1, 17, 5, 1.4), (1, 3, 10, 2.1),
                           (2, 15, 5, 2.2), (2, 3, 10, 0.6),
                           (3, 9, 5, 0.8), (3, 2, 10, 1.8),
                           (4, 9, 5, 1.9), (4, 2, 10, 0.4),
                           (5, 9, 5, 2.6), (5, 2, 10, 1.2))),
}
SPLITS = {**{f"T{i}": "train" for i in range(1, 13)},
          "T13": "validation", "T14": "validation", "T15": "test", "T16": "test"}


@dataclass(frozen=True)
class Wide16Config:
    trajectory_id: str
    profile: str
    joint_start_rad: tuple[float, ...]
    components: tuple[tuple[int, float, int, float], ...]
    duration_s: float = DURATION_S
    cycles: int = CYCLES
    ramp_s: float = RAMP_S
    pre_hold_seconds: float = 1.0
    post_hold_seconds: float = 1.0
    sample_hz: int = SAMPLE_HZ
    reference_tcp_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)


def _quintic(u: float) -> tuple[float, float, float]:
    return 10*u**3 - 15*u**4 + 6*u**5, 30*u**2 - 60*u**3 + 30*u**4, 60*u - 180*u**2 + 120*u**3


def _envelope(t: float) -> tuple[float, float, float]:
    if t < RAMP_S:
        value, velocity, accel = _quintic(t / RAMP_S)
        return value, velocity / RAMP_S, accel / RAMP_S**2
    if t > DURATION_S - RAMP_S:
        value, velocity, accel = _quintic((DURATION_S - t) / RAMP_S)
        return value, -velocity / RAMP_S, accel / RAMP_S**2
    return 1.0, 0.0, 0.0


def generate_wide(trajectory_id: str) -> TrajectoryPlan:
    if trajectory_id not in DESIGNS:
        raise ValueError(f"unknown wide16 trajectory: {trajectory_id}")
    home, base_components = DESIGNS[trajectory_id]
    components = tuple((joint, amplitude * AMPLITUDE_SCALE[joint], cycles, phase)
                       for joint, amplitude, cycles, phase in base_components)
    config = Wide16Config(trajectory_id, "wide16_v3", home, components)
    zero = (0.0,) * 6
    points = []
    waypoint_indices = []

    def append(time_s, phase, q, dq, ddq, waypoint=False):
        points.append(TrajectoryPoint(round(time_s, 9), phase, waypoint,
                                      None, None, None, q, dq, ddq))
        if waypoint:
            waypoint_indices.append(len(points) - 1)

    append(0.0, "pre_hold", home, zero, zero, True)
    for k in range(1, SAMPLE_HZ + 1):
        append(k / SAMPLE_HZ, "pre_hold", home, zero, zero)
    offset = 1.0
    for k in range(1, int(DURATION_S * SAMPLE_HZ) + 1):
        t = k / SAMPLE_HZ
        envelope, de, dde = _envelope(t)
        q, dq, ddq = list(home), [0.0] * 6, [0.0] * 6
        for joint, amplitude_deg, cycles, phase in components:
            amplitude = math.radians(amplitude_deg)
            omega = 2 * math.pi * cycles / DURATION_S
            angle = omega * t + phase
            sine, cosine = math.sin(angle), math.cos(angle)
            q[joint] += amplitude * envelope * sine
            dq[joint] += amplitude * (de * sine + envelope * omega * cosine)
            ddq[joint] += amplitude * (dde * sine + 2 * de * omega * cosine
                                       - envelope * omega**2 * sine)
        waypoint = k % (SAMPLE_HZ * int(DURATION_S / CYCLES)) == 0
        append(offset + t, "motion", tuple(q), tuple(dq), tuple(ddq), waypoint)
    for k in range(1, SAMPLE_HZ + 1):
        append(offset + DURATION_S + k / SAMPLE_HZ, "post_hold", home,
               zero, zero, k == SAMPLE_HZ)
    return TrajectoryPlan(config, tuple(points), tuple(waypoint_indices))
