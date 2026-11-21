"""Load a six-joint JSON trajectory and check its sampled simulation path."""

from __future__ import annotations

import argparse
from bisect import bisect_right
from dataclasses import dataclass
import json
import math
from pathlib import Path

JOINT_NAMES = tuple(f"joint_{i}" for i in range(1, 7))
GRIPPER_TYPES = ("none", "ag145", "mdpg_1s1")
# Conservative A0509 experiment limits; J5 includes the 115.02 deg practice Home.
POSITION_LIMITS = ((-3.14, 3.14), (-1.3, 1.3), (-2.0, 2.0),
                   (-3.14, 3.14), (-2.1, 2.1), (-3.14, 3.14))
VELOCITY_LIMITS = (3.67, 3.32, 3.67, 6.98, 6.98, 10.47)
ACCELERATION_LIMITS = (0.734, 0.664, 0.734, 1.396, 1.396, 2.094)


@dataclass(frozen=True)
class Point:
    time_s: float
    q_rad: tuple[float, ...]
    dq_rad_s: tuple[float, ...]
    ddq_rad_s2: tuple[float, ...]


@dataclass(frozen=True)
class Trajectory:
    name: str
    source: Path
    points: tuple[Point, ...]
    samples: tuple[Point, ...]
    gripper_type: str | None = None

    @property
    def duration_s(self) -> float:
        return self.points[-1].time_s


def _six(raw, label: str) -> tuple[float, ...]:
    if not isinstance(raw, (list, tuple)) or len(raw) != 6:
        raise ValueError(f"{label} must contain six joint values")
    try:
        values = tuple(float(value) for value in raw)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{label} contains a nonnumeric value") from error
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{label} contains a nonfinite value")
    return values


def _point(raw: dict, index: int) -> Point:
    if not isinstance(raw, dict):
        raise ValueError(f"points[{index}] must be an object")
    try:
        timestamp = float(raw.get("time_s", raw.get("time_from_start")))
    except (TypeError, ValueError) as error:
        raise ValueError(f"points[{index}] needs time_s") from error
    if not math.isfinite(timestamp):
        raise ValueError(f"points[{index}] time must be finite")
    return Point(
        timestamp,
        _six(raw.get("q_rad", raw.get("positions")), f"points[{index}].q_rad"),
        _six(raw.get("dq_rad_s", raw.get("velocities")), f"points[{index}].dq_rad_s"),
        _six(raw.get("ddq_rad_s2", raw.get("accelerations")), f"points[{index}].ddq_rad_s2"),
    )


def _interpolate(a: Point, b: Point, time_s: float) -> Point:
    duration = b.time_s - a.time_s
    u = (time_s - a.time_s) / duration
    positions, velocities, accelerations = [], [], []
    for joint in range(6):
        c0 = a.q_rad[joint]
        c1 = a.dq_rad_s[joint] * duration
        c2 = 0.5 * a.ddq_rad_s2[joint] * duration**2
        delta_q = b.q_rad[joint] - c0 - c1 - c2
        delta_v = b.dq_rad_s[joint] * duration - c1 - 2*c2
        delta_a = b.ddq_rad_s2[joint] * duration**2 - 2*c2
        c3 = 10*delta_q - 4*delta_v + 0.5*delta_a
        c4 = -15*delta_q + 7*delta_v - delta_a
        c5 = 6*delta_q - 3*delta_v + 0.5*delta_a
        positions.append(c0 + c1*u + c2*u**2 + c3*u**3 + c4*u**4 + c5*u**5)
        velocities.append((c1 + 2*c2*u + 3*c3*u**2 + 4*c4*u**3 + 5*c5*u**4) / duration)
        accelerations.append((2*c2 + 6*c3*u + 12*c4*u**2 + 20*c5*u**3) / duration**2)
    return Point(time_s, tuple(positions), tuple(velocities), tuple(accelerations))


def _check_limits(point: Point, label: str) -> None:
    for joint in range(6):
        if not POSITION_LIMITS[joint][0] <= point.q_rad[joint] <= POSITION_LIMITS[joint][1]:
            raise ValueError(f"{label}: joint_{joint+1} position exceeds the simulation limit")
        if abs(point.dq_rad_s[joint]) > VELOCITY_LIMITS[joint] + 1e-8:
            raise ValueError(f"{label}: joint_{joint+1} velocity exceeds the simulation limit")
        if abs(point.ddq_rad_s2[joint]) > ACCELERATION_LIMITS[joint] + 1e-8:
            raise ValueError(f"{label}: joint_{joint+1} acceleration exceeds the simulation limit")


def load_trajectory(path: str | Path, sample_hz: int = 100) -> Trajectory:
    source = Path(path).expanduser().resolve()
    data = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("points"), list):
        raise ValueError("JSON must be an object with a points array")
    names = data.get("joint_names", JOINT_NAMES)
    if not isinstance(names, (list, tuple)) or tuple(names) != JOINT_NAMES:
        raise ValueError("joint_names must be joint_1 through joint_6 in order")
    if data.get("units", "rad") != "rad":
        raise ValueError("trajectory joint units must be rad")
    gripper_type = data.get("gripper_type")
    if gripper_type is not None and gripper_type not in GRIPPER_TYPES:
        raise ValueError("gripper_type must be none, ag145 or mdpg_1s1")
    if sample_hz < 20 or sample_hz > 500:
        raise ValueError("sample_hz must be 20..500")
    if not 2 <= len(data["points"]) <= 100000:
        raise ValueError("trajectory needs 2..100000 points")
    points = tuple(_point(raw, i) for i, raw in enumerate(data["points"]))
    if abs(points[0].time_s) > 1e-9:
        raise ValueError("trajectory must start at time_s=0")
    if any(b.time_s <= a.time_s for a, b in zip(points, points[1:])):
        raise ValueError("trajectory times must increase strictly")
    if points[-1].time_s > 600:
        raise ValueError("trajectory duration must be at most 600 s")
    for end in (points[0], points[-1]):
        if any(abs(value) > 1e-6 for value in (*end.dq_rad_s, *end.ddq_rad_s2)):
            raise ValueError("trajectory must start and end at rest")
    for index, point in enumerate(points):
        _check_limits(point, f"points[{index}]")
    # A quintic joins position, velocity and acceleration at each JSON point.
    # Checking the interpolated states catches overshoot between sparse waypoints.
    samples = []
    step = 1.0 / sample_hz
    for index, (a, b) in enumerate(zip(points, points[1:])):
        count = max(1, math.ceil((b.time_s - a.time_s) / step))
        for sample_index in range(count):
            point = _interpolate(a, b, a.time_s + (b.time_s-a.time_s)*sample_index/count)
            _check_limits(point, f"segment {index} at {point.time_s:.3f}s")
            samples.append(point)
    samples.append(points[-1])
    config = data.get("config")
    if config is not None and not isinstance(config, dict):
        raise ValueError("config must be an object when provided")
    name = str(data.get("name") or (config or {}).get("trajectory_id") or source.stem)
    return Trajectory(name, source, points, tuple(samples), gripper_type)


def save_for_gripper(trajectory: Trajectory, destination: str | Path, gripper_type: str) -> Path:
    """Save the validated JSON with an explicit gripper label for later RT review."""
    if gripper_type not in GRIPPER_TYPES:
        raise ValueError("unsupported gripper_type")
    target = Path(destination).expanduser().resolve()
    if target.suffix.lower() != ".json":
        raise ValueError("destination must end in .json")
    data = json.loads(trajectory.source.read_text(encoding="utf-8"))
    data["gripper_type"] = gripper_type
    data["physical_tool_verified"] = False
    target.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return target


def resample_uniform(trajectory: Trajectory, rate_hz: int = 100) -> tuple[Point, ...]:
    """Return every uniformly timed setpoint, including the final point."""
    if rate_hz != 100:
        raise ValueError("real practice requires exactly 100 Hz")
    count = round(trajectory.duration_s * rate_hz)
    if count < 1 or abs(count / rate_hz - trajectory.duration_s) > 1e-8:
        raise ValueError("trajectory duration must be a multiple of 0.01 s")
    times = [point.time_s for point in trajectory.points]
    output = []
    for index in range(count + 1):
        sample_time = index / rate_hz
        segment = min(len(times) - 2, bisect_right(times, sample_time) - 1)
        point = trajectory.points[-1] if index == count else _interpolate(
            trajectory.points[segment], trajectory.points[segment + 1], sample_time)
        _check_limits(point, f"100 Hz sample {index}")
        output.append(point)
    return tuple(output)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Validate an A0509 joint trajectory JSON for simulation")
    parser.add_argument("trajectory_file")
    args = parser.parse_args(argv)
    try:
        trajectory = load_trajectory(args.trajectory_file)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.exit(2, f"Invalid trajectory: {error}\n")
    print(json.dumps({"name": trajectory.name, "source": str(trajectory.source),
                      "gripper_type": trajectory.gripper_type,
                      "duration_s": trajectory.duration_s, "points": len(trajectory.points),
                      "checked_samples": len(trajectory.samples), "simulation_limits_passed": True,
                      "real_motion_validated": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
