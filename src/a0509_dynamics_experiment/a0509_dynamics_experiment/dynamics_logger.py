"""ROS-independent normalization and quality accounting for dynamics samples."""

from __future__ import annotations

from collections import defaultdict
import math
import time
from typing import Any, Sequence

from .trajectory_generator import JOINT_NAMES

RT_FIELDS = ("actual_joint_position", "actual_joint_velocity", "actual_motor_torque",
             "actual_joint_torque", "external_joint_torque", "actual_tcp_position",
             "target_joint_position", "joint_temperature", "time_stamp", "robot_state", "singularity")
TORQUE_DEFINITIONS = {
    "actual_motor_torque": "DRFL motor-current/gear conversion [Nm], not an independent torque sensor",
    "actual_joint_torque": "DRFL controller-estimated joint torque [Nm], not an independent torque sensor",
    "external_joint_torque": "DRFL controller-estimated external joint torque [Nm]",
    "qfrc_inverse": "MuJoCo inverse-dynamics generalized force [Nm for hinge DOFs]",
    "qfrc_actuator": "MuJoCo actuator generalized force [Nm for hinge DOFs] when directly available",
    "qfrc_applied": "MuJoCo externally applied generalized force [Nm for hinge DOFs] if ROS JointState effort is exported",
    "qfrc_bias": "MuJoCo bias generalized force [Nm for hinge DOFs]",
}


def ordered(values: Sequence[float], names: Sequence[str]) -> tuple[float, ...]:
    if len(values) != len(names) or len(set(names)) != len(names):
        raise ValueError("joint state names and values must have matching unique lengths")
    by_name = dict(zip(names, values))
    if any(name not in by_name for name in JOINT_NAMES):
        raise ValueError("joint state missing A0509 joints")
    return tuple(float(by_name[name]) for name in JOINT_NAMES)


class DynamicsRecorder:
    def __init__(self, mode: str, trajectory_id: str, trial_id: str, gripper_type: str,
                 payload_mass: float, sample_hz: int = 100, max_age_s: float = 0.03):
        if mode not in ("simulation", "real"):
            raise ValueError("recorder mode must be simulation or real")
        self.mode = mode
        self.trajectory_id = trajectory_id
        self.trial_id = trial_id
        self.gripper_type = gripper_type
        self.payload_mass = payload_mass
        self.sample_hz = sample_hz
        self.max_age_s = max_age_s
        self.joint: dict[str, Any] | None = None
        self.rt: dict[str, tuple[float, tuple[float, ...]]] = {}
        self.topic_arrivals: dict[str, list[float]] = defaultdict(list)
        self.topic_source_times: dict[str, list[float]] = defaultdict(list)
        self.rows: list[dict[str, Any]] = []
        self._previous_real_dq: tuple[float, ...] | None = None
        self._previous_real_time: float | None = None

    def ingest_joint_state(self, names: Sequence[str], position: Sequence[float],
                           velocity: Sequence[float] | None = None, effort: Sequence[float] | None = None,
                           arrival_s: float | None = None, source_time_s: float | None = None) -> None:
        now = time.monotonic() if arrival_s is None else arrival_s
        q = ordered(position, names)
        dq = ordered(velocity, names) if velocity and len(velocity) == len(names) else None
        tau = ordered(effort, names) if effort and len(effort) == len(names) else None
        self.joint = {"arrival": now, "q": q, "dq": dq, "effort": tau}
        self.topic_arrivals["joint_states"].append(now)
        if source_time_s is not None and math.isfinite(source_time_s) and source_time_s > 0:
            self.topic_source_times["joint_states"].append(source_time_s)

    def ingest_rt(self, key: str, values: Sequence[float], arrival_s: float | None = None) -> None:
        if key not in RT_FIELDS:
            raise ValueError(f"unsupported RT key: {key}")
        expected = 1 if key in ("time_stamp", "robot_state", "singularity") else 6
        if len(values) != expected or not all(math.isfinite(float(v)) for v in values):
            raise ValueError(f"{key} requires {expected} finite values")
        now = time.monotonic() if arrival_s is None else arrival_s
        self.rt[key] = (now, tuple(float(v) for v in values))
        self.topic_arrivals[key].append(now)

    def _recent_rt(self, key: str, now: float) -> tuple[float, ...] | None:
        value = self.rt.get(key)
        return value[1] if value and 0 <= now - value[0] <= self.max_age_s else None

    def snapshot(self, timestamp: float, ros_time_s: float, target_q: Sequence[float] | None = None,
                 physics: dict[str, Any] | None = None, arrival_s: float | None = None) -> dict[str, Any]:
        now = time.monotonic() if arrival_s is None else arrival_s
        row: dict[str, Any] = {"timestamp": timestamp, "ros_time_s": ros_time_s,
                               "trajectory_id": self.trajectory_id, "trial_id": self.trial_id,
                               "gripper_type": self.gripper_type, "payload_mass": self.payload_mass,
                               "mode": self.mode}
        if self.mode == "real":
            for key in RT_FIELDS:
                row[key] = self._recent_rt(key, now)
            rt_timestamp = row.get("time_stamp")
            row["rt_time_stamp"] = rt_timestamp[0] if rt_timestamp else None
            position_deg = row.get("actual_joint_position")
            velocity_deg = row.get("actual_joint_velocity")
            q = tuple(math.radians(v) for v in position_deg) if position_deg else None
            dq = tuple(math.radians(v) for v in velocity_deg) if velocity_deg else None
            target_deg = row.get("target_joint_position")
            target_q = tuple(math.radians(v) for v in target_deg) if target_deg else target_q
            tcp = row.get("actual_tcp_position")
            row["tcp_position"] = tcp[:3] if tcp else None
            ddq = None
            if dq and self._previous_real_dq and self._previous_real_time is not None and now > self._previous_real_time:
                delta_t = now - self._previous_real_time
                ddq = tuple((dq[i] - self._previous_real_dq[i]) / delta_t for i in range(6))
            if dq:
                self._previous_real_dq, self._previous_real_time = dq, now
        else:
            fresh = self.joint and 0 <= now - self.joint["arrival"] <= self.max_age_s
            q = self.joint["q"] if fresh else None
            dq = self.joint["dq"] if fresh else None
            ddq = physics.get("qacc") if physics else None
            row["tcp_position"] = physics.get("tcp_position") if physics else None
            if physics:
                row.update(physics)
            # JointState.effort has no guaranteed qfrc_actuator semantics in the upstream bridge.
            row.setdefault("qfrc_actuator", None)
        for stem, values in (("q", q), ("dq", dq), ("ddq", ddq), ("target_q", target_q)):
            if values is not None:
                for i, value in enumerate(values[:6], 1):
                    row[f"{stem}{i}"] = float(value)
        self.rows.append(row)
        return row

    def quality(self) -> dict[str, Any]:
        def metrics(stamps: list[float]) -> dict[str, Any]:
            gaps = [b - a for a, b in zip(stamps, stamps[1:]) if b > a]
            duration = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
            expected = max(1, round(duration * self.sample_hz) + 1)
            return {"received": len(stamps),
                    "observed_hz": (len(stamps) - 1) / duration if duration > 0 else None,
                    "gap_estimated_missing": sum(max(0, round(g * self.sample_hz) - 1) for g in gaps),
                    "estimated_missing_rate": max(0, expected - len(stamps)) / expected,
                    "max_gap_s": max(gaps) if gaps else None}

        result: dict[str, Any] = {}
        for key, stamps in self.topic_arrivals.items():
            wall = metrics(stamps)
            source = self.topic_source_times.get(key, [])
            if self.mode == "simulation" and len(source) > 1:
                result[key] = {**metrics(source), "time_basis": "message_header_sim_time",
                               "wall_arrival_hz": wall["observed_hz"],
                               "wall_arrival_max_gap_s": wall["max_gap_s"]}
            else:
                result[key] = {**wall, "time_basis": "arrival_wall_time"}
        result["rows_written"] = len(self.rows)
        result["blank_field_counts"] = {
            key: sum(row.get(key) is None for row in self.rows)
            for key in ("q1", "dq1", "ddq1", "tcp_position", "actual_motor_torque", "qfrc_inverse")
        }
        return result
