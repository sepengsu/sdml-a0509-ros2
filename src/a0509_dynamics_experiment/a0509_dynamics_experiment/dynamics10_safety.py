"""Conservative MuJoCo-only preflight for joint excitation experiments.

Limits mirror a0509_mujoco_sim/config/joint_limits.yaml. This is not a
certification for real-robot motion, which remains disabled for T5-T14.
"""

from __future__ import annotations

import math

from .trajectory_generator import TrajectoryPlan

POSITION_LIMITS = ((-3.14, 3.14), (-1.3, 1.3), (-2.0, 2.0),
                   (-3.14, 3.14), (-2.0, 2.0), (-3.14, 3.14))
VELOCITY_LIMITS = (3.67, 3.32, 3.67, 6.98, 6.98, 10.47)
ACCELERATION_LIMITS = (.734, .664, .734, 1.396, 1.396, 2.094)


def validate_joint_plan(plan: TrajectoryPlan, observer=None) -> dict:
    """Check every sampled state, analytic derivative and MuJoCo contact."""
    if not plan.points or plan.points[0].q_rad is None:
        raise ValueError("joint-space plan required")
    peak_velocity = [0.0] * 6
    peak_acceleration = [0.0] * 6
    min_position = [math.inf] * 6
    max_position = [-math.inf] * 6
    last_time = -math.inf
    for index, point in enumerate(plan.points):
        if point.time_s <= last_time:
            raise ValueError(f"non-increasing trajectory time at sample {index}")
        last_time = point.time_s
        for joint in range(6):
            q, dq, ddq = point.q_rad[joint], point.dq_rad_s[joint], point.ddq_rad_s2[joint]
            if not all(math.isfinite(value) for value in (q, dq, ddq)):
                raise ValueError(f"nonfinite joint state at sample {index}")
            low, high = POSITION_LIMITS[joint]
            if not low <= q <= high:
                raise ValueError(f"J{joint+1} position limit exceeded at sample {index}")
            if abs(dq) > VELOCITY_LIMITS[joint] or abs(ddq) > ACCELERATION_LIMITS[joint]:
                raise ValueError(f"J{joint+1} velocity/acceleration limit exceeded at sample {index}")
            peak_velocity[joint] = max(peak_velocity[joint], abs(dq))
            peak_acceleration[joint] = max(peak_acceleration[joint], abs(ddq))
            min_position[joint] = min(min_position[joint], q)
            max_position[joint] = max(max_position[joint], q)
        if observer is not None:
            observer.require_contact_free(point.q_rad)
    if any(abs(value) > 1e-8 for value in (*plan.points[0].dq_rad_s,
                                            *plan.points[-1].dq_rad_s,
                                            *plan.points[0].ddq_rad_s2,
                                            *plan.points[-1].ddq_rad_s2)):
        raise ValueError("trajectory does not start and stop at rest")
    return {"validated_samples": len(plan.points), "contact_checks": len(plan.points) if observer else 0,
            "position_min_rad": min_position, "position_max_rad": max_position,
            "peak_velocity_rad_s": peak_velocity, "peak_acceleration_rad_s2": peak_acceleration,
            "position_limits_rad": POSITION_LIMITS, "velocity_limits_rad_s": VELOCITY_LIMITS,
            "acceleration_limits_rad_s2": ACCELERATION_LIMITS,
            "limits_source": "a0509_mujoco_sim/config/joint_limits.yaml",
            "valid_for_real_robot": False}
