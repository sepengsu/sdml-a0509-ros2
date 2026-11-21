"""Read-only check of the RT values needed for sim/real gap analysis."""

from __future__ import annotations

import argparse
import json
import math
import time

from .trajectory import Trajectory, load_trajectory

FIELDS = {
    "actual_joint_position": (6, "required", "deg"),
    "actual_joint_velocity": (6, "required", "deg/s"),
    "actual_joint_torque": (6, "review_required", "Nm; controller estimate"),
    "actual_motor_torque": (6, "important", "Nm; current/gear conversion"),
    "gravity_torque": (6, "important", "Nm"),
    "time_stamp": (1, "important", "controller acquisition timestamp"),
}
SPLIT_TYPE = "std_msgs/msg/Float32MultiArray"
COMPOSITE_TYPE = "dsr_msgs2/msg/RobotStateRt"


def validate_values(field: str, raw) -> list[float]:
    expected = FIELDS[field][0]
    if field == "time_stamp" and isinstance(raw, (int, float)):
        raw = [raw]
    try:
        values = [float(item) for item in raw]
    except (TypeError, ValueError) as error:
        raise ValueError(f"{field}: nonnumeric data") from error
    if len(values) != expected or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{field}: expected {expected} finite values")
    if field == "time_stamp" and values[0] < 0:
        raise ValueError("time_stamp must be nonnegative")
    return values


def compare_trajectory_start(report: dict, trajectory: Trajectory) -> dict:
    """Compare a real RT snapshot with the JSON start; never authorize motion."""
    fields = report["fields"]
    q_deg = fields["actual_joint_position"]["values"]
    dq_deg_s = fields["actual_joint_velocity"]["values"]
    if q_deg is None or dq_deg_s is None:
        return {"trajectory": trajectory.name, "status": "missing_position_or_velocity",
                "start_pose_match": False, "stationary": False}
    error = [abs(math.radians(q_deg[i]) - trajectory.points[0].q_rad[i]) for i in range(6)]
    speed = [abs(math.radians(value)) for value in dq_deg_s]
    match = max(error) <= 0.05
    stationary = max(speed) <= 0.05
    return {"trajectory": trajectory.name,
            "status": "review" if match and stationary else "mismatch",
            "max_start_error_rad": max(error), "max_speed_rad_s": max(speed),
            "start_pose_match": match, "stationary": stationary,
            "real_motion_validated": False}


def inspect_topics(timeout_s: float = 5.0) -> dict:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray

    rclpy.init()
    node = Node("sim_real_gap_rt_check")
    received: dict[str, tuple[str, list[float]]] = {}
    invalid: dict[str, str] = {}
    subscriptions = []
    composite_subscribed = set()

    def publisher_types(topic: str) -> list[str]:
        return sorted({endpoint.topic_type for endpoint in node.get_publishers_info_by_topic(topic)})

    def capture(field: str, raw, source: str) -> None:
        try:
            received[field] = (source, validate_values(field, raw))
            invalid.pop(field, None)
        except ValueError as error:
            invalid[field] = str(error)

    for field in FIELDS:
        topic = f"/rt_topic/{field}"
        subscriptions.append(node.create_subscription(
            Float32MultiArray, topic,
            lambda msg, field=field, topic=topic: capture(field, msg.data, topic), 10))

    try:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            graph = dict(node.get_topic_names_and_types())
            for topic, types in graph.items():
                if (COMPOSITE_TYPE in types and COMPOSITE_TYPE in publisher_types(topic)
                        and topic not in composite_subscribed):
                    from dsr_msgs2.msg import RobotStateRt

                    def on_composite(msg, topic=topic):
                        for field in FIELDS:
                            capture(field, getattr(msg, field), topic)

                    subscriptions.append(node.create_subscription(RobotStateRt, topic, on_composite, 10))
                    composite_subscribed.add(topic)
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0.0, deadline-time.monotonic())))
            if len(received) == len(FIELDS):
                break
        graph = dict(node.get_topic_names_and_types())
        composite_topics = [topic for topic, types in graph.items()
                            if COMPOSITE_TYPE in types and COMPOSITE_TYPE in publisher_types(topic)]
        fields = {}
        for field, (_, priority, unit) in FIELDS.items():
            topic = f"/rt_topic/{field}"
            types = publisher_types(topic)
            source, values = received.get(field, (None, None))
            if values is not None:
                status = "ok"
            elif field in invalid:
                status = "invalid"
            elif types and SPLIT_TYPE not in types and not composite_topics:
                status = "wrong_type"
            elif SPLIT_TYPE in types or composite_topics:
                status = "no_data"
            else:
                status = "missing"
            fields[field] = {
                "priority": priority, "unit": unit, "status": status,
                "topic_present": bool(types or composite_topics), "topic_types": types,
                "source": source, "values": values,
                "detail": invalid.get(field),
            }
        return {
            "message_note": "Fields belong to dsr_msgs2/RobotStateRt; dsr_msgs2/RobotState has different field names",
            "topic_sources": {"split": "/rt_topic/<field>", "composite": composite_topics},
            "fields": fields,
            "all_fields_available": all(item["status"] == "ok" for item in fields.values()),
            "real_motion_validated": False,
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Inspect A0509 real RT fields without sending motion")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--trajectory", help="JSON trajectory to compare with the current real start pose")
    args = parser.parse_args(argv)
    if not 0 < args.timeout <= 60:
        parser.error("--timeout must be >0 and <=60")
    trajectory = None
    if args.trajectory:
        try:
            trajectory = load_trajectory(args.trajectory)
        except (OSError, ValueError, json.JSONDecodeError) as error:
            parser.error(f"invalid trajectory: {error}")
    report = inspect_topics(args.timeout)
    if trajectory is not None:
        report["trajectory_start"] = compare_trajectory_start(report, trajectory)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    start_ok = trajectory is None or report["trajectory_start"]["status"] == "review"
    return 0 if report["all_fields_available"] and start_ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
