"""Guarded 100 Hz ROS/DRFL setpoint streaming for an operator-confirmed real A0509.

This process runs in the real ROS domain, separate from the MuJoCo GUI node.
It publishes every uniformly resampled JSON setpoint in order. Missed timing,
stale feedback, unexpected robot state or excessive tracking error aborts the
stream and requests a quick stop. This is a practice tool, not a certified RT
controller or a full workspace/collision validator.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import signal
import threading
import time

from .rt_check import FIELDS, inspect_topics, validate_values
from .trajectory import load_trajectory, resample_uniform

PERIOD_S = 0.01
SERVO_TIME_S = 0.03  # Above the 10 ms send period and recent 25 ms controller minimum.
MAX_LATE_S = 0.003
MAX_FEEDBACK_AGE_S = 0.1
MAX_TRACKING_ERROR_RAD = 0.08
MAX_COMMAND_VELOCITY_DEG_S = 30.0
MAX_COMMAND_ACCELERATION_DEG_S2 = 30.0
RT_INPUT_LOSS_COUNT = 50  # 0.5 s controller watchdog; software feedback guard is 0.1 s.


def validate_real_plan(trajectory, gripper_type: str, *, supervised_trial: bool = False):
    source_data = json.loads(trajectory.source.read_text(encoding="utf-8"))
    if supervised_trial:
        if (source_data.get("execution_scope") != "supervised_real_trial" or
                source_data.get("real_motion_validated") is not False or
                not source_data.get("physical_preflight_report")):
            raise ValueError("supervised trial requires a prepared physical preflight trajectory")
        report_path = Path(source_data["physical_preflight_report"]).resolve()
        report = json.loads(report_path.read_text(encoding="utf-8"))
        if (report.get("status") != "ready_for_supervised_trial" or
                report.get("tool") != gripper_type or
                not report.get("ready_for_supervised_trial")):
            raise ValueError("supervised trial preflight report is not ready for this gripper")
        scene_path = Path(report["arm_scene"]).resolve()
        if hashlib.sha256(scene_path.read_bytes()).hexdigest() != report["arm_scene_sha256"]:
            raise ValueError("collision scene differs from physical preflight")
        entry = next((item for item in report["files"].values()
                      if Path(item["path"]).resolve() == trajectory.source.resolve()), None)
        if entry is None or entry["sha256"] != hashlib.sha256(trajectory.source.read_bytes()).hexdigest():
            raise ValueError("supervised trial trajectory differs from the preflight copy")
    elif (source_data.get("execution_scope") == "simulation_only" or
          source_data.get("real_motion_validated") is False):
        raise ValueError("this trajectory is simulation-only; real motion validation is incomplete")
    if trajectory.gripper_type != gripper_type:
        raise ValueError("JSON gripper_type must match the selected gripper")
    samples = resample_uniform(trajectory, 100)
    peak_v = max(abs(math.degrees(value)) for point in samples for value in point.dq_rad_s)
    peak_a = max(abs(math.degrees(value)) for point in samples for value in point.ddq_rad_s2)
    if peak_v > MAX_COMMAND_VELOCITY_DEG_S or peak_a > MAX_COMMAND_ACCELERATION_DEG_S2:
        raise ValueError("real practice is limited to 30 deg/s and 30 deg/s² per joint")
    return samples


def _call(node, service_type, name: str, *, require_success: bool = True,
          request_fields: dict | None = None):
    import rclpy

    client = node.create_client(service_type, name)
    if not client.wait_for_service(timeout_sec=2.0):
        raise RuntimeError(f"service unavailable: {name}")
    request = service_type.Request()
    for field, value in (request_fields or {}).items():
        setattr(request, field, value)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
    if not future.done() or future.exception() is not None:
        raise RuntimeError(f"service call failed: {name}")
    response = future.result()
    if require_success and not response.success:
        raise RuntimeError(f"service rejected request: {name}")
    return response


def _set_rt_session(node, *, streaming: bool) -> None:
    """Switch the driver's existing RT connection between command and monitor modes.

    The pinned hardware interface starts a monitor-only 1 kHz output session.
    A 100 Hz servoj_rt stream must declare a matching 10 ms input period before
    start_rt_control; otherwise the controller may silently ignore commands.
    """
    from dsr_msgs2.srv import (SetRtControlInput, SetRtControlOutput,
                               StartRtControl, StopRtControl)

    _call(node, StopRtControl, "/realtime/stop_rt_control",
          require_success=streaming)
    _call(node, SetRtControlInput, "/realtime/set_rt_control_input",
          request_fields={"version": "v1.0", "period": PERIOD_S if streaming else 0.0,
                          "loss": RT_INPUT_LOSS_COUNT if streaming else 4})
    _call(node, SetRtControlOutput, "/realtime/set_rt_control_output",
          request_fields={"version": "v1.0", "period": PERIOD_S if streaming else 0.001,
                          "loss": RT_INPUT_LOSS_COUNT if streaming else 4})
    _call(node, StartRtControl, "/realtime/start_rt_control")


def _request_quick_stop(node) -> None:
    from dsr_msgs2.srv import MoveStop
    import rclpy

    try:
        client = node.create_client(MoveStop, "/motion/move_stop")
        if not client.wait_for_service(timeout_sec=0.5):
            print("Quick-stop service unavailable", flush=True)
            return
        request = MoveStop.Request()
        request.stop_mode = 1  # DR_QSTOP, Stop Category 2
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
        if future.done() and future.exception() is None and future.result().success:
            print("Quick-stop confirmed", flush=True)
        else:
            print("Quick-stop was not confirmed", flush=True)
    except Exception as error:  # Best effort; preserve the original abort reason.
        print(f"Quick-stop request failed: {error}", flush=True)


def execute(trajectory_file: str, gripper_type: str, output_dir: str, *,
            supervised_trial: bool = False, experiment_id: str | None = None,
            profile_name: str | None = None, repeat_index: int | None = None) -> Path:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray
    from dsr_msgs2.msg import ServojRtStream
    from dsr_msgs2.srv import GetRobotMode, GetRobotState, GetRobotSystem

    trajectory = load_trajectory(trajectory_file)
    samples = validate_real_plan(trajectory, gripper_type,
                                 supervised_trial=supervised_trial)
    report = inspect_topics(timeout_s=3.0)
    if not report["all_fields_available"]:
        missing = [field for field, item in report["fields"].items() if item["status"] != "ok"]
        raise RuntimeError(f"real RT preflight failed: {', '.join(missing)}")

    folder = Path(output_dir).expanduser().resolve()
    folder.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    record_path = folder / f"{trajectory.source.stem}_real_execution_{stamp}.json"
    samples_path = folder / f"{trajectory.source.stem}_real_execution_{stamp}_samples.jsonl"
    record = {
        "trajectory_file": str(trajectory.source),
        "trajectory_sha256": hashlib.sha256(trajectory.source.read_bytes()).hexdigest(),
        "gripper_type": gripper_type,
        "experiment_id": experiment_id,
        "profile_name": profile_name,
        "repeat_index": repeat_index,
        "source": "real",
        "control_mode": "servoj_rt",
        "torque_signal_ground_truth_verified": False,
        "torque_signal_note": "actual_joint_torque is a controller estimate; characterize before dynamics-gap ground truth use",
        "supervised_trial": supervised_trial,
        "physical_tool_confirmed_by_operator": True,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "rate_hz": 100,
        "rt_input_period_s": PERIOD_S,
        "rt_input_loss_count": RT_INPUT_LOSS_COUNT,
        "initial_hold_command_sent": False,
        "servo_time_s": SERVO_TIME_S,
        "setpoints_planned": len(samples),
        "setpoints_sent": 0,
        "samples_file": str(samples_path),
        "max_lateness_s": 0.0,
        "max_tracking_error_rad": 0.0,
        "result": "not_started",
    }
    stop_requested = threading.Event()
    old_handlers = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}

    def on_stop(_signum, _frame):
        stop_requested.set()

    rclpy.init()
    node = Node("sim_real_gap_real_100hz_runner")
    for signum in old_handlers:
        signal.signal(signum, on_stop)
    latest = {}
    measured_rows = []
    stamps_seen = 0
    last_stamp = None
    stamp_arrivals = []

    def on_field(message: Float32MultiArray, field: str):
        nonlocal stamps_seen, last_stamp
        try:
            if field == "robot_state":
                values = [float(value) for value in message.data]
                if len(values) != 1 or not math.isfinite(values[0]):
                    return
            else:
                values = validate_values(field, message.data)
        except ValueError:
            return
        latest[field] = (time.monotonic(), values)
        if field == "time_stamp" and (last_stamp is None or values[0] > last_stamp):
            stamps_seen += 1
            last_stamp = values[0]
            stamp_arrivals.append(time.monotonic())

    subscriptions = [node.create_subscription(
        Float32MultiArray, f"/rt_topic/{field}",
        lambda message, field=field: on_field(message, field), 10)
        for field in (*FIELDS, "robot_state")]
    publisher = node.create_publisher(ServojRtStream, "/servoj_rt_stream", 10)
    sent_any = False
    session_reconfigured = False
    try:
        if _call(node, GetRobotSystem, "/system/get_robot_system").robot_system != 0:
            raise RuntimeError("controller reports a virtual robot, not a real robot")
        if _call(node, GetRobotMode, "/system/get_robot_mode").robot_mode != 1:
            raise RuntimeError("robot must already be in AUTONOMOUS mode")
        # The pinned driver fills robot_state but leaves its success flag at the default.
        if _call(node, GetRobotState, "/system/get_robot_state", require_success=False).robot_state != 1:
            raise RuntimeError("robot must be in STANDBY state")

        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and stamps_seen < 50:
            rclpy.spin_once(node, timeout_sec=0.01)
        if stamps_seen < 50:
            raise RuntimeError("RT time_stamp is not advancing at the expected rate")
        observed_hz = (len(stamp_arrivals)-1) / (stamp_arrivals[-1]-stamp_arrivals[0])
        record["rt_timestamp_observed_hz"] = observed_hz
        if observed_hz < 80 or observed_hz > 150:
            raise RuntimeError(f"RT timestamp rate {observed_hz:.1f} Hz is outside 80..150 Hz")
        for field in (*FIELDS, "robot_state"):
            if field not in latest or time.monotonic() - latest[field][0] > MAX_FEEDBACK_AGE_S:
                raise RuntimeError(f"stale or absent RT feedback: {field}")
        if round(latest["robot_state"][1][0]) != 1:
            raise RuntimeError("robot left STANDBY before execution")
        position = latest["actual_joint_position"][1]
        velocity = latest["actual_joint_velocity"][1]
        if max(abs(math.radians(position[i]) - samples[0].q_rad[i]) for i in range(6)) > 0.02:
            raise RuntimeError("real joint pose differs from JSON start by over 0.02 rad")
        if max(abs(math.radians(value)) for value in velocity) > 0.03:
            raise RuntimeError("real robot must be stationary before execution")
        subscriber_deadline = time.monotonic() + 2.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < subscriber_deadline:
            rclpy.spin_once(node, timeout_sec=0.01)
        if publisher.get_subscription_count() != 1:
            raise RuntimeError("exactly one real servoj_rt_stream subscriber is required")
        if len(node.get_publishers_info_by_topic("/servoj_rt_stream")) != 1:
            raise RuntimeError("another servoj_rt_stream publisher is active")
        if stop_requested.is_set():
            raise RuntimeError("operator requested stop before streaming")

        # The hardware driver owns the RT socket. Its bringup configures output
        # alone, so the command session must be configured through its services.
        session_reconfigured = True
        _set_rt_session(node, streaming=True)
        first = ServojRtStream()
        first.pos = [math.degrees(value) for value in samples[0].q_rad]
        first.vel = [0.0] * 6
        first.acc = [0.0] * 6
        first.time = SERVO_TIME_S
        publisher.publish(first)
        sent_any = True
        record["initial_hold_command_sent"] = True
        # Keep the RT input watchdog fed while the output stream resumes. A
        # successful configuration service call alone is not controller ACK.
        feedback_count_before = stamps_seen
        warmup_deadline = time.monotonic() + 1.0
        next_warmup_hold = time.monotonic() + PERIOD_S
        while time.monotonic() < warmup_deadline:
            if stop_requested.is_set():
                raise RuntimeError("operator requested stop during RT warmup")
            rclpy.spin_once(node, timeout_sec=0.001)
            now = time.monotonic()
            if now >= next_warmup_hold:
                if now - next_warmup_hold > MAX_LATE_S:
                    raise RuntimeError("100 Hz RT warmup hold deadline missed")
                publisher.publish(first)
                next_warmup_hold += PERIOD_S
            if (stamps_seen >= feedback_count_before + 3 and
                    all(field in latest and now - latest[field][0] <= MAX_FEEDBACK_AGE_S
                        for field in (*FIELDS, "robot_state"))):
                break
        else:
            raise RuntimeError("RT feedback did not resume after 100 Hz input configuration")
        if round(latest["robot_state"][1][0]) != 1:
            raise RuntimeError("robot left STANDBY during RT warmup")
        if max(abs(math.radians(latest["actual_joint_position"][1][i]) - samples[0].q_rad[i])
               for i in range(6)) > 0.02:
            raise RuntimeError("real joint pose changed during RT warmup")
        print(f"Starting 100 Hz real stream: {trajectory.name}, {len(samples)} setpoints", flush=True)
        stream_stamp_start = stamps_seen
        start = time.monotonic() + PERIOD_S
        for index, point in enumerate(samples):
            due = start + index * PERIOD_S
            while time.monotonic() < due:
                if stop_requested.is_set():
                    raise RuntimeError("operator requested stop")
                rclpy.spin_once(node, timeout_sec=min(0.001, max(0.0, due - time.monotonic())))
            lateness = time.monotonic() - due
            record["max_lateness_s"] = max(record["max_lateness_s"], lateness)
            if lateness > MAX_LATE_S:
                raise RuntimeError(f"100 Hz deadline missed by {lateness:.4f} s at sample {index}")
            if publisher.get_subscription_count() != 1:
                raise RuntimeError("RT command subscriber changed during execution")
            for field in (*FIELDS, "robot_state"):
                if field not in latest or time.monotonic() - latest[field][0] > MAX_FEEDBACK_AGE_S:
                    raise RuntimeError(f"RT feedback lost: {field}")
            if time.monotonic() - stamp_arrivals[-1] > MAX_FEEDBACK_AGE_S:
                raise RuntimeError("RT time_stamp stopped advancing")
            if round(latest["robot_state"][1][0]) not in (1, 2):
                raise RuntimeError("robot entered a non-motion state")
            actual = latest["actual_joint_position"][1]
            error = max(abs(math.radians(actual[j]) - point.q_rad[j]) for j in range(6))
            record["max_tracking_error_rad"] = max(record["max_tracking_error_rad"], error)
            if error > MAX_TRACKING_ERROR_RAD:
                raise RuntimeError(f"joint tracking error {error:.3f} rad exceeds limit")
            command = ServojRtStream()
            command.pos = [math.degrees(value) for value in point.q_rad]
            command.vel = [math.degrees(value) for value in point.dq_rad_s]
            command.acc = [math.degrees(value) for value in point.ddq_rad_s2]
            command.time = SERVO_TIME_S
            publisher.publish(command)
            sent_any = True
            record["setpoints_sent"] += 1
            measured_rows.append({
                "experiment_id": experiment_id,
                "profile_name": profile_name,
                "repeat_index": repeat_index,
                "target_time_s": point.time_s,
                "send_offset_s": time.monotonic() - start,
                "command_utc": datetime.now(timezone.utc).isoformat(),
                "feedback_arrival_monotonic_s": latest["time_stamp"][0],
                "target_q_rad": list(point.q_rad),
                "target_dq_rad_s": list(point.dq_rad_s),
                "target_ddq_rad_s2": list(point.ddq_rad_s2),
                "real_rt": {field: latest[field][1] for field in (*FIELDS, "robot_state")},
            })
            if index % 100 == 0:
                print(f"Real stream {index}/{len(samples)-1} at {point.time_s:.2f}s", flush=True)

        feedback_hz = (stamps_seen - stream_stamp_start) / (len(samples) * PERIOD_S)
        record["rt_feedback_observed_hz_during_stream"] = feedback_hz
        if feedback_hz < 80 or feedback_hz > 150:
            raise RuntimeError(f"RT feedback averaged {feedback_hz:.1f} Hz during the stream")
        settle_started = time.monotonic()
        settle_deadline = settle_started + 2.0
        next_hold = settle_started
        final = samples[-1]
        while time.monotonic() < settle_deadline:
            if stop_requested.is_set():
                raise RuntimeError("operator requested stop")
            rclpy.spin_once(node, timeout_sec=0.001)
            now = time.monotonic()
            if now >= next_hold:
                if now - next_hold > MAX_LATE_S:
                    raise RuntimeError("100 Hz final hold deadline missed")
                publisher.publish(command)  # Last setpoint has zero velocity and acceleration.
                next_hold += PERIOD_S
            for field in (*FIELDS, "robot_state"):
                if field not in latest or now - latest[field][0] > MAX_FEEDBACK_AGE_S:
                    raise RuntimeError(f"RT feedback lost during final hold: {field}")
            if round(latest["robot_state"][1][0]) not in (1, 2):
                raise RuntimeError("robot entered a non-motion state during final hold")
            if "actual_joint_position" not in latest or "actual_joint_velocity" not in latest:
                continue
            if time.monotonic() - latest["actual_joint_position"][0] > MAX_FEEDBACK_AGE_S:
                raise RuntimeError("RT position feedback lost during final settle")
            actual = latest["actual_joint_position"][1]
            velocity = latest["actual_joint_velocity"][1]
            if (now - settle_started > 0.05 and
                    max(abs(math.radians(actual[j]) - final.q_rad[j]) for j in range(6)) < 0.02
                    and max(abs(math.radians(value)) for value in velocity) < 0.03):
                record["result"] = "completed"
                print("Real trajectory completed and final pose confirmed", flush=True)
                return record_path
        raise RuntimeError("final real pose was not confirmed within 2 s")
    except Exception as error:
        record["result"] = "aborted"
        record["reason"] = str(error)
        print(f"Real execution aborted: {error}", flush=True)
        if sent_any:
            _request_quick_stop(node)
        raise
    finally:
        record["ended_utc"] = datetime.now(timezone.utc).isoformat()
        restore_error = None
        try:
            if session_reconfigured:
                try:
                    _set_rt_session(node, streaming=False)
                    record["rt_monitor_session_restored"] = True
                except Exception as error:
                    restore_error = error
                    record["rt_monitor_session_restored"] = False
                    record["rt_restore_error"] = str(error)
                    record["result"] = "aborted"
                    print(f"RT monitor session restoration failed: {error}", flush=True)
            with samples_path.open("w", encoding="utf-8") as stream:
                for row in measured_rows:
                    stream.write(json.dumps(row, ensure_ascii=False) + "\n")
            record_path.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        finally:
            node.destroy_node()
            rclpy.shutdown()
            for signum, previous in old_handlers.items():
                signal.signal(signum, previous)
        if restore_error is not None:
            raise RuntimeError(f"RT monitor session restoration failed: {restore_error}")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Operator-confirmed A0509 real 100 Hz JSON practice")
    parser.add_argument("--trajectory", required=True)
    parser.add_argument("--gripper-type", choices=("none", "ag145", "mdpg_1s1"), required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--operator-confirmed", action="store_true")
    parser.add_argument("--supervised-trial", action="store_true",
                        help="Require a prepared F1-F4 physical preflight trajectory")
    parser.add_argument("--experiment-id")
    parser.add_argument("--profile-name")
    parser.add_argument("--repeat-index", type=int)
    args = parser.parse_args(argv)
    if not args.operator_confirmed:
        parser.error("physical tool, clear workspace and operator supervision must be confirmed")
    try:
        path = execute(args.trajectory, args.gripper_type, args.output_dir,
                       supervised_trial=args.supervised_trial,
                       experiment_id=args.experiment_id,
                       profile_name=args.profile_name,
                       repeat_index=args.repeat_index)
    except (OSError, RuntimeError, ValueError, json.JSONDecodeError) as error:
        print(f"Real execution failed: {error}", flush=True)
        return 2
    print(f"Execution record: {path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
