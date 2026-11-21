import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim_real_gap_getting.rt_check import FIELDS, compare_trajectory_start, validate_values
from sim_real_gap_getting.trajectory import load_trajectory, resample_uniform, save_for_gripper
from sim_real_gap_getting.gui import PracticeGui, _base_stem, _next_available
from sim_real_gap_getting.real_execute import _call, validate_real_plan
from sim_real_gap_getting.home_pose import HOME_JOINT_DEG, HOME_JOINT_RAD, plan_sim_home


EXAMPLE = Path(__file__).resolve().parents[1] / "examples" / "small_joint_1.json"
FORWARD_WALL = EXAMPLE.with_name("forward_150mm_wall.json")
RIGHT_ANGLE = EXAMPLE.with_name("right_angle_20cm_twice.json")


def test_example_resamples_to_safe_simulation_path():
    trajectory = load_trajectory(EXAMPLE)
    assert len(trajectory.samples) == 201
    assert trajectory.samples[100].q_rad[0] == pytest.approx(0.03)
    assert trajectory.points[0].q_rad == pytest.approx(HOME_JOINT_RAD, abs=1e-9)
    assert trajectory.samples[0].q_rad == trajectory.samples[-1].q_rad


def test_simulation_home_plan_reaches_requested_joint_pose_within_limits():
    plan = plan_sim_home((0.0,) * 6)
    assert len(plan.samples) == 801
    assert plan.samples[0].q_rad == (0.0,) * 6
    assert plan.samples[-1].q_rad == pytest.approx(HOME_JOINT_RAD)
    assert HOME_JOINT_DEG[0] == 0.0
    assert all(abs(value) < 0.524 for point in plan.samples for value in point.dq_rad_s)


def test_forward_wall_path_keeps_j1_fixed_and_passes_real_command_limits():
    trajectory = load_trajectory(FORWARD_WALL)
    samples = validate_real_plan(trajectory, "none")
    assert len(samples) == 601
    assert samples[0].q_rad == pytest.approx(HOME_JOINT_RAD)
    assert all(point.q_rad[0] == point.dq_rad_s[0] == point.ddq_rad_s2[0] == 0
               for point in samples)
    assert samples[-1].q_rad[1] > samples[0].q_rad[1]


def test_right_angle_path_stops_at_every_corner_and_keeps_j1_fixed():
    trajectory = load_trajectory(RIGHT_ANGLE)
    samples = validate_real_plan(trajectory, "none")
    assert len(samples) == 3401
    for index in (0, 100, 900, 1700, 2500, 3300, 3400):
        assert max(abs(value) for value in samples[index].dq_rad_s) < 1e-8
    assert all(point.q_rad[0] == point.dq_rad_s[0] == point.ddq_rad_s2[0] == 0
               for point in samples)


def test_exported_plan_json_format_is_accepted(tmp_path):
    data = json.loads(EXAMPLE.read_text(encoding="utf-8"))
    data.pop("name")
    data.pop("joint_names")
    data.pop("units")
    data["config"] = {"trajectory_id": "T4"}
    source = tmp_path / "trajectory.json"
    source.write_text(json.dumps(data), encoding="utf-8")
    assert load_trajectory(source).name == "T4"


def test_interpolated_limit_violation_is_rejected(tmp_path):
    data = json.loads(EXAMPLE.read_text(encoding="utf-8"))
    data["points"][1]["q_rad"][0] = 1.0
    source = tmp_path / "too_fast.json"
    source.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="acceleration exceeds"):
        load_trajectory(source)


def test_all_requested_rt_fields_have_expected_shape_and_zero_is_valid():
    assert set(FIELDS) == {
        "actual_joint_position", "actual_joint_velocity", "actual_joint_torque",
        "actual_motor_torque", "gravity_torque", "time_stamp",
    }
    for field in FIELDS:
        assert validate_values(field, 0.0 if field == "time_stamp" else [0]*6)
    with pytest.raises(ValueError, match="expected 6"):
        validate_values("gravity_torque", [0]*5)


def test_real_start_comparison_converts_degrees_and_checks_stationary():
    trajectory = load_trajectory(EXAMPLE)
    fields = {field: {"values": list(HOME_JOINT_DEG) if field == "actual_joint_position" else
                      ([0] if field == "time_stamp" else [0]*6)}
              for field in FIELDS}
    assert compare_trajectory_start({"fields": fields}, trajectory)["status"] == "review"
    fields["actual_joint_position"]["values"][0] = 10
    assert compare_trajectory_start({"fields": fields}, trajectory)["status"] == "mismatch"


@pytest.mark.parametrize("gripper", ["none", "ag145", "mdpg_1s1"])
def test_gripper_save_round_trip_keeps_source_unchanged(tmp_path, gripper):
    original = EXAMPLE.read_text(encoding="utf-8")
    target = tmp_path / f"practice_{gripper}.json"
    saved = save_for_gripper(load_trajectory(EXAMPLE), target, gripper)
    assert saved == target
    assert load_trajectory(saved).gripper_type == gripper
    assert json.loads(saved.read_text(encoding="utf-8"))["physical_tool_verified"] is False
    assert EXAMPLE.read_text(encoding="utf-8") == original


def test_unknown_gripper_cannot_be_saved(tmp_path):
    with pytest.raises(ValueError, match="unsupported gripper_type"):
        save_for_gripper(load_trajectory(EXAMPLE), tmp_path / "wrong.json", "other")


def test_filename_replaces_existing_gripper_suffix():
    assert _base_stem("practice_ag145") == "practice"
    assert _base_stem("practice_mdpg_1s1") == "practice"
    assert _base_stem("practice") == "practice"
    assert _base_stem("practice_ag145_001") == "practice"


def test_selected_folder_uses_new_filename_when_file_exists(tmp_path):
    existing = tmp_path / "practice_ag145.json"
    existing.write_text("existing", encoding="utf-8")
    assert _next_available(existing).name == "practice_ag145_001.json"


def test_output_folder_is_separated_by_gripper(tmp_path):
    gui = PracticeGui.__new__(PracticeGui)
    gui.output_dir_var = type("Var", (), {"get": lambda self: str(tmp_path)})()
    assert gui._output_folder("ag145") == tmp_path / "ag145"
    assert gui._output_folder("mdpg_1s1") == tmp_path / "mdpg_1s1"


def test_real_plan_has_every_10ms_setpoint(tmp_path):
    tagged = save_for_gripper(load_trajectory(EXAMPLE), tmp_path / "small_none.json", "none")
    trajectory = load_trajectory(tagged)
    samples = validate_real_plan(trajectory, "none")
    assert len(samples) == 201
    assert all(abs(sample.time_s - index / 100) < 1e-9 for index, sample in enumerate(samples))
    assert samples == resample_uniform(trajectory)


def test_real_plan_rejects_mismatched_gripper_and_high_speed(tmp_path):
    tagged = save_for_gripper(load_trajectory(EXAMPLE), tmp_path / "small_ag145.json", "ag145")
    with pytest.raises(ValueError, match="gripper_type"):
        validate_real_plan(load_trajectory(tagged), "none")
    data = json.loads(tagged.read_text(encoding="utf-8"))
    data["points"] = [data["points"][0], data["points"][-1]]
    data["points"][1]["q_rad"][3] = 0.6
    tagged.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="30 deg/s"):
        validate_real_plan(load_trajectory(tagged), "ag145")


def test_pinned_robot_state_service_does_not_set_success(monkeypatch):
    response = SimpleNamespace(success=False, robot_state=1)
    future = SimpleNamespace(done=lambda: True, exception=lambda: None, result=lambda: response)
    client = SimpleNamespace(wait_for_service=lambda timeout_sec: True,
                             call_async=lambda request: future)
    node = SimpleNamespace(create_client=lambda service_type, name: client)
    service = SimpleNamespace(Request=lambda: object())
    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace(spin_until_future_complete=lambda *a, **k: None))
    assert _call(node, service, "/system/get_robot_state", require_success=False).robot_state == 1
    with pytest.raises(RuntimeError, match="rejected"):
        _call(node, service, "/system/get_robot_state")
