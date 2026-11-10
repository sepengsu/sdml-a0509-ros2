import ast
import csv
import json
import math
from pathlib import Path

import pytest

from a0509_dynamics_experiment.dataset_exporter import create_trial, save_dynamics, save_metadata, save_trajectory
from a0509_dynamics_experiment.ag145_model import AG145_MASS_KG, build_ag145_scene, source_paths
from a0509_dynamics_experiment.dynamics_logger import DynamicsRecorder
from a0509_dynamics_experiment.real_executor import validate_real_execution
from a0509_dynamics_experiment.run_experiment import main
from a0509_dynamics_experiment.simulation_executor import MujocoObserver
from a0509_dynamics_experiment.trajectory_generator import JOINT_NAMES, TrajectoryConfig, generate
from a0509_dynamics_experiment.urdf_kinematics import A0509Urdf, find_a0509_urdf


@pytest.mark.parametrize("trajectory_id,extent", [
    ("T1", (120, 0, 0)), ("T2", (0, 80, 60)), ("T3", (120, 80, 60))])
def test_cartesian_extents_and_smooth_stops(trajectory_id, extent):
    plan = generate(TrajectoryConfig(trajectory_id, reference_tcp_mm=(500, 100, 300),
                                     pre_hold_seconds=0, post_hold_seconds=0))
    xyz = [point.xyz_mm for point in plan.points]
    for axis in range(3):
        assert max(p[axis] for p in xyz) - min(p[axis] for p in xyz) == pytest.approx(extent[axis])
    assert all(abs(v) < 1e-10 for index in plan.waypoint_indices
               for v in plan.points[index].velocity_mm_s)
    assert all(abs(a) < 1e-10 for index in plan.waypoint_indices
               for a in plan.points[index].acceleration_mm_s2)


def test_t4_sequential_and_simultaneous_excitation():
    sequential = generate(TrajectoryConfig("T4", joint_indices=(0, 1), pre_hold_seconds=0,
                                           post_hold_seconds=0))
    simultaneous = generate(TrajectoryConfig("T4", joint_indices=(0, 1), joint_mode="simultaneous",
                                             pre_hold_seconds=0, post_hold_seconds=0))
    assert sequential.points[-1].q_rad == pytest.approx((0,) * 6)
    assert len(sequential.points) > len(simultaneous.points)
    assert max(p.q_rad[0] for p in sequential.points) == pytest.approx(math.radians(5))
    assert any(p.q_rad[0] > 0 and p.q_rad[1] > 0 for p in simultaneous.points)
    assert not any(abs(p.q_rad[0]) > 0.001 and abs(p.q_rad[1]) > 0.001 for p in sequential.points)


def test_exports_share_one_schema_and_preserve_missing_data(tmp_path):
    plan = generate(TrajectoryConfig("T1", sample_hz=10, segment_seconds=0.2,
                                     pre_hold_seconds=0, post_hold_seconds=0))
    folder, trial_id = create_trial(tmp_path, "simulation", "T1")
    save_trajectory(plan, folder)
    assert trial_id == "trial_001"
    assert len(list(csv.DictReader((folder / "trajectory.csv").open()))) == len(plan.points)
    assert json.loads((folder / "trajectory.json").read_text())["config"]["trajectory_id"] == "T1"
    save_dynamics([{"timestamp": 1.0, "q1": 0.1, "mode": "simulation"}], folder)
    row = next(csv.DictReader((folder / "dynamics.csv").open()))
    assert row["q1"] == "0.1" and row["actual_motor_torque"] == ""
    save_metadata(folder, plan, "simulation", trial_id, {"sample_note": "test"})
    metadata = json.loads((folder / "metadata.json").read_text())
    assert metadata["units"]["q"] == "rad" and metadata["sample_note"] == "test"


def test_real_rt_conversion_and_staleness():
    recorder = DynamicsRecorder("real", "T1", "trial_001", "none", 0.0, max_age_s=0.03)
    recorder.ingest_rt("actual_joint_position", [180, 0, 0, 0, 0, 0], arrival_s=1.0)
    recorder.ingest_rt("actual_joint_velocity", [90, 0, 0, 0, 0, 0], arrival_s=1.0)
    recorder.ingest_rt("actual_motor_torque", [1] * 6, arrival_s=1.0)
    current = recorder.snapshot(100.0, 10.0, arrival_s=1.01)
    assert current["q1"] == pytest.approx(math.pi)
    assert current["dq1"] == pytest.approx(math.pi / 2)
    assert current["actual_motor_torque"] == (1.0,) * 6
    stale = recorder.snapshot(100.1, 10.1, arrival_s=1.1)
    assert stale.get("q1") is None and stale["actual_motor_torque"] is None
    assert recorder.quality()["blank_field_counts"]["q1"] == 1


def test_simulation_quality_uses_simulation_clock():
    recorder = DynamicsRecorder("simulation", "T1", "trial_001", "none", 0.0)
    for index in range(101):
        recorder.ingest_joint_state(JOINT_NAMES, [0.0] * 6,
                                    arrival_s=1.0 + index / 75,
                                    source_time_s=1.0 + index / 100)
    quality = recorder.quality()["joint_states"]
    assert quality["time_basis"] == "message_header_sim_time"
    assert quality["observed_hz"] == pytest.approx(100)
    assert quality["wall_arrival_hz"] == pytest.approx(75)
    assert quality["estimated_missing_rate"] == 0


def test_real_motion_fail_closed_and_no_motion_interface():
    reasons = validate_real_execution(TrajectoryConfig("T1"), {})
    assert any("collision" in reason for reason in reasons)
    source = Path(__file__).parents[1] / "a0509_dynamics_experiment" / "real_executor.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    prohibited = {"create_publisher", "create_client", "create_action_client"}
    assert not [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute) and node.func.attr in prohibited]


def test_unsafe_mode_combinations_are_rejected():
    with pytest.raises(SystemExit):
        main(["--preview", "--execute", "--trajectory", "T1"])
    with pytest.raises(SystemExit):
        main(["--preview", "--simulation", "--trajectory", "T1"])
    with pytest.raises(SystemExit):
        main(["--preview", "--record-video", "--trajectory", "T1"])
    with pytest.raises(SystemExit):
        main(["--trajectory", "all"])


def test_simulation_execute_combination_never_enters_real_mode(monkeypatch, tmp_path):
    from a0509_dynamics_experiment import simulation_executor, real_executor
    calls = []
    monkeypatch.setattr(simulation_executor, "execute", lambda *args, **kwargs: calls.append("simulation") or tmp_path)
    monkeypatch.setattr(real_executor, "execute", lambda *args, **kwargs: pytest.fail("real executor called"))
    assert main(["--simulation", "--execute", "--trajectory", "T1"]) == 0
    assert calls == ["simulation"]


def test_simulation_all_dispatches_each_reference_path(monkeypatch, tmp_path):
    from a0509_dynamics_experiment import simulation_executor
    calls = []
    monkeypatch.setattr(simulation_executor, "execute",
                        lambda config, *args, **kwargs: calls.append(config.trajectory_id) or tmp_path)
    assert main(["--simulation", "--trajectory", "all"]) == 0
    assert calls == ["T1", "T2", "T3", "T4"]


def test_preview_produces_png_html_without_ros(tmp_path):
    assert main(["--preview", "--trajectory", "T1", "--results-dir", str(tmp_path),
                 "--sample-hz", "10", "--segment-seconds", "0.2", "--pre-hold", "0",
                 "--post-hold", "0"]) == 0
    folder = tmp_path / "preview" / "T1" / "trial_001"
    assert (folder / "trajectory_3d.png").stat().st_size > 1000
    assert "data:image/png;base64" in (folder / "trajectory_3d.html").read_text()
    assert (folder / "waypoints.csv").exists()


def test_urdf_model_contains_six_link_chain():
    path = find_a0509_urdf()
    if path is None:
        pytest.skip("Doosan submodule not initialized")
    points = A0509Urdf(path).links_mm((0,) * 6)
    assert len(points) == 7
    assert points[-1][2] > 1000


def test_ag145_scene_has_tool_mass_visuals_and_tcp(tmp_path):
    pytest.importorskip("mujoco")
    arm_scene, gripper_urdf = source_paths()
    if not arm_scene.is_file() or not gripper_urdf.is_file():
        pytest.skip("A0509 or AG145 model package not initialized")
    scene = build_ag145_scene(tmp_path / "scene.xml", arm_scene, gripper_urdf)
    bare = MujocoObserver(arm_scene)
    gripper = MujocoObserver(scene)
    assert gripper.model.nq == bare.model.nq == 6
    assert sum(gripper.model.body_mass) - sum(bare.model.body_mass) == pytest.approx(AG145_MASS_KG)
    assert gripper.tcp_site_id >= 0
    assert sum(name.startswith("ag145_") for name in
               (gripper.model.geom(i).name for i in range(gripper.model.ngeom))) >= 8
    q = (-0.5, -0.5, 1.6, 0, -0.6, 0)
    assert math.dist(gripper.tcp_mm(q), bare.tcp_mm(q)) > 100
    gripper.set_ik_reference(q)
    target = tuple(value + offset for value, offset in zip(gripper.tcp_mm(q), (10, 0, 0)))
    solved = gripper.solve_ik(target, q)
    gripper.require_contact_free(solved)
    assert math.dist(gripper.tcp_mm(solved), target) < 1.1


def test_ag145_preview_uses_tool_tcp_without_robot_connection(tmp_path):
    pytest.importorskip("mujoco")
    assert main(["--preview", "--trajectory", "T1", "--gripper-type", "ag145",
                 "--results-dir", str(tmp_path), "--sample-hz", "10",
                 "--segment-seconds", "0.2", "--pre-hold", "0",
                 "--post-hold", "0"]) == 0
    folder = tmp_path / "preview" / "T1" / "trial_001"
    observer = MujocoObserver(folder / "ag145_scene.xml")
    home = (-0.5, -0.5, 1.6, 0, -0.6, 0)
    with (folder / "trajectory.csv").open(newline="", encoding="utf-8") as stream:
        first = next(csv.DictReader(stream))
    assert float(first["x_mm"]) == pytest.approx(observer.tcp_mm(home)[0])
    assert (folder / "trajectory_3d.png").stat().st_size > 1000


@pytest.mark.parametrize("trajectory_id", ["T1", "T2", "T3"])
def test_full_cartesian_path_ik_on_pinned_mujoco_model(trajectory_id):
    pytest.importorskip("mujoco")
    model = Path(__file__).parents[2] / "doosan-robot2" / "dsr_description2" / "mujoco_models" / "a0509" / "scene.xml"
    if not model.is_file():
        pytest.skip("Doosan submodule not initialized")
    observer = MujocoObserver(model)
    q = (-0.5, -0.5, 1.6, 0, -0.6, 0)
    observer.set_ik_reference(q)
    plan = generate(TrajectoryConfig(trajectory_id, reference_tcp_mm=observer.tcp_mm(q)))
    for point in plan.points:
        q = observer.solve_ik(point.xyz_mm, q)
        observer.require_contact_free(q)
    assert math.dist(observer.tcp_mm(q), plan.points[-1].xyz_mm) < 1.1
    physics = observer.observe(q, (0,) * 6, (0,) * 6)
    assert len(physics["qfrc_inverse"]) == 6
    assert physics["qfrc_actuator"] is None
