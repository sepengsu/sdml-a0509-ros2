"""The parallel collector must resume only matching, complete simulation trials."""

import importlib.util
import json
from pathlib import Path


SOURCE = Path(__file__).resolve().parents[1] / "scripts/parallel_collection_worker.py"
SPEC = importlib.util.spec_from_file_location("parallel_collection_worker", SOURCE)
worker = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(worker)


def test_completed_requires_matching_mount_and_outputs(tmp_path, monkeypatch):
    monkeypatch.setattr(worker, "RESULTS", tmp_path)
    job = {"condition": "mdpg_p05", "gripper": "mdpg_1s1", "payload_kg": 0.5,
           "trajectory": "T5", "repeat": 2}
    campaign = {"results_name": "campaign", "mount": {"xyz_m": [0, 0, 0],
                                                   "rpy_rad": [0, 0, 0]}, "record_video": False}
    trial = tmp_path / "campaign/mdpg_p05/repeat_002/simulation/T5/trial_001"
    (trial / "rosbag").mkdir(parents=True)
    (trial / "dynamics.csv").write_text("timestamp\n", encoding="utf-8")
    (trial / "rosbag/metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    metadata = {"execution_success": True, "sample_count": 1401,
                "quality": {"rows_written": 1401}, "trajectory_id": "T5",
                "gripper_type": "mdpg_1s1", "payload_mass_kg": 0.5,
                "payload_model": {"spec_sha256": "expected"},
                "gripper_model": {"dynamics_model": "box_approx",
                                  "mount_xyz_m": [0, 0, 0], "mount_rpy_rad": [0, 0, 0]}}
    (trial / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
    assert worker.completed(job, campaign, "expected")
    assert not worker.completed(job, campaign, "different")
    campaign["mount"]["xyz_m"] = [0.01, 0, 0]
    assert not worker.completed(job, campaign, "expected")
    campaign["mount"]["xyz_m"] = [0, 0, 0]
    campaign["record_video"] = True
    assert not worker.completed(job, campaign, "expected")
    (trial / "simulation.mp4").write_bytes(b"video")
    assert worker.completed(job, campaign, "expected")
    (trial / "rosbag/metadata.yaml").unlink()
    assert not worker.completed(job, campaign, "expected")


def test_command_is_simulation_only_and_isolated(monkeypatch, tmp_path):
    monkeypatch.setattr(worker, "RESULTS", tmp_path)
    job = {"condition": "mdpg_p05", "gripper": "mdpg_1s1", "payload_kg": 0.5,
           "trajectory": "T8", "repeat": 3}
    campaign = {"results_name": "campaign", "mount": {"xyz_m": [0, 0, 0],
                                                   "rpy_rad": [0, 0, 0]}, "record_video": False}
    argv = worker.command(job, campaign, 177)
    assert argv[:4] == ["ros2", "run", "a0509_dynamics_experiment", "run_experiment"]
    assert "--simulation" in argv and "--execute" not in argv
    assert argv[argv.index("--sim-domain-id") + 1] == "177"
    assert Path(argv[argv.index("--results-dir") + 1]).parts[-3:] == (
        "campaign", "mdpg_p05", "repeat_003")
    assert "--defer-summary" in argv and "--mdpg-dynamics-model" in argv
