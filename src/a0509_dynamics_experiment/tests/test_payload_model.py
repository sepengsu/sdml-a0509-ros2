"""Payload specs must affect actual MuJoCo inertia, not just CSV metadata."""

import json
from pathlib import Path

import mujoco
import numpy as np
import pytest

from a0509_dynamics_experiment.payload_model import (
    attach_payload, load_payload_spec, main as validate_payloads)
from a0509_dynamics_experiment.run_experiment import main


def _spec(path: Path, gripper: str, mass: float):
    key = f"{'ag145' if gripper == 'ag145' else 'mdpg'}_p{'05' if mass == 0.5 else '10'}"
    inertia = mass * .1**2 / 6
    path.write_text(json.dumps({"schema_version": 1, "conditions": {
        key: {"mass_kg": mass, "parameter_source": "declared_simulation_assumption",
              "source_note": "test-only 0.1 m uniform cube",
              "mount_xyz_m": [0, 0, .2], "mount_rpy_rad": [0, 0, 0],
              "com_xyz_m": [0, 0, 0],
              "inertia_kg_m2": [inertia, inertia, inertia, 0, 0, 0]}}}),
                    encoding="utf-8")
    return path


def test_declared_box_assumptions_and_unmodeled_payload_fail_closed(tmp_path):
    template = Path(__file__).parents[1] / "config/payload_conditions.json"
    assert validate_payloads(["--check-all", str(template)]) == 0
    for gripper in ("ag145", "mdpg_1s1"):
        for mass in (.5, 1.0):
            spec = load_payload_spec(template, gripper, mass)
            assert spec["parameter_source"] == "declared_simulation_assumption"
            assert spec["box_size_xyz_m"] == (0.06, 0.04, 0.04)
            assert spec["inertia_kg_m2"] == pytest.approx((
                mass * (0.04**2 + 0.04**2) / 12,
                mass * (0.06**2 + 0.04**2) / 12,
                mass * (0.06**2 + 0.04**2) / 12, 0, 0, 0))
    with pytest.raises(ValueError, match="only for AG145/mDPG"):
        load_payload_spec(template, "none", .5)
    with pytest.raises(SystemExit) as blocked:
        main(["--simulation", "--trajectory", "T1", "--gripper-type", "ag145",
              "--payload-mass", ".5"])
    assert blocked.value.code == 2
    bad = _spec(tmp_path / "bad.json", "ag145", .5)
    doc = json.loads(bad.read_text())
    doc["conditions"]["ag145_p05"]["inertia_kg_m2"] = [-1, 0, 0, 0, 0, 0]
    bad.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match="physically realizable"):
        load_payload_spec(bad, "ag145", .5)
    doc = json.loads(template.read_text())
    doc["conditions"]["ag145_p05"]["box_size_xyz_m"] = [0, 0.04, 0.04]
    bad.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match="must be positive"):
        load_payload_spec(bad, "ag145", .5)


def test_all_condition_preflight(tmp_path):
    conditions = {}
    for gripper in ("ag145", "mdpg_1s1"):
        for mass in (.5, 1.0):
            path = _spec(tmp_path / "one.json", gripper, mass)
            conditions.update(json.loads(path.read_text())["conditions"])
    path = tmp_path / "all.json"
    path.write_text(json.dumps({"schema_version": 1, "conditions": conditions}))
    assert validate_payloads(["--check-all", str(path)]) == 0


@pytest.mark.parametrize("gripper,mass", [("ag145", .5), ("mdpg_1s1", 1.0)])
def test_cli_sends_complete_payload_spec_to_simulation(tmp_path, monkeypatch, gripper, mass):
    from a0509_dynamics_experiment import simulation_executor
    spec_path = _spec(tmp_path / "payload.json", gripper, mass)
    seen = {}
    def fake_execute(*args, **kwargs):
        seen.update({"gripper": args[2], "mass": args[3], **kwargs})
        return tmp_path
    monkeypatch.setattr(simulation_executor, "execute", fake_execute)
    options = (["--mdpg-dynamics-model", "box_approx", "--mdpg-mount-xyz", "0", "0", "0",
                "--mdpg-mount-rpy", "0", "0", "0"] if gripper == "mdpg_1s1" else [])
    assert main(["--simulation", "--trajectory", "T1", "--gripper-type", gripper,
                 "--payload-mass", str(mass), "--payload-spec-file", str(spec_path),
                 *options]) == 0
    assert seen["gripper"] == gripper
    assert seen["mass"] == mass
    assert seen["payload_spec_file"] == str(spec_path)


@pytest.mark.parametrize("gripper,mass", [("ag145", .5), ("mdpg_1s1", 1.0)])
def test_payload_changes_scene_mass_and_inverse_dynamics(tmp_path, gripper, mass):
    if gripper == "ag145":
        from a0509_dynamics_experiment.ag145_model import build_ag145_scene
        scene = build_ag145_scene(tmp_path / "scene.xml")
    else:
        from mdpg_1s1_description.model import build_mdpg_scene
        scene = build_mdpg_scene(tmp_path / "scene.xml", (0, 0, 0), (0, 0, 0),
                                 dynamics_model="box_approx")
    before = mujoco.MjModel.from_xml_path(str(scene))
    spec = load_payload_spec(Path(__file__).parents[1] / "config/payload_conditions.json",
                             gripper, mass)
    attach_payload(scene, spec)
    after = mujoco.MjModel.from_xml_path(str(scene))
    assert after.nq == before.nq == 6
    assert sum(after.body_mass) - sum(before.body_mass) == pytest.approx(mass)
    assert after.body("experiment_payload").mass == pytest.approx(mass)
    assert after.geom("experiment_payload_visual").size == pytest.approx(
        [0.03, 0.02, 0.02])

    def torque(model):
        data = mujoco.MjData(model)
        data.qpos[:] = [-.5, -.5, 1.6, 0, -.6, 0]
        mujoco.mj_forward(model, data)
        data.qacc[:] = [.1, .2, .3, .4, .5, .6]
        mujoco.mj_inverse(model, data)
        return data.qfrc_inverse.copy()

    assert np.linalg.norm(torque(after) - torque(before)) > .01
