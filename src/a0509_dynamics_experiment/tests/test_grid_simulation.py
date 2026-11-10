"""Physics checks for the headless multi-robot simulation path."""
from pathlib import Path

import pytest

from a0509_dynamics_experiment.grid_simulation import (_lane_ids, _plan, build_condition_scenes,
                                                        build_grid_scene, CONDITIONS, TORQUE_LIMITS_NM)
from a0509_dynamics_experiment.simulation_executor import MujocoObserver
from a0509_dynamics_experiment.trajectory_generator import BASE_HOME


@pytest.fixture(scope="module")
def scenes(tmp_path_factory):
    pytest.importorskip("mujoco")
    folder = tmp_path_factory.mktemp("grid_scenes")
    spec = Path(__file__).resolve().parents[1] / "config/payload_conditions.json"
    return build_condition_scenes(folder, (0, 0, 0), (0, 0, 0), spec)


def test_grid_has_independent_joint_and_actuator_names(scenes, tmp_path):
    import mujoco as mj
    lanes = [dict(condition=c, gripper=g, payload_kg=p, repeat=1)
             for c, g, p in CONDITIONS for _ in range(3)][:16]
    model = mj.MjModel.from_xml_path(str(build_grid_scene(scenes, lanes, tmp_path / "grid.xml", 4)))
    assert model.nq == model.nv == model.nu == 96
    assert len({lane["prefix"] for lane in lanes}) == 16
    for lane in lanes:
        _lane_ids(mj, model, lane)
        assert len(set(lane["qpos"])) == len(set(lane["dof"])) == len(set(lane["act"])) == 6
    assert len({i for lane in lanes for i in lane["dof"]}) == 96
    data = mj.MjData(model)
    for lane in lanes:
        for addr, value in zip(lane["qpos"], BASE_HOME):
            data.qpos[addr] = value
    mj.mj_forward(model, data)
    assert data.ncon == 0
    data.qacc[:] = 0
    mj.mj_inverse(model, data)
    for lane in (lanes[0], lanes[9], lanes[15]):
        tool = "mdpg_tcp" if lane["gripper"] == "mdpg_1s1" else "ag145_tcp"
        standalone = MujocoObserver(scenes[lane["condition"]], tcp_site=tool)
        expected = standalone.observe(BASE_HOME, (0,) * 6, (0,) * 6)["qfrc_inverse"]
        assert [float(data.qfrc_inverse[i]) for i in lane["dof"]] == pytest.approx(expected, abs=1e-6)


def test_cartesian_targets_are_continuous(scenes):
    observer = MujocoObserver(scenes["bare_p0"])
    plan, targets, tcp, velocity, acceleration = _plan(observer, "T1")
    assert len(plan.points) == len(targets) == len(tcp) == 2201
    assert max(abs(targets[k][j] - targets[k-1][j])
               for k in range(1, len(targets)) for j in range(6)) < 0.003
    assert velocity.shape == acceleration.shape == (2201, 6)


def test_all_campaign_trajectories_preflight_for_each_tool(scenes):
    import mujoco as mj
    for condition, gripper, _mass in CONDITIONS:
        site = "mdpg_tcp" if gripper == "mdpg_1s1" else "ag145_tcp"
        observer = MujocoObserver(scenes[condition], tcp_site=site)
        for number in range(1, 15):
            plan, targets, _tcp, velocity, acceleration = _plan(observer, f"T{number}")
            assert len(plan.points) == len(targets)
            for sample in range(0, len(plan.points), 10):
                observer._set_state(targets[sample], velocity[sample], acceleration[sample])
                mj.mj_inverse(observer.model, observer.data)
                for joint, address in enumerate(observer.dof_addr):
                    assert abs(observer.data.qfrc_inverse[address]) < 0.95 * TORQUE_LIMITS_NM[joint], (condition, number, sample, joint)
