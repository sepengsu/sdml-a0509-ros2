import pytest

from a0509_dynamics_experiment.dynamics10_safety import validate_joint_plan
from a0509_dynamics_experiment.trajectory_generator import (
    DYNAMICS10_IDS, EXCITATIONS, EXTENDED_IDS, TrajectoryConfig, generate,
)
from a0509_dynamics_experiment.simulation_executor import SimTimeVelocityFilter
from a0509_dynamics_experiment.run_experiment import main


@pytest.mark.parametrize("trajectory_id", DYNAMICS10_IDS)
def test_joint_excitation_is_smooth_analytic_and_within_limits(trajectory_id):
    plan = generate(TrajectoryConfig(trajectory_id))
    assert len(plan.points) >= 1401
    assert plan.points[0].q_rad == pytest.approx(plan.points[-1].q_rad)
    assert plan.points[0].dq_rad_s == (0.0,) * 6
    assert plan.points[-1].ddq_rad_s2 == (0.0,) * 6
    assert validate_joint_plan(plan)["validated_samples"] == len(plan.points)
    for index in (200, len(plan.points)//2, len(plan.points)-201):
        before, point, after = plan.points[index-1:index+2]
        dt = after.time_s - before.time_s
        for joint in range(6):
            estimated_dq = (after.q_rad[joint] - before.q_rad[joint]) / dt
            estimated_ddq = (after.dq_rad_s[joint] - before.dq_rad_s[joint]) / dt
            assert point.dq_rad_s[joint] == pytest.approx(estimated_dq, abs=.003)
            assert point.ddq_rad_s2[joint] == pytest.approx(estimated_ddq, abs=.02)


def test_split_and_shifted_postures():
    assert [EXCITATIONS[i]["split"] for i in DYNAMICS10_IDS] == ["train"]*7 + ["validation"] + ["test"]*2
    assert generate(TrajectoryConfig("T13")).points[0].q_rad != generate(TrajectoryConfig("T14")).points[0].q_rad
    assert generate(TrajectoryConfig("T13")).points[0].q_rad != generate(TrajectoryConfig("T10")).points[0].q_rad
    assert [EXCITATIONS[i]["split"] for i in EXTENDED_IDS] == ["train", "test"]
    assert generate(TrajectoryConfig("T15")).points[0].q_rad != generate(TrajectoryConfig("T16")).points[0].q_rad


def test_sim_time_acceleration_filter_rejects_duplicate_stamp():
    filt = SimTimeVelocityFilter()
    for k in range(5):
        filt.add(.01*k, [2*.01*k] * 6)
    filt.add(.04, [999] * 6)
    assert filt.acceleration() == pytest.approx((2.0,) * 6)


def test_new_joint_cli_is_simulation_only_and_batch_dispatches(monkeypatch, tmp_path):
    with pytest.raises(SystemExit):
        main(["--trajectory", "T5"])
    for trajectory_id in EXTENDED_IDS:
        with pytest.raises(SystemExit):
            main(["--trajectory", trajectory_id, "--execute"])
    from a0509_dynamics_experiment import dynamics10_dataset, simulation_executor, real_executor
    calls = []
    monkeypatch.setattr(simulation_executor, "execute", lambda config, *a, **k: calls.append(config.trajectory_id) or tmp_path)
    monkeypatch.setattr(real_executor, "execute", lambda *a, **k: pytest.fail("real executor called"))
    monkeypatch.setattr(dynamics10_dataset, "summarize", lambda *a: {})
    assert main(["--simulation", "--trajectory-set", "dynamics10", "--gripper-type", "ag145"]) == 0
    assert calls == list(DYNAMICS10_IDS)
