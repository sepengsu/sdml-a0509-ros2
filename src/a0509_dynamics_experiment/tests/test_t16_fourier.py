import math

import pytest

from a0509_dynamics_experiment.dynamics10_safety import validate_joint_plan
from a0509_dynamics_experiment.t16_fourier import evaluate, load_spec
from a0509_dynamics_experiment.trajectory_generator import TrajectoryConfig, generate
from a0509_dynamics_experiment.run_experiment import main


def test_t16_fourier_endpoint_and_analytic_derivatives():
    spec = load_spec()
    assert spec["optimizer"]["success"] is True
    assert spec["optimized"]["condition_number"] < spec["initial"]["condition_number"]
    assert spec["optimized"]["effective_rank"] == spec["initial"]["effective_rank"]
    home = spec["home_rad"]
    free = spec["free_coefficients_rad_s"]
    for endpoint in (0.0, 10.0):
        q, dq, ddq = evaluate(free, home, endpoint)
        assert q == pytest.approx(home, abs=1e-12)
        assert dq == pytest.approx((0,) * 6, abs=1e-12)
        assert ddq == pytest.approx((0,) * 6, abs=1e-12)
    dt = 1e-4
    before = evaluate(free, home, 3.2-dt)
    middle = evaluate(free, home, 3.2)
    after = evaluate(free, home, 3.2+dt)
    for j in range(6):
        assert middle[1][j] == pytest.approx((after[0][j]-before[0][j])/(2*dt), abs=1e-8)
        assert middle[2][j] == pytest.approx((after[1][j]-before[1][j])/(2*dt), abs=1e-8)


def test_t16_generated_plan_is_100_hz_c2_and_within_joint_limits():
    plan = generate(TrajectoryConfig("T16"))
    assert len(plan.points) == 1201
    assert plan.points[-1].time_s == pytest.approx(12.0)
    assert validate_joint_plan(plan)["validated_samples"] == 1201
    assert plan.points[0].q_rad == pytest.approx(plan.points[-1].q_rad)
    assert all(math.isfinite(x) for point in plan.points for x in point.q_rad)


def test_t16_rejects_unvalidated_variants_and_real_motion():
    for kwargs in ({"time_scale": 2.25}, {"speed": "medium"},
                   {"sample_hz": 50}, {"joint_amplitude_deg": 4.0}):
        with pytest.raises(ValueError):
            generate(TrajectoryConfig("T16", **kwargs))
    with pytest.raises(SystemExit):
        main(["--trajectory", "T16", "--execute"])
    with pytest.raises(SystemExit):
        main(["--preview", "--trajectory", "T16", "--gripper-type", "none"])
