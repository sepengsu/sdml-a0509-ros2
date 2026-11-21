import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim_real_gap_getting.front_trajectory import _setup_plus_cycles
from sim_real_gap_getting.front_gripper_audit import clearance_decision
from sim_real_gap_getting.front_real_review import review_front_real
from sim_real_gap_getting.real_execute import validate_real_plan
from sim_real_gap_getting.trajectory import load_trajectory


FRONT = Path(__file__).resolve().parents[1] / "examples" / "front"
NAMES = ("f1_slow_forward", "f2_medium_forward", "f3_front_postures",
         "f4_wrist_and_holds")


def test_setup_and_four_cycles_are_joined_and_model_validated():
    setup = load_trajectory(FRONT / "a0509_front_setup_10cm.json")
    report = json.loads((FRONT / "a0509_front_validation.json").read_text(encoding="utf-8"))
    assert report["setup"]["tcp_span_mm"][0] == pytest.approx(100, abs=2)
    assert report["setup"]["tcp_span_mm"][2] > 50
    assert report["setup"]["simulation_preflight"]["passed"] is True
    for name in NAMES:
        cycle = load_trajectory(FRONT / f"a0509_front_{name}.json")
        data = report["paths"][name]
        assert setup.points[-1].q_rad == pytest.approx(cycle.points[0].q_rad, abs=1e-9)
        assert cycle.points[-1].q_rad == pytest.approx(cycle.points[0].q_rad, abs=1e-9)
        assert len(cycle.points) == round(cycle.duration_s * 100) + 1
        assert data["tcp_span_mm"][1] < 400
        assert data["simulation_preflight"]["passed"] is True
        assert data["simulation_preflight"]["contact_steps"] == 0
        assert data["simulation_preflight"]["torque_limit_samples"] == 0
        with pytest.raises(ValueError, match="simulation-only"):
            validate_real_plan(cycle, "none")


def test_four_paths_add_speed_posture_and_wrist_variation():
    report = json.loads((FRONT / "a0509_front_validation.json").read_text(encoding="utf-8"))["paths"]
    slow, medium, postures, wrist = (report[name] for name in NAMES)
    assert slow["tcp_span_mm"][0] == pytest.approx(medium["tcp_span_mm"][0], abs=.1)
    assert medium["max_velocity_rad_s"][1] > slow["max_velocity_rad_s"][1]
    assert postures["tcp_span_mm"][1] > 15
    assert postures["tcp_span_mm"][2] > 20
    assert postures["position_max_rad"][1] - postures["position_min_rad"][1] > .4
    assert all(wrist["position_max_rad"][j] - wrist["position_min_rad"][j] > .2
               for j in (3, 4, 5))


def test_setup_plus_three_cycles_has_no_duplicate_time_at_seam():
    setup = load_trajectory(FRONT / "a0509_front_setup_10cm.json")
    cycle = load_trajectory(FRONT / "a0509_front_f1_slow_forward.json")
    combined = _setup_plus_cycles(setup.points, cycle.points, 3)
    assert len(combined) == len(setup.points) + 3 * (len(cycle.points) - 1)
    assert combined[len(setup.points)].time_s - combined[len(setup.points) - 1].time_s == pytest.approx(.01)
    assert combined[-1].q_rad == pytest.approx(setup.points[-1].q_rad)


def test_ag145_conservative_envelope_exceeds_400mm_and_real_is_blocked():
    audit = json.loads((FRONT / "a0509_front_gripper_clearance.json").read_text(encoding="utf-8"))
    assert audit["real_robot_executable"] is False
    assert audit["tools"]["ag145"]["whole_campaign_lateral_span_mm"] > 400
    assert audit["tools"]["ag145"]["whole_campaign_lateral_span_passed"] is False
    assert audit["tools"]["mdpg_1s1"]["whole_campaign_lateral_span_passed"] is True
    for tool in ("ag145", "mdpg_1s1"):
        for path in audit["tools"][tool]["paths"].values():
            assert path["tool_lateral_span_passed"] is True
            assert path["tool_lateral_span_mm"] <= 400
            assert path["model_contact_steps_without_lower_obstacle"] == 0
            assert path["tool_mass_simulation_preflight"]["passed"] is True
            assert path["model_clearance_passed"] is None
            assert path["measured_corridor_passed"] is None
            assert path["real_robot_executable"] is False


def test_lower_obstacle_clearance_fails_closed_at_threshold():
    assert clearance_decision(405.76, None, 30)["model_clearance_passed"] is None
    assert clearance_decision(405.76, 376, 30)["model_clearance_passed"] is False
    assert clearance_decision(405.76, 375, 30)["model_clearance_passed"] is True


def test_real_review_keeps_ag145_f3_and_unmeasured_paths_simulation_only():
    report = review_front_real(FRONT / "a0509_front_gripper_clearance.json",
                               FRONT / "a0509_front_validation.json")
    ag_f3 = report["tools"]["ag145"]["paths"]["f3_front_postures"]
    assert ag_f3["model_width_mm"] > 400
    assert "conservative whole-robot width exceeds available width" in ag_f3["reasons"]
    for tool in report["tools"].values():
        for path in tool["paths"].values():
            assert path["execution_scope"] == "simulation_only"
            assert path["real_robot_executable"] is False
            assert path["reasons"]
