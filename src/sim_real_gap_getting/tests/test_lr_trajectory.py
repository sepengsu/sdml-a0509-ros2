"""Acceptance checks for the generated one-cycle simulation plans."""

import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim_real_gap_getting.lr_trajectory import _expand, _setup_plus_cycles, generate_dataset
from sim_real_gap_getting.real_execute import validate_real_plan
from sim_real_gap_getting.trajectory import load_trajectory


EXAMPLES = Path(__file__).resolve().parents[1] / "examples" / "lr"


def test_forward_setup_joins_all_cycles_and_remains_simulation_only():
    setup = load_trajectory(EXAMPLES / "a0509_lr_forward_10cm_setup.json")
    assert len(setup.points) == 1001
    validation = json.loads((EXAMPLES / "a0509_lr_validation.json").read_text(encoding="utf-8"))
    assert validation["setup"]["tcp_span_mm"][0] == pytest.approx(100, abs=1)
    assert validation["setup"]["simulation_preflight"]["passed"] is True
    for profile in ("p1_slow", "p2_medium", "p3_variable", "p4_reversal", "p5_multijoint"):
        cycle = load_trajectory(EXAMPLES / f"a0509_lr_{profile}.json")
        assert setup.points[-1].q_rad == pytest.approx(cycle.points[0].q_rad, abs=1e-9)
    with pytest.raises(ValueError, match="simulation-only"):
        validate_real_plan(setup, "none")


@pytest.mark.parametrize("profile", ["p1_slow", "p2_medium", "p3_variable",
                                       "p4_reversal", "p5_multijoint"])
def test_generated_cycle_is_closed_and_cannot_be_streamed_to_real_robot(profile):
    trajectory = load_trajectory(EXAMPLES / f"a0509_lr_{profile}.json")
    assert trajectory.points[0].q_rad == pytest.approx(trajectory.points[-1].q_rad, abs=1e-10)
    assert trajectory.points[0].dq_rad_s == trajectory.points[-1].dq_rad_s == (0.0,) * 6
    assert trajectory.points[0].ddq_rad_s2 == trajectory.points[-1].ddq_rad_s2 == (0.0,) * 6
    assert len(trajectory.points) == round(trajectory.duration_s * 100) + 1
    with pytest.raises(ValueError, match="simulation-only"):
        validate_real_plan(trajectory, "none")


def test_repeat_expansion_has_no_duplicate_or_missing_boundary_samples():
    trajectory = load_trajectory(EXAMPLES / "a0509_lr_p4_reversal.json")
    expanded = _expand(trajectory.points, 3)
    cycle_ticks = len(trajectory.points) - 1
    assert len(expanded) == 3 * cycle_ticks + 1
    assert expanded[cycle_ticks].q_rad == pytest.approx(expanded[cycle_ticks + 1].q_rad, abs=1e-6)
    assert expanded[cycle_ticks + 1].time_s - expanded[cycle_ticks].time_s == pytest.approx(.01)
    assert expanded[cycle_ticks].dq_rad_s == expanded[2 * cycle_ticks].dq_rad_s == (0.0,) * 6
    assert expanded[cycle_ticks + 1].dq_rad_s == trajectory.points[1].dq_rad_s
    setup = load_trajectory(EXAMPLES / "a0509_lr_forward_10cm_setup.json")
    combined = _setup_plus_cycles(setup.points, trajectory.points, 3)
    assert len(combined) == len(setup.points) + 3 * cycle_ticks
    assert combined[len(setup.points) - 1].q_rad == pytest.approx(
        combined[len(setup.points)].q_rad, abs=1e-6)


def test_lateral_band_rejects_width_beyond_room_limit(tmp_path):
    with pytest.raises(ValueError, match="lateral band"):
        generate_dataset(tmp_path, half_width_mm=120, center_offset_mm=31)
