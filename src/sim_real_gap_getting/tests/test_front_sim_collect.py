import gzip
import json
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim_real_gap_getting.front_sim_collect import collect_front_sim


FRONT = Path(__file__).resolve().parents[1] / "examples/front"


def test_compressed_sim_trial_has_continuous_100hz_targets_and_feedback(tmp_path):
    pytest.importorskip("mujoco")
    report = json.loads((FRONT / "a0509_front_gripper_clearance.json").read_text())
    if not Path(report["tools"]["ag145"]["scene"]).is_file():
        pytest.skip("tool scene must be regenerated on this machine")
    summary_file = collect_front_sim(FRONT, tmp_path, tool="ag145",
                                     profile="f1_slow_forward", repeats=1)
    summary = json.loads(summary_file.read_text())
    assert summary["result"] == "completed"
    assert summary["execution_scope"] == "simulation_only"
    assert summary["planned_rows"] == summary["written_rows"] == 3601
    assert summary["contact_steps"] == summary["torque_saturation_steps"] == 0
    assert summary["maximum_tracking_error_rad"] < 0.05
    with gzip.open(summary["samples_file"], "rt", encoding="utf-8") as stream:
        rows = [json.loads(line) for line in stream]
    assert rows[0]["phase"] == "setup"
    assert rows[1600]["time_s"] == 16.0
    assert rows[1601]["phase"] == "cycle"
    assert rows[1601]["repeat_no"] == 1
    assert rows[-1]["time_s"] == 36.0
    assert all(row["sample_index"] == index and row["time_s"] == pytest.approx(index / 100)
               for index, row in enumerate(rows))
    assert all(len(row["target_q_rad"]) == len(row["sim_q_rad"]) == 6 for row in rows)
