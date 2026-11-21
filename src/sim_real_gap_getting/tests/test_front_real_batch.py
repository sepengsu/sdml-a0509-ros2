import hashlib
import json
from pathlib import Path
import sys
import types

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim_real_gap_getting.front_real_batch import (prepare_front_real_batch,
                                                   run_prepared_real_batch)
from sim_real_gap_getting import front_real_batch
from sim_real_gap_getting.lr_trajectory import _scene_path
from sim_real_gap_getting.real_execute import validate_real_plan
from sim_real_gap_getting import real_execute
from sim_real_gap_getting.trajectory import load_trajectory


FRONT = Path(__file__).resolve().parents[1] / "examples/front"
SMALL = Path(__file__).resolve().parents[1] / "examples/small_joint_1.json"


def test_supervised_trial_requires_matching_preflight_file_hash(tmp_path):
    path = tmp_path / "practice_ag145.json"
    report_path = tmp_path / "preflight.json"
    data = json.loads(SMALL.read_text(encoding="utf-8"))
    data.update({"gripper_type": "ag145", "execution_scope": "supervised_real_trial",
                 "real_motion_validated": False, "physical_preflight_report": str(report_path)})
    path.write_text(json.dumps(data), encoding="utf-8")
    sha = hashlib.sha256(path.read_bytes()).hexdigest()
    report_path.write_text(json.dumps({
        "status": "ready_for_supervised_trial", "ready_for_supervised_trial": True,
        "tool": "ag145", "arm_scene": str(SMALL),
        "arm_scene_sha256": hashlib.sha256(SMALL.read_bytes()).hexdigest(),
        "files": {"cycle": {"path": str(path), "sha256": sha}}
    }), encoding="utf-8")
    assert validate_real_plan(load_trajectory(path), "ag145", supervised_trial=True)
    data["description"] = "changed after preflight"
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="differs from the preflight"):
        validate_real_plan(load_trajectory(path), "ag145", supervised_trial=True)


def test_real_batch_refuses_blocked_report(tmp_path):
    path = tmp_path / "preflight.json"
    path.write_text(json.dumps({"status": "blocked", "ready_for_supervised_trial": False}),
                    encoding="utf-8")
    with pytest.raises(ValueError, match="has not passed"):
        run_prepared_real_batch(path)


def test_ag145_f1_stays_blocked_with_400mm_corridor_and_side_margins(tmp_path,
                                                                      monkeypatch):
    audit = json.loads((FRONT / "a0509_front_gripper_clearance.json").read_text())
    monkeypatch.setattr(front_real_batch, "audit_front_grippers", lambda *a, **k: audit)
    path = prepare_front_real_batch(
        FRONT, _scene_path(), tmp_path,
        tool="ag145", profiles=("f1_slow_forward",), repeats=1,
        corridor_y_min_mm=-150, corridor_y_max_mm=250,
        lower_obstacle_top_z_mm=0,
        physical_tool_confirmed=True, other_obstacles_clear_confirmed=True)
    report = json.loads(path.read_text(encoding="utf-8"))
    assert report["status"] == "blocked"
    assert not report["ready_for_supervised_trial"]
    assert any("20 mm side margins" in reason
               for reason in report["model_checks"]["f1_slow_forward"]["reasons"])
    assert not report["files"]


def test_rt_command_session_matches_100hz_and_restores_monitoring(monkeypatch):
    service_names = ("StopRtControl", "SetRtControlInput", "SetRtControlOutput",
                     "StartRtControl")
    srv = types.ModuleType("dsr_msgs2.srv")
    for name in service_names:
        setattr(srv, name, type(name, (), {}))
    package = types.ModuleType("dsr_msgs2")
    package.srv = srv
    monkeypatch.setitem(sys.modules, "dsr_msgs2", package)
    monkeypatch.setitem(sys.modules, "dsr_msgs2.srv", srv)
    calls = []
    monkeypatch.setattr(real_execute, "_call", lambda _node, typ, name, **kwargs:
                        calls.append((typ.__name__, name, kwargs)))
    real_execute._set_rt_session(object(), streaming=True)
    real_execute._set_rt_session(object(), streaming=False)
    assert [item[0] for item in calls] == list(service_names) * 2
    assert calls[1][2]["request_fields"]["period"] == 0.01
    assert calls[2][2]["request_fields"]["period"] == 0.01
    assert calls[5][2]["request_fields"]["period"] == 0.0
    assert calls[6][2]["request_fields"]["period"] == 0.001
