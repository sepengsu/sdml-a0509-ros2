"""Selected-tool isolation and native F1-F4 three-cycle collection."""
import gzip
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
for package in ("sim_real_gap_getting", "a0509_dynamics_experiment", "mdpg_1s1_description"):
    sys.path.insert(0, str(ROOT / "src" / package))

from sim_real_gap_getting import front_sim_batch, front_gripper_audit
from sim_real_gap_getting.front_trajectory import PROFILE_NAMES
from sim_real_gap_getting.trajectory import load_trajectory


class FrontToolSelectionTests(unittest.TestCase):
    def test_cli_accepts_all_three_single_tools(self):
        for tool in ("none", "ag145", "mdpg_1s1"):
            with self.subTest(tool=tool), patch.object(front_sim_batch, "collect_front_batch") as collect:
                self.assertEqual(front_sim_batch.main(["--tool", tool, "--repeats", "3"]), 0)
                self.assertEqual(collect.call_args.kwargs["tool"], tool)
                self.assertEqual(collect.call_args.kwargs["repeats"], 3)

    def test_none_scene_does_not_build_grippers_or_modify_arm(self):
        with tempfile.TemporaryDirectory() as folder:
            arm = Path(folder) / "arm.xml"
            arm.write_text("<mujoco/>")
            scenes = front_gripper_audit._build_scenes(
                Path(folder) / "models", (0, 0, 0), (0, 0, 0), arm_scene=arm, tools=("none",))
            self.assertEqual(scenes, {"none": arm.resolve()})
            self.assertEqual(arm.read_text(), "<mujoco/>")
            self.assertEqual(list((Path(folder) / "models").iterdir()), [])

    def test_native_three_tools_four_profiles_three_cycles(self):
        import mujoco

        source = ROOT / "src/sim_real_gap_getting/examples/front"
        original = ROOT / "src/doosan-robot2/dsr_description2/mujoco_models/a0509/scene.xml"
        original_sha = hashlib.sha256(original.read_bytes()).hexdigest()
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            # Local test fixture: the unchanged arm plus a rear-wall model.
            tree = ET.parse(original)
            root = tree.getroot()
            root.find("include").set("file", (original.parent / "a0509.xml").as_posix())
            root.find("compiler").set("meshdir", original.parent.parent.as_posix())
            ET.SubElement(root.find("worldbody"), "geom", name="test_rear_wall", type="box",
                          pos="-0.375 0 0.8", size="0.025 1 0.8", contype="1", conaffinity="1")
            scene = output / "rear_wall.xml"
            tree.write(scene, encoding="utf-8", xml_declaration=True)
            baseline = mujoco.MjModel.from_xml_path(str(scene))
            for tool in ("none", "ag145", "mdpg_1s1"):
                with self.subTest(tool=tool):
                    path = front_sim_batch.collect_front_batch(
                        source, output / tool, tool=tool, repeats=3, arm_scene=scene)
                    manifest = json.loads(path.read_text())
                    self.assertEqual(manifest["result"], "completed")
                    self.assertEqual(manifest["tool_models"], [tool])
                    self.assertEqual(manifest["expected_trials"], 4)
                    self.assertEqual(len(manifest["completed_trials"]), 4)
                    audit = json.loads((path.parent / "inputs/a0509_front_gripper_clearance.json").read_text())
                    self.assertEqual(set(audit["tools"]), {tool})
                    model = mujoco.MjModel.from_xml_path(audit["tools"][tool]["scene"])
                    self.assertEqual(model.dof_damping.tolist(), baseline.dof_damping.tolist())
                    if tool == "none":
                        self.assertEqual(model.body_mass.tolist(), baseline.body_mass.tolist())
                    for trial in manifest["completed_trials"]:
                        counts = {0: 0, 1: 0, 2: 0, 3: 0}
                        with gzip.open(trial["samples_file"], "rt", encoding="utf-8") as stream:
                            for line in stream:
                                row = json.loads(line)
                                self.assertEqual(row["tool_model"], tool)
                                counts[row["repeat_no"]] += 1
                        cycle = load_trajectory(source / f"a0509_front_{trial['profile']}.json")
                        self.assertEqual([counts[i] for i in (1, 2, 3)], [len(cycle.points)-1]*3)
                        self.assertGreater(counts[0], 0)
                        summary = json.loads(Path(trial["summary_file"]).read_text())
                        self.assertEqual(summary["planned_rows"], summary["written_rows"])
                        self.assertEqual(summary["torque_saturation_steps"], 0)
                        self.assertEqual(summary["contact_steps"], 0)
            self.assertEqual(hashlib.sha256(original.read_bytes()).hexdigest(), original_sha)


if __name__ == "__main__":
    unittest.main()
