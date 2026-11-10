"""Check selected end-effectors in the active A0509 descriptions."""

from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

from mdpg_1s1_description.model import build_mdpg_scene


TOOL_XACRO = Path(get_package_share_directory("sdml_a0509_description")) / "urdf/a0509_with_gripper.urdf.xacro"
ARM_SCENE = Path(get_package_share_directory("dsr_description2")) / "mujoco_models/a0509/scene.xml"


def expand(gripper_type):
    doc = xacro.process_file(str(TOOL_XACRO), mappings={
        "gripper_type": gripper_type,
        # Explicit identity transform is only a display/test pose, not a measured mount.
        "mdpg_mount_xyz": "0 0 0",
        "mdpg_mount_rpy": "0 0 0",
    })
    return ET.fromstring(doc.toxml())


@pytest.mark.parametrize("gripper,expected_link,other_link", [
    ("none", None, None),
    ("ag145", "ag145_base_link", "mdpg_base_link"),
    ("mdpg_1s1", "mdpg_base_link", "ag145_base_link"),
])
def test_xacro_tool_selection(gripper, expected_link, other_link):
    root = expand(gripper)
    assert root.find(".//link[@name='link_6']") is not None
    if expected_link:
        assert root.find(f".//link[@name='{expected_link}']") is not None
        mount = next((joint for joint in root.findall(".//joint")
                      if joint.find("child") is not None
                      and joint.find("child").attrib.get("link") == expected_link), None)
        assert mount is not None
        assert mount.find("parent").attrib["link"] == "link_6"
        assert root.find(f".//link[@name='{other_link}']") is None
    else:
        assert root.find(".//link[@name='ag145_base_link']") is None
        assert root.find(".//link[@name='mdpg_base_link']") is None
    if gripper == "mdpg_1s1":
        assert root.find(".//link[@name='mdpg_tcp']") is not None
        assert len(root.findall(".//link[@name='mdpg_base_link']/visual")) == 6
        assert root.find(".//link[@name='mdpg_base_link']/inertial") is None


def test_mdpg_mujoco_visual_only_and_mount_required():
    with tempfile.TemporaryDirectory() as folder:
        output = Path(folder) / "scene.xml"
        with pytest.raises(ValueError, match="TODO"):
            build_mdpg_scene(output, None, None)
        build_mdpg_scene(output, (0, 0, 0), (0, 0, 0))
        model = mujoco.MjModel.from_xml_path(str(output))
        arm = mujoco.MjModel.from_xml_path(str(ARM_SCENE))
        assert (model.nq, model.nv) == (arm.nq, arm.nv) == (6, 6)
        assert sum(model.body_mass) == pytest.approx(sum(arm.body_mass))
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "mdpg_tcp") >= 0
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ag145_tcp") < 0
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i)
                 for i in range(model.ngeom)]
        visuals = [i for i, name in enumerate(names) if name and name.startswith("mdpg_")]
        assert len(visuals) == 6
        assert all(model.geom_contype[i] == 0 and model.geom_conaffinity[i] == 0
                   for i in visuals)
