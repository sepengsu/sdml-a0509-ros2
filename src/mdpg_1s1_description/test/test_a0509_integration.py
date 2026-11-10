"""Check selected end-effectors in the active A0509 descriptions."""

from pathlib import Path
import struct
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import xacro
from ament_index_python.packages import get_package_share_directory

from mdpg_1s1_description.model import (BOX_INERTIA_KG_M2, SPEC_MASS_KG,
                                         build_mdpg_scene)


TOOL_XACRO = Path(get_package_share_directory("sdml_a0509_description")) / "urdf/a0509_with_gripper.urdf.xacro"
ARM_SCENE = Path(get_package_share_directory("dsr_description2")) / "mujoco_models/a0509/scene.xml"


def expand(gripper_type, dynamics_model="visual"):
    doc = xacro.process_file(str(TOOL_XACRO), mappings={
        "gripper_type": gripper_type,
        # Identity is an unmeasured display fixture, not an adapter pose.
        "mdpg_mount_xyz": "0 0 0",
        "mdpg_mount_rpy": "0 0 0",
        "mdpg_dynamics_model": dynamics_model,
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
        assert mount.find("origin").attrib["xyz"] == "0 0 0"
        assert all(visual.find("origin").attrib["rpy"] == "3.141592653589793 0 0"
                   for visual in root.findall(".//link[@name='mdpg_base_link']/visual"))
        tcp_joint = next(joint for joint in root.findall(".//joint")
                         if joint.attrib.get("name") == "mdpg_tcp_joint")
        assert [float(value) for value in tcp_joint.find("origin").attrib["xyz"].split()] == [0, 0, 0.09]
        assert tcp_joint.find("origin").attrib["rpy"] == "3.141592653589793 0 0"


def _stl_z_bounds(path):
    raw = path.read_bytes()
    faces = struct.unpack_from("<I", raw, 80)[0]
    vertices = np.ndarray((faces, 3, 3), dtype="<f4", buffer=raw, offset=96,
                          strides=(50, 12, 4))
    return np.min(vertices[:, :, 2]) / 1000, np.max(vertices[:, :, 2]) / 1000


def test_visual_mount_rotates_cad_pad_outward_from_j6_end():
    arm_z_min, arm_z_max = _stl_z_bounds(ARM_SCENE.parent / "assets/A0509_6_0.stl")
    _, housing_z_max = _stl_z_bounds(Path(get_package_share_directory("mdpg_1s1_description"))
                                     / "meshes/pump_cover.stl")
    pad_z_min, _ = _stl_z_bounds(Path(get_package_share_directory("mdpg_1s1_description"))
                                / "meshes/air_pad_contact.stl")
    assert arm_z_min == pytest.approx(-0.066, abs=1e-6)
    assert arm_z_max == pytest.approx(0.0, abs=1e-6)
    assert housing_z_max == pytest.approx(0.0, abs=1e-6)
    assert pad_z_min == pytest.approx(-0.090, abs=1e-6)
    assert -pad_z_min > housing_z_max


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
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        flange = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_6")
        tcp = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "mdpg_tcp")
        local_tcp = data.xmat[flange].reshape(3, 3).T @ (data.site_xpos[tcp] - data.xpos[flange])
        np.testing.assert_allclose(local_tcp, [0, 0, 0.09], atol=1e-6)
        local_tcp_rotation = data.xmat[flange].reshape(3, 3).T @ data.site_xmat[tcp].reshape(3, 3)
        np.testing.assert_allclose(local_tcp_rotation, np.diag([1, -1, -1]), atol=1e-6)


def test_mdpg_box_inertia_requires_opt_in_and_matches_urdf():
    root = expand("mdpg_1s1", "box_approx")
    inertial = root.find(".//link[@name='mdpg_base_link']/inertial")
    assert inertial is not None
    assert float(inertial.find("mass").attrib["value"]) == SPEC_MASS_KG
    inertia = inertial.find("inertia").attrib
    for field, expected in zip(("ixx", "iyy", "izz"), BOX_INERTIA_KG_M2):
        assert float(inertia[field]) == pytest.approx(expected)
    with tempfile.TemporaryDirectory() as folder:
        output = Path(folder) / "box_scene.xml"
        with pytest.raises(ValueError, match="dynamics_model"):
            build_mdpg_scene(output, (0, 0, 0), (0, 0, 0), dynamics_model="unknown")
        build_mdpg_scene(output, (0, 0, 0), (0, 0, 0), dynamics_model="box_approx")
        model = mujoco.MjModel.from_xml_path(str(output))
        bare = mujoco.MjModel.from_xml_path(str(ARM_SCENE))
        assert (model.nq, model.nv) == (bare.nq, bare.nv) == (6, 6)
        assert sum(model.body_mass) - sum(bare.body_mass) == pytest.approx(SPEC_MASS_KG)
        tool = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "mdpg_base_link")
        assert model.body_mass[tool] == pytest.approx(SPEC_MASS_KG)
        np.testing.assert_allclose(model.body_inertia[tool], BOX_INERTIA_KG_M2)
        q = [-0.5, -0.5, 1.6, 0, -0.6, 0]
        def inverse_torque(m):
            data = mujoco.MjData(m)
            data.qpos[:] = q
            mujoco.mj_forward(m, data)
            # Gravity is disabled in the upstream scene; excite the mass matrix.
            data.qacc[:] = [.1, .2, .3, .4, .5, .6]
            mujoco.mj_inverse(m, data)
            return data.qfrc_inverse.copy()
        assert np.linalg.norm(inverse_torque(model) - inverse_torque(bare)) > .01
