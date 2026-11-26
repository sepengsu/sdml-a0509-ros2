import hashlib
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import xacro
from xacro import substitution_args
import yaml

from a0509_dynamics_experiment.ag145_model import build_ag145_scene, source_paths
from a0509_dynamics_experiment.tcc1_conditions import build_tcc1_condition_scene
from tcc1_tool_changer_description.model import package_share, write_scene


SRC = Path(__file__).resolve().parents[2]
SHARE = package_share()
Q = (-0.5, -0.5, 1.6, 0.0, -0.6, 0.1)


@pytest.fixture(scope="module")
def ag145(tmp_path_factory):
    return build_ag145_scene(tmp_path_factory.mktemp("ag145") / "scene.xml")


def test_raw_cad_matches_manifest():
    manifest = json.loads((SHARE / "config/mesh_manifest.json").read_text(encoding="utf-8"))
    for item in manifest["models"].values():
        raw = (SHARE / item["mesh_file"]).read_bytes()
        assert hashlib.sha256(raw).hexdigest() == item["mesh_sha256"]
        faces = struct.unpack_from("<I", raw, 80)[0]
        vertices = np.ndarray((faces, 3, 3), dtype="<f4", buffer=raw, offset=96, strides=(50, 12, 4))
        assert np.ptp(vertices.reshape(-1, 3), axis=0) == pytest.approx(item["bbox_dimensions_mm"], abs=0.03)


def _expand(monkeypatch, gripper, changer="tcc1", rack="true", mode="visual", config_path=None):
    # Resolves the source workspace on Windows without an installed ROS graph.
    packages = {p.parent.name: p.parent.as_posix() for p in SRC.rglob("package.xml")}
    monkeypatch.setattr(substitution_args, "_eval_find", packages.__getitem__)
    path = Path(packages["sdml_a0509_description"]) / "urdf/a0509_with_gripper.urdf.xacro"
    mappings = {"gripper_type": gripper, "tool_changer_type": changer, "tool_rack_enabled": rack,
                "tool_changer_dynamics_model": mode}
    if config_path:
        mappings["tcc1_config"] = config_path.as_posix()
    doc = xacro.process_file(str(path), mappings=mappings)
    return ET.fromstring(doc.toxml())


@pytest.mark.parametrize(("condition", "gripper", "gripper_link", "tcp"), [
    ("tcc1_master_p0", "none", None, None),
    ("tcc1_ag145_p0", "ag145", "ag145_base_link", "ag145_tcp"),
    ("tcc1_mdpg_p0", "mdpg_1s1", "mdpg_base_link", "mdpg_tcp"),
])
def test_urdf_and_mjcf_mounts_agree(tmp_path, monkeypatch, condition, gripper, gripper_link, tcp):
    urdf = _expand(monkeypatch, gripper)
    scene = build_tcc1_condition_scene(condition, tmp_path / "model.xml", mode="visual", rack=True)
    model = mujoco.MjModel.from_xml_path(str(scene))
    assert (model.nq, model.nv) == (6, 6)
    chain = {"tcc1_master": "link_6", "tcc1_rack": "world"}
    if gripper_link:
        chain.update(tcc1_tool="tcc1_master")
        chain[gripper_link] = "tcc1_tool"
    for child, parent in chain.items():
        joint = urdf.find(f"joint/child[@link='{child}']/..")
        assert joint.get("type") == "fixed"
        assert joint.find("parent").get("link") == parent
        body = model.body(child)
        assert model.body(int(body.parentid[0])).name == parent
        xyz = np.fromstring(joint.find("origin").get("xyz"), sep=" ")
        assert body.pos == pytest.approx(xyz)
        rpy = np.fromstring(joint.find("origin").get("rpy"), sep=" ")
        quat = np.empty(4)
        mujoco.mju_euler2Quat(quat, rpy, "XYZ")
        assert abs(quat @ body.quat) == pytest.approx(1.0)
    if tcp:
        joint = urdf.find(f"joint/child[@link='{tcp}']/..")
        assert joint.find("parent").get("link") == gripper_link
        assert model.site(tcp).pos == pytest.approx(np.fromstring(joint.find("origin").get("xyz"), sep=" "))
    else:
        assert urdf.find("link[@name='tcc1_tool']") is None
        assert ET.parse(scene).find(".//body[@name='tcc1_tool']") is None


def _dynamics(scene):
    model = mujoco.MjModel.from_xml_path(str(scene))
    state = mujoco.MjData(model)
    state.qpos[:] = Q
    state.qvel[:] = (0.1, -0.2, 0.3, 0.2, 0.1, -0.1)
    mujoco.mj_forward(model, state)
    tcp = state.site("ag145_tcp").xpos.copy()
    state.qacc[:] = (0.25, 0.4, -0.1, 0.0, 0.1, -0.2)
    mujoco.mj_inverse(model, state)
    return model, state.qfrc_inverse.copy(), tcp


def test_off_and_rack_leave_dynamics_unchanged(tmp_path, ag145):
    off = write_scene(ag145, tmp_path / "off.xml", "ag145")
    assert off == ag145
    orig, baseline, tcp = _dynamics(ag145)
    assert np.array_equal(_dynamics(off)[1], baseline)
    preview = write_scene(ag145, tmp_path / "preview.xml", "ag145", tool_changer_type="tcc1")
    visual, without_rack, new_tcp = _dynamics(preview)
    assert visual.body_mass.sum() == pytest.approx(orig.body_mass.sum())
    assert np.linalg.norm(new_tcp - tcp) == pytest.approx(0.042)
    scene = write_scene(ag145, tmp_path / "rack.xml", "ag145", tool_changer_type="tcc1", rack=True)
    with_rack, torque, _ = _dynamics(scene)
    assert with_rack.body("tcc1_rack").mass[0] == 0
    np.testing.assert_allclose(torque, without_rack, atol=1e-11)


def test_dynamics_requires_parameters_and_preserves_payload(tmp_path):
    output = tmp_path / "dynamic.xml"
    config = yaml.safe_load((SHARE / "config/tcc1_parameters.yaml").read_text(encoding="utf-8"))
    path = tmp_path / "config.yaml"
    config["components"]["master"]["mass_kg"] = None
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    with pytest.raises(ValueError, match=r"master.mass_kg"):
        build_tcc1_condition_scene("tcc1_master_p0", output, config_path=path)
    assert not output.exists()
    for name in ("master", "tool"):
        config["components"][name].update(mass_kg=0.2, com_m=[0, 0, 0.02],
                                           inertia_kg_m2=[0.0002, 0.0002, 0.0003, 0, 0, 0],
                                           parameter_source="synthetic pytest fixture", verified=False)
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    payload = SRC / "a0509_dynamics_experiment/config/payload_conditions.json"
    build_tcc1_condition_scene("tcc1_mdpg_p05", output, config_path=path, payload_spec=payload, rack=True)
    model = mujoco.MjModel.from_xml_path(str(output))
    load_bare = mujoco.MjModel.from_xml_path(str(source_paths()[0]))
    assert model.body("experiment_payload").mass[0] == pytest.approx(0.5)
    assert model.body("tcc1_master").mass[0] == pytest.approx(0.2)
    assert model.body_mass.sum() - load_bare.body_mass.sum() == pytest.approx(0.62 + 0.5 + 0.4)
    config["components"]["tool"]["inertia_kg_m2"][2] = 1.0
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    with pytest.raises(ValueError, match="principal moments"):
        build_tcc1_condition_scene("tcc1_mdpg_p05", output, config_path=path, payload_spec=payload)


def test_catalog_inertia_and_optional_parts(tmp_path, ag145, monkeypatch):
    config = yaml.safe_load((SHARE / "config/tcc1_parameters.yaml").read_text(encoding="utf-8"))
    cad = json.loads((SHARE / "config/nominal_cad_manifest.json").read_text(encoding="utf-8"))
    mounted = write_scene(ag145, tmp_path / "nominal.xml", "ag145", tool_changer_type="tcc1", mode="dynamics")
    model, torque, tcp = _dynamics(mounted)
    baseline, off_torque, off_tcp = _dynamics(ag145)
    assert model.body_mass.sum() - baseline.body_mass.sum() == pytest.approx(0.593)
    assert np.linalg.norm(tcp - off_tcp) == pytest.approx(0.042)
    assert np.linalg.norm(torque - off_torque) > 0.01
    assert "tcc1_master_air_coupler" not in [model.body(i).name for i in range(model.nbody)]
    robot = _expand(monkeypatch, "ag145", mode="dynamics")
    mjcf = ET.parse(mounted)
    rotation = np.diag([1, -1, -1])
    for part, mass in (("master", 0.448), ("tool", 0.145)):
        record = cad["parts"][part]
        assert hashlib.sha256((SHARE / record["mesh"]).read_bytes()).hexdigest() == record["mesh_sha256"]
        expected_com = (np.array(config["components"][part]["link_to_raw_cad"]["xyz_m"])
                        + rotation @ np.array(record["uniform_volume_centroid_mm"]) / 1000)
        tensor = mass * rotation @ np.array(record["uniform_volume_inertia_per_unit_mass_m2"]) @ rotation.T
        expected_inertia = tensor[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]]
        urdf = robot.find(f"link[@name='tcc1_{part}']/inertial")
        assert float(urdf.find("mass").get("value")) == mass
        assert np.fromstring(urdf.find("origin").get("xyz"), sep=" ") == pytest.approx(expected_com)
        assert [float(urdf.find("inertia").get(k)) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")] == (
            pytest.approx(expected_inertia, abs=1e-14))
        inertial = mjcf.find(f".//body[@name='tcc1_{part}']/inertial")
        assert np.fromstring(inertial.get("fullinertia"), sep=" ") == pytest.approx(expected_inertia, abs=1e-14)

    config["components"]["master"]["mass_kg"] = 0.470
    accessory = config["accessories"]["master"]["air_coupler"]
    accessory.update(mass_kg=0.05, com_m=[0.03, 0, 0.02],
                     inertia_kg_m2=[2e-5, 2e-5, 2e-5, 0, 0, 0], parameter_source="synthetic test")
    path = tmp_path / "with_accessory.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    output = write_scene(ag145, tmp_path / "with_accessory.xml", "ag145", tool_changer_type="tcc1",
                         mode="dynamics", config_path=path)
    model = mujoco.MjModel.from_xml_path(str(output))
    assert model.body_mass.sum() - baseline.body_mass.sum() == pytest.approx(0.470 + 0.145 + 0.05)
    assert model.body("tcc1_master_air_coupler").mass[0] == pytest.approx(0.05)
    assert np.array_equal(model.body("tcc1_master_air_coupler").ipos, [0.03, 0.0, 0.02])
    robot = _expand(monkeypatch, "ag145", mode="dynamics", config_path=path)
    assert float(robot.find("link[@name='tcc1_master_air_coupler']/inertial/mass").get("value")) == 0.05
    moment = float(robot.find("link[@name='tcc1_master']/inertial/inertia").get("ixx"))
    assert moment == pytest.approx(0.470 * config["components"]["master"]["inertia_per_unit_mass_m2"][0])
