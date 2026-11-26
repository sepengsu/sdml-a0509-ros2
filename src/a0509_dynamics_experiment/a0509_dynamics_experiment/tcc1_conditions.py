"""Build an individual TCC1 research scene outside the legacy collection grid."""

from pathlib import Path

from tcc1_tool_changer_description.model import load_configuration, write_scene

from .ag145_model import AG145_BOX_GRASP_ANGLE_RAD, build_ag145_scene, source_paths
from .payload_model import attach_payload, load_payload_spec


TCC1_CONDITIONS = {
    "tcc1_master_p0": ("none", 0.0),
    "tcc1_ag145_p0": ("ag145", 0.0),
    "tcc1_ag145_p05": ("ag145", 0.5),
    "tcc1_ag145_p10": ("ag145", 1.0),
    "tcc1_mdpg_p0": ("mdpg_1s1", 0.0),
    "tcc1_mdpg_p05": ("mdpg_1s1", 0.5),
    "tcc1_mdpg_p10": ("mdpg_1s1", 1.0),
}


def build_tcc1_condition_scene(condition: str, output: str | Path, *, mode: str = "dynamics",
                               config_path: str | Path | None = None, rack: bool = False,
                               payload_spec: str | Path | None = None) -> Path:
    """Generate one opt-in condition using existing gripper and payload models."""
    gripper, mass = TCC1_CONDITIONS[condition]
    load_configuration(config_path, gripper, mode, rack=rack)
    if mass and payload_spec is None:
        raise ValueError(f"{condition} requires the existing payload specification")
    payload = load_payload_spec(payload_spec, gripper, mass) if mass else None
    output = Path(output)
    if gripper == "ag145":
        source = build_ag145_scene(output, grasp_angle_rad=AG145_BOX_GRASP_ANGLE_RAD if mass else None)
    elif gripper == "mdpg_1s1":
        from mdpg_1s1_description.model import build_mdpg_scene
        source = build_mdpg_scene(output, (0, 0, 0), (0, 0, 0), dynamics_model="box_approx")
    else:
        source = source_paths()[0]
    if payload:
        attach_payload(source, payload)
    return write_scene(source, output, gripper, tool_changer_type="tcc1", mode=mode,
                       rack=rack, config_path=config_path)
