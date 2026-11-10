"""Attach a user-specified inertial payload to a fixed tool body in MJCF."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


CONDITION_KEYS = {
    ("ag145", 0.5): "ag145_p05",
    ("ag145", 1.0): "ag145_p10",
    ("mdpg_1s1", 0.5): "mdpg_p05",
    ("mdpg_1s1", 1.0): "mdpg_p10",
}
PARENT_BODIES = {"ag145": "ag145_base_link", "mdpg_1s1": "mdpg_base_link"}


def _triple(value, name: str) -> tuple[float, float, float]:
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} is TODO: provide three measured or declared values")
    try:
        values = tuple(float(item) for item in value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} requires three finite numbers") from error
    if not all(math.isfinite(item) for item in values):
        raise ValueError(f"{name} requires three finite numbers")
    return values


def _quat(rpy: tuple[float, float, float]) -> str:
    roll, pitch, yaw = (value / 2 for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return " ".join(f"{item:.12g}" for item in (
        cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy))


def load_payload_spec(path: str | Path, gripper_type: str, mass_kg: float) -> dict:
    """Reject missing/invalid inertia instead of silently adding a metadata-only mass."""
    key = CONDITION_KEYS.get((gripper_type, mass_kg))
    if key is None:
        raise ValueError("payload is supported only for AG145/mDPG at 0.5 or 1.0 kg")
    path = Path(path).resolve()
    raw = path.read_bytes()
    document = json.loads(raw)
    if document.get("schema_version") != 1 or not isinstance(document.get("conditions"), dict):
        raise ValueError("payload spec requires schema_version 1 and conditions")
    entry = document["conditions"].get(key)
    if not isinstance(entry, dict):
        raise ValueError(f"payload condition {key} missing")
    if entry.get("mass_kg") != mass_kg:
        raise ValueError(f"payload condition {key} mass differs from CLI {mass_kg} kg")
    source = entry.get("parameter_source")
    if source not in ("measured", "declared_simulation_assumption"):
        raise ValueError(f"{key}.parameter_source must declare measured or declared_simulation_assumption")
    source_note = entry.get("source_note")
    if not isinstance(source_note, str) or not source_note.strip():
        raise ValueError(f"{key}.source_note must describe the measurement or simulation assumption")
    mount_xyz = _triple(entry.get("mount_xyz_m"), f"{key}.mount_xyz_m")
    mount_rpy = _triple(entry.get("mount_rpy_rad"), f"{key}.mount_rpy_rad")
    com_xyz = _triple(entry.get("com_xyz_m"), f"{key}.com_xyz_m")
    inertia = entry.get("inertia_kg_m2")
    if not isinstance(inertia, list) or len(inertia) != 6:
        raise ValueError(f"{key}.inertia_kg_m2 is TODO: provide [ixx,iyy,izz,ixy,ixz,iyz]")
    try:
        ixx, iyy, izz, ixy, ixz, iyz = (float(value) for value in inertia)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{key}.inertia_kg_m2 requires six finite numbers") from error
    values = (ixx, iyy, izz, ixy, ixz, iyz)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{key}.inertia_kg_m2 requires six finite numbers")
    tensor = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]])
    eigen = np.linalg.eigvalsh(tensor)
    if np.min(eigen) <= 0 or np.max(eigen) > np.sum(eigen) - np.max(eigen) + 1e-12:
        raise ValueError(f"{key}.inertia_kg_m2 must be positive definite and physically realizable")
    return {
        "condition_id": key, "mass_kg": mass_kg, "parameter_source": source,
        "source_note": source_note.strip(),
        "mount_xyz_m": mount_xyz, "mount_rpy_rad": mount_rpy,
        "com_xyz_m": com_xyz, "inertia_kg_m2": values,
        "parent_body": PARENT_BODIES[gripper_type],
        "spec_path": str(path), "spec_sha256": hashlib.sha256(raw).hexdigest(),
    }


def attach_payload(scene_path: str | Path, spec: dict) -> Path:
    """Add a fixed inertial body; contact/visual geometry is deliberately absent."""
    scene_path = Path(scene_path).resolve()
    tree = ET.parse(scene_path)
    parent = tree.getroot().find(f".//body[@name='{spec['parent_body']}']")
    if parent is None:
        raise ValueError(f"MuJoCo scene has no {spec['parent_body']} payload parent")
    if tree.getroot().find(".//body[@name='experiment_payload']") is not None:
        raise ValueError("MuJoCo scene already contains experiment_payload")
    body = ET.SubElement(parent, "body", name="experiment_payload",
                         pos=" ".join(str(x) for x in spec["mount_xyz_m"]),
                         quat=_quat(spec["mount_rpy_rad"]))
    ET.SubElement(body, "inertial",
                  pos=" ".join(str(x) for x in spec["com_xyz_m"]),
                  mass=str(spec["mass_kg"]),
                  fullinertia=" ".join(str(x) for x in spec["inertia_kg_m2"]))
    ET.indent(tree)
    tree.write(scene_path, encoding="utf-8", xml_declaration=True)
    return scene_path


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Validate all seven-condition payload specifications")
    parser.add_argument("--check-all", required=True, type=Path,
                        help="path to payload_conditions.json")
    args = parser.parse_args(argv)
    for (gripper, mass), key in CONDITION_KEYS.items():
        try:
            spec = load_payload_spec(args.check_all, gripper, mass)
        except (ValueError, OSError, KeyError) as error:
            parser.error(str(error))
        print(f"{key}: {spec['mass_kg']} kg, {spec['parameter_source']}, {spec['spec_sha256'][:12]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
