"""Tool-aware lower-obstacle preflight for the four forward research cycles.

The reported height is a model bound in DR_BASE Z, not a real robot clearance
certificate. The mDPG mount and both tools' physical adapters need measurement.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from .lr_trajectory import _scene_path
from .trajectory import load_trajectory


TRAJECTORIES = ("setup_10cm", "f1_slow_forward", "f2_medium_forward",
                "f3_front_postures", "f4_wrist_and_holds")
DEFAULT_MARGIN_MM = 30.0  # Design allowance, not an A0509 certified distance.
MAX_LATERAL_SPAN_MM = 400.0


def clearance_decision(lowest_z_mm: float, table_top_z_mm: float | None,
                       margin_mm: float) -> dict:
    if not math.isfinite(lowest_z_mm) or not math.isfinite(margin_mm) or margin_mm < 0:
        raise ValueError("lowest Z and nonnegative margin must be finite")
    threshold = lowest_z_mm - margin_mm
    result = {"minimum_model_z_mm": round(lowest_z_mm, 2),
              "planning_margin_mm": margin_mm,
              "highest_horizontal_obstacle_z_for_margin_mm": round(threshold, 2),
              "lower_obstacle_top_z_mm": table_top_z_mm,
              "model_clearance_passed": None}
    if table_top_z_mm is not None:
        if not math.isfinite(table_top_z_mm):
            raise ValueError("table top Z must be finite")
        result["model_clearance_passed"] = bool(lowest_z_mm - table_top_z_mm >= margin_mm)
        result["model_clearance_mm"] = round(lowest_z_mm - table_top_z_mm, 2)
    return result


def _add_mdpg_collision_envelope(scene: Path) -> None:
    """Oversize 80x80x100 mm nominal box, centered 45 mm from the mount."""
    tree = ET.parse(scene)
    tool = tree.getroot().find(".//body[@name='mdpg_base_link']")
    if tool is None:
        raise ValueError("mDPG scene has no fixed tool body")
    ET.SubElement(tool, "geom", name="mdpg_planning_envelope_collision",
                  type="box", pos="0 0 0.045", size="0.04 0.04 0.05",
                  density="0", contype="1", conaffinity="1", rgba="0 0 0 0")
    ET.indent(tree)
    tree.write(scene, encoding="utf-8", xml_declaration=True)


def _build_scenes(scene_dir: Path, mdpg_mount_xyz_m, mdpg_mount_rpy_rad,
                  arm_scene: Path | None = None) -> dict[str, Path]:
    from a0509_dynamics_experiment.ag145_model import build_ag145_scene
    from mdpg_1s1_description.model import build_mdpg_scene

    arm = Path(arm_scene).resolve() if arm_scene is not None else _scene_path()
    scene_dir.mkdir(parents=True, exist_ok=True)
    ag145 = build_ag145_scene(scene_dir / "ag145_scene.xml", arm_scene=arm)
    mdpg = build_mdpg_scene(scene_dir / "mdpg_scene.xml",
                            mount_xyz=mdpg_mount_xyz_m, mount_rpy=mdpg_mount_rpy_rad,
                            arm_scene=arm,
                            dynamics_model="box_approx")
    _add_mdpg_collision_envelope(mdpg)
    return {"ag145": ag145, "mdpg_1s1": mdpg}


def _tool_geometry_ids(model, prefix: str) -> tuple[list[int], list[int]]:
    import mujoco

    collision, visual = [], []
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or ""
        if name.startswith(prefix):
            (visual if name.endswith("_visual") else collision).append(geom_id)
    if not collision or not visual:
        raise ValueError(f"{prefix} collision envelope or CAD visuals missing")
    return collision, visual


def _geom_axis_bounds_mm(data, model, geom_id: int, axis: int) -> tuple[float, float]:
    # Boxes and mesh bounding boxes use their rotated half extents. This is
    # conservative for the mesh surface and exact for the planning boxes.
    import mujoco
    import numpy as np

    if model.geom_type[geom_id] not in (mujoco.mjtGeom.mjGEOM_BOX,
                                        mujoco.mjtGeom.mjGEOM_MESH):
        center = float(1000 * data.geom_xpos[geom_id, axis])
        radius = float(1000 * model.geom_rbound[geom_id])
        return center - radius, center + radius
    rotation_axis = data.geom_xmat[geom_id].reshape(3, 3)[axis]
    center = data.geom_xpos[geom_id, axis]
    radius = np.abs(rotation_axis).dot(model.geom_size[geom_id])
    return float(1000 * (center - radius)), float(1000 * (center + radius))


def _moving_robot_geoms(model) -> set[int]:
    import mujoco

    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_1")
    if root < 0:
        raise ValueError("A0509 scene has no link_1")
    result = set()
    for geom_id in range(model.ngeom):
        body = int(model.geom_bodyid[geom_id])
        while body and body != root:
            body = int(model.body_parentid[body])
        if body == root:
            result.add(geom_id)
    return result


def _moving_robot_contact_count(data, moving_geoms: set[int]) -> int:
    return sum(int(contact.geom1 in moving_geoms or contact.geom2 in moving_geoms)
               for contact in data.contact[:data.ncon])


def audit_front_grippers(trajectory_dir: Path, scene_dir: Path, *,
                         lower_obstacle_top_z_mm: float | None = None,
                         planning_margin_mm: float = DEFAULT_MARGIN_MM,
                         corridor_y_min_mm: float | None = None,
                         corridor_y_max_mm: float | None = None,
                         mdpg_mount_xyz_m=(0.0, 0.0, 0.0),
                         mdpg_mount_rpy_rad=(0.0, 0.0, 0.0),
                         arm_scene: Path | None = None) -> dict:
    from a0509_dynamics_experiment.simulation_executor import MujocoObserver
    from .lr_trajectory import simulate_cycle

    if planning_margin_mm < 0 or not math.isfinite(planning_margin_mm):
        raise ValueError("planning margin must be finite and nonnegative")
    if lower_obstacle_top_z_mm is not None and not math.isfinite(lower_obstacle_top_z_mm):
        raise ValueError("lower obstacle top Z must be finite")
    if (corridor_y_min_mm is None) != (corridor_y_max_mm is None):
        raise ValueError("provide both lateral corridor boundaries")
    if corridor_y_min_mm is not None and (not all(math.isfinite(v) for v in
                                                   (corridor_y_min_mm, corridor_y_max_mm)) or
                                          not 0 < corridor_y_max_mm - corridor_y_min_mm <= MAX_LATERAL_SPAN_MM):
        raise ValueError("lateral corridor must have a finite width at most 400 mm")
    trajectories = {name: load_trajectory(trajectory_dir / f"a0509_front_{name}.json")
                    for name in TRAJECTORIES}
    scenes = _build_scenes(scene_dir, mdpg_mount_xyz_m, mdpg_mount_rpy_rad,
                           arm_scene=arm_scene)
    report = {
        "status": "pending_model_preflight",
        "coordinate_frame": "DR_BASE Z; horizontal lower obstacle assumed across entire path",
        "trajectory_dir": str(trajectory_dir.resolve()),
        "arm_scene": str(Path(arm_scene).resolve() if arm_scene is not None else _scene_path()),
        "model_sources": {"ag145": "DH AG-105-145 CAD + three approximate collision boxes",
                          "mdpg_1s1": "mDPG CAD + oversize 80x80x100 mm planning box"},
        "robot_extent_method": "Rotated MuJoCo geom bounding boxes for all moving arm and tool geoms",
        "manufacturer_nominal_size_mm": {"ag145": [203.9, 212.3, 67.0],
                                         "mdpg_1s1_with_pad": [65.0, 70.0, 81.0]},
        "mdpg_mount_xyz_m": list(mdpg_mount_xyz_m),
        "mdpg_mount_rpy_rad": list(mdpg_mount_rpy_rad),
        "mdpg_mount_measured": False,
        "lower_obstacle_geometry_measured": False,
        "planning_margin_mm": planning_margin_mm,
        "maximum_allowed_lateral_span_mm": MAX_LATERAL_SPAN_MM,
        "measured_lateral_corridor_y_mm": ([corridor_y_min_mm, corridor_y_max_mm]
                                           if corridor_y_min_mm is not None else None),
        "real_robot_executable": False,
        "tools": {},
    }
    for tool, scene in scenes.items():
        observer = MujocoObserver(scene, tcp_site=("ag145_tcp" if tool == "ag145" else "mdpg_tcp"))
        moving_geoms = _moving_robot_geoms(observer.model)
        blocking_contacts = lambda data: _moving_robot_contact_count(data, moving_geoms)
        collision_ids, visual_ids = _tool_geometry_ids(observer.model, "ag145_" if tool == "ag145" else "mdpg_")
        tool_report = {"scene": str(scene.resolve()), "paths": {},
                       "collision_model": "approximate", "physical_mount_verified": False,
                       "real_robot_executable": False}
        for name, trajectory in trajectories.items():
            box_min = visual_min = float("inf")
            box_time = visual_time = 0.0
            lateral_min, lateral_max = float("inf"), -float("inf")
            robot_y_min, robot_y_max = float("inf"), -float("inf")
            contact_steps = background_contact_samples = 0
            for point in trajectory.points:
                observer._set_state(point.q_rad)
                if blocking_contacts(observer.data):
                    contact_steps += 1
                elif observer.data.ncon:
                    background_contact_samples += 1
                for geom_id in moving_geoms:
                    y_low, y_high = _geom_axis_bounds_mm(observer.data, observer.model,
                                                          geom_id, 1)
                    robot_y_min = min(robot_y_min, y_low)
                    robot_y_max = max(robot_y_max, y_high)
                for geom_id in collision_ids:
                    z, _ = _geom_axis_bounds_mm(observer.data, observer.model, geom_id, 2)
                    y_low, y_high = _geom_axis_bounds_mm(observer.data, observer.model, geom_id, 1)
                    lateral_min = min(lateral_min, y_low)
                    lateral_max = max(lateral_max, y_high)
                    if z < box_min:
                        box_min, box_time = z, point.time_s
                for geom_id in visual_ids:
                    # A bounding sphere contains each CAD mesh at every angle.
                    z = 1000 * (observer.data.geom_xpos[geom_id, 2] -
                                observer.model.geom_rbound[geom_id])
                    y = 1000 * observer.data.geom_xpos[geom_id, 1]
                    radius = 1000 * observer.model.geom_rbound[geom_id]
                    lateral_min = min(lateral_min, y - radius)
                    lateral_max = max(lateral_max, y + radius)
                    if z < visual_min:
                        visual_min, visual_time = float(z), point.time_s
                robot_y_min = min(robot_y_min, lateral_min)
                robot_y_max = max(robot_y_max, lateral_max)
            lower = min(box_min, visual_min)
            decision = clearance_decision(lower, lower_obstacle_top_z_mm,
                                          planning_margin_mm)
            simulation = simulate_cycle(observer, trajectory.points,
                                        contact_is_blocking=blocking_contacts)
            corridor_passed = (None if corridor_y_min_mm is None else
                               bool(robot_y_min >= corridor_y_min_mm and
                                    robot_y_max <= corridor_y_max_mm))
            decision.update({"sample_count": len(trajectory.points),
                             "collision_box_min_z_mm": round(box_min, 2),
                             "collision_box_min_time_s": box_time,
                             "cad_bounding_sphere_min_z_mm": round(visual_min, 2),
                             "cad_bounding_sphere_min_time_s": visual_time,
                             "tool_lateral_y_min_mm": round(lateral_min, 2),
                             "tool_lateral_y_max_mm": round(lateral_max, 2),
                             "tool_lateral_span_mm": round(lateral_max - lateral_min, 2),
                             "tool_lateral_span_passed": bool(lateral_max - lateral_min <=
                                                               MAX_LATERAL_SPAN_MM),
                             "whole_robot_lateral_y_min_mm": round(robot_y_min, 2),
                             "whole_robot_lateral_y_max_mm": round(robot_y_max, 2),
                             "whole_robot_lateral_span_mm": round(robot_y_max - robot_y_min, 2),
                             "whole_robot_lateral_span_passed": bool(robot_y_max - robot_y_min <=
                                                                      MAX_LATERAL_SPAN_MM),
                             "measured_corridor_passed": corridor_passed,
                             "model_contact_steps_without_lower_obstacle": contact_steps,
                             "model_background_contact_samples": background_contact_samples,
                             "tool_mass_simulation_preflight": simulation,
                             "model_geometry_clearance_ok": bool(contact_steps == 0 and
                                                                simulation["passed"] is True and
                                                                decision["model_clearance_passed"] is True and
                                                                corridor_passed is True),
                             "real_robot_executable": False})
            tool_report["paths"][name] = decision
        campaign_min = min(p["whole_robot_lateral_y_min_mm"] for p in tool_report["paths"].values())
        campaign_max = max(p["whole_robot_lateral_y_max_mm"] for p in tool_report["paths"].values())
        tool_report["whole_campaign_lateral_y_mm"] = [campaign_min, campaign_max]
        tool_report["whole_campaign_lateral_span_mm"] = round(campaign_max - campaign_min, 2)
        tool_report["whole_campaign_lateral_span_passed"] = bool(campaign_max - campaign_min <=
                                                                  MAX_LATERAL_SPAN_MM)
        tool_report["whole_campaign_measured_corridor_passed"] = (
            None if corridor_y_min_mm is None else
            bool(campaign_min >= corridor_y_min_mm and campaign_max <= corridor_y_max_mm))
        report["tools"][tool] = tool_report
    all_model_checks_passed = all(
        all(path["tool_mass_simulation_preflight"]["passed"] and
            path["model_contact_steps_without_lower_obstacle"] == 0
            for path in item["paths"].values())
        for item in report["tools"].values())
    report["status"] = ("model_preflight_passed_physical_unverified" if all_model_checks_passed
                        else "model_preflight_failed")
    return report


def _default_trajectory_dir() -> Path:
    local = Path(__file__).resolve().parents[1] / "examples/front"
    if local.is_dir():
        return local
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("sim_real_gap_getting")) / "examples/front"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Audit lower gripper clearance for front paths")
    parser.add_argument("--trajectory-dir", type=Path,
                        default=_default_trajectory_dir())
    parser.add_argument("--scene-dir", type=Path,
                        default=Path.cwd() / "results/front_gripper_preflight")
    parser.add_argument("--lower-obstacle-top-z-mm", type=float)
    parser.add_argument("--planning-margin-mm", type=float, default=DEFAULT_MARGIN_MM)
    parser.add_argument("--corridor-y-min-mm", type=float)
    parser.add_argument("--corridor-y-max-mm", type=float)
    parser.add_argument("--mdpg-mount-xyz-m", type=float, nargs=3, default=(0.0, 0.0, 0.0))
    parser.add_argument("--mdpg-mount-rpy-rad", type=float, nargs=3, default=(0.0, 0.0, 0.0))
    parser.add_argument("--arm-scene", type=Path,
                        help="Alternative A0509 MuJoCo scene for model comparison")
    parser.add_argument("--report-file", type=Path,
                        help="Where to write the audit JSON; defaults to the trajectory folder")
    args = parser.parse_args(argv)
    report = audit_front_grippers(args.trajectory_dir, args.scene_dir,
                                  lower_obstacle_top_z_mm=args.lower_obstacle_top_z_mm,
                                  planning_margin_mm=args.planning_margin_mm,
                                  corridor_y_min_mm=args.corridor_y_min_mm,
                                  corridor_y_max_mm=args.corridor_y_max_mm,
                                  mdpg_mount_xyz_m=args.mdpg_mount_xyz_m,
                                  mdpg_mount_rpy_rad=args.mdpg_mount_rpy_rad,
                                  arm_scene=args.arm_scene)
    path = args.report_file or args.trajectory_dir / "a0509_front_gripper_clearance.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({tool: {name: data["minimum_model_z_mm"]
                             for name, data in item["paths"].items()}
                      for tool, item in report["tools"].items()}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
