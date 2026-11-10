"""CLI entry point. Importing this module never connects to a robot."""

from __future__ import annotations

import argparse
from dataclasses import replace
import math
from pathlib import Path
import sys

from .dataset_exporter import create_trial, save_metadata, save_trajectory
from .trajectory_generator import (DYNAMICS10_IDS, EXCITATION_IDS, TRAJECTORY_IDS, TrajectoryConfig,
                                   excitation_metadata, generate, is_joint_trajectory)
from .trajectory_visualizer import save_trajectory_plot


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="A0509 dynamics trajectory preview and recording")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--preview", action="store_true", help="generate plots without ROS or robot connection")
    mode.add_argument("--simulation", action="store_true", help="run in MuJoCo on an isolated ROS domain")
    parser.add_argument("--execute", action="store_true", help="request real motion; denied if any safety check is missing")
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--trajectory", choices=[*TRAJECTORY_IDS, "all"])
    selection.add_argument("--trajectory-set", choices=["dynamics10"])
    parser.add_argument("--reference-tcp-mm", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--start-mm", type=float, nargs=3, metavar=("X", "Y", "Z"), default=(0, 0, 0))
    parser.add_argument("--end-mm", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--joint-start-deg", type=float, nargs=6, metavar=("J1", "J2", "J3", "J4", "J5", "J6"))
    parser.add_argument("--joints", type=int, nargs="+", choices=range(1, 7), default=[1, 2, 3, 4, 5, 6])
    parser.add_argument("--joint-mode", choices=["sequential", "simultaneous"], default="sequential")
    parser.add_argument("--amplitude-deg", type=float, default=5.0)
    parser.add_argument("--speed", choices=["low", "medium"], default="low")
    parser.add_argument("--segment-seconds", type=float)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--pre-hold", type=float, default=1.0)
    parser.add_argument("--post-hold", type=float, default=1.0)
    parser.add_argument("--sample-hz", type=int, default=100)
    parser.add_argument("--duration", type=float, default=10.0, help="real read-only recording duration, seconds")
    parser.add_argument("--connection-timeout", type=float, default=10.0)
    parser.add_argument("--results-dir", default="results")
    parser.add_argument("--gripper-type", choices=["none", "ag145", "mdpg_1s1"], default="none")
    parser.add_argument("--mdpg-mount-xyz", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        help="measured link_6 to mDPG origin in metres")
    parser.add_argument("--mdpg-mount-rpy", type=float, nargs=3, metavar=("R", "P", "Y"),
                        help="measured link_6 to mDPG orientation in radians")
    parser.add_argument("--mdpg-dynamics-model", choices=["visual", "box_approx"], default="visual",
                        help="explicit uniform-box inertia estimate for mDPG simulation only")
    parser.add_argument("--payload-mass", type=float, default=None,
                        help="attached payload mass in kg; positive simulation mass requires a full inertial spec")
    parser.add_argument("--payload-spec-file", type=str,
                        help="JSON with declared payload mount, COM and inertia for simulation")
    parser.add_argument("--sim-domain-id", type=int, default=173)
    parser.add_argument("--connect-existing", action="store_true", help="connect to existing MuJoCo in the isolated domain")
    parser.add_argument("--record-video", action="store_true", help="save an MP4 of the MuJoCo display")
    parser.add_argument("--defer-summary", action="store_true",
                        help="skip per-trial T5-T14 audit; run the audit once after a collection shard")
    return parser


def _validate(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    new_joint = args.trajectory_set == "dynamics10" or args.trajectory in EXCITATION_IDS
    if new_joint and not (args.preview or args.simulation):
        parser.error("T5-T16 are simulation/preview only; real mode is unavailable")
    if new_joint and (args.reference_tcp_mm or args.end_mm or args.start_mm != (0, 0, 0)):
        parser.error("T5-T16 are joint-space paths; Cartesian options are unsupported")
    if args.gripper_type == "mdpg_1s1":
        if args.mdpg_mount_xyz is None or args.mdpg_mount_rpy is None:
            parser.error("mDPG mount is TODO; provide measured --mdpg-mount-xyz and --mdpg-mount-rpy")
        if not all(math.isfinite(value) for value in (*args.mdpg_mount_xyz, *args.mdpg_mount_rpy)):
            parser.error("mDPG mount values must be finite")
        if args.simulation and args.mdpg_dynamics_model != "box_approx":
            parser.error("mDPG dynamics simulation requires --mdpg-dynamics-model box_approx (unvalidated estimate)")
        if args.execute:
            parser.error("mDPG real motion is blocked until mount, TCP and pneumatic IO are validated")
        if not (args.preview or args.simulation) and args.mdpg_dynamics_model != "visual":
            parser.error("mDPG box_approx is simulation/preview only")
    elif args.mdpg_dynamics_model != "visual":
        parser.error("--mdpg-dynamics-model requires --gripper-type mdpg_1s1")
    if args.preview and args.execute:
        parser.error("--preview and --execute cannot be combined")
    if args.trajectory == "all" and not (args.preview or args.simulation):
        parser.error("--trajectory all is only supported in preview or simulation")
    if args.connect_existing and not args.simulation:
        parser.error("--connect-existing requires --simulation")
    if args.record_video and not args.simulation:
        parser.error("--record-video requires --simulation")
    if args.defer_summary and not args.simulation:
        parser.error("--defer-summary requires --simulation")
    if args.payload_mass is not None and (args.payload_mass < 0 or not math.isfinite(args.payload_mass)):
        parser.error("--payload-mass must be finite and nonnegative")
    if args.payload_mass and args.gripper_type == "none":
        parser.error("bare A0509 does not have a payload mount; use payload mass zero")
    if args.payload_mass and args.preview:
        parser.error("positive payload mass is currently supported only in simulation")
    if args.payload_mass and args.simulation:
        if not args.payload_spec_file:
            parser.error("positive simulation payload requires --payload-spec-file")
        from .payload_model import load_payload_spec
        try:
            load_payload_spec(args.payload_spec_file, args.gripper_type, args.payload_mass)
        except (ValueError, OSError, KeyError) as error:
            parser.error(str(error))
    elif args.payload_spec_file:
        parser.error("--payload-spec-file requires simulation with positive payload mass")
    if args.duration <= 0 or args.connection_timeout <= 0:
        parser.error("duration and connection timeout must be positive")
    if args.end_mm and args.trajectory == "T4":
        parser.error("--end-mm cannot be used for joint excitation")
    if args.reference_tcp_mm and args.trajectory == "T4":
        parser.error("--reference-tcp-mm cannot be used for joint excitation")
    if args.preview and args.simulation:
        parser.error("--preview and --simulation cannot be combined")


def _config(args: argparse.Namespace, trajectory_id: str) -> TrajectoryConfig:
    return TrajectoryConfig(
        trajectory_id=trajectory_id,
        reference_tcp_mm=tuple(args.reference_tcp_mm or (0.0, 0.0, 0.0)),
        start_mm=tuple(args.start_mm), end_mm=tuple(args.end_mm) if args.end_mm else None,
        joint_start_rad=tuple(math.radians(x) for x in (args.joint_start_deg or (0,) * 6)),
        joint_indices=tuple(i - 1 for i in args.joints), joint_mode=args.joint_mode,
        joint_amplitude_deg=args.amplitude_deg, speed=args.speed,
        segment_seconds=args.segment_seconds, repeats=args.repeats,
        pre_hold_seconds=args.pre_hold, post_hold_seconds=args.post_hold, sample_hz=args.sample_hz,
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _validate(parser, args)
    payload_mass = args.payload_mass if args.payload_mass is not None else 0.0
    if args.simulation and args.execute:
        print("--simulation takes priority; --execute has no real-robot effect.", file=sys.stderr)
    ids = (DYNAMICS10_IDS if args.trajectory_set == "dynamics10" else
           ("T1", "T2", "T3", "T4") if args.trajectory == "all" else (args.trajectory,))
    batch = args.trajectory_set == "dynamics10"
    outcomes = {}
    for trajectory_id in ids:
        try:
            config = _config(args, trajectory_id)
            config.validate()
            if args.preview:
                folder, trial_id = create_trial(args.results_dir, "preview", trajectory_id)
                plot_options = {}
                preview_details = {
                    "reference_frame": "base_link",
                    "preview_note": "Coordinates are relative to the specified TCP reference; default is base origin and not a measured robot pose.",
                    "movement_commands_sent": 0,
                    "gripper_type": args.gripper_type,
                }
                if args.gripper_type in ("ag145", "mdpg_1s1") or trajectory_id in EXCITATION_IDS:
                    from .simulation_executor import MujocoObserver
                    if args.gripper_type == "ag145":
                        from .ag145_model import AG145_MASS_KG, AG145_TCP_OFFSET_MM, build_ag145_scene
                        scene = build_ag145_scene(folder / "ag145_scene.xml")
                        tcp_site = "ag145_tcp"
                    elif args.gripper_type == "mdpg_1s1":
                        from mdpg_1s1_description.model import build_mdpg_scene
                        scene = build_mdpg_scene(folder / "mdpg_scene.xml", args.mdpg_mount_xyz,
                                                 args.mdpg_mount_rpy,
                                                 dynamics_model=args.mdpg_dynamics_model)
                        tcp_site = "mdpg_tcp"
                    else:
                        from .ag145_model import source_paths
                        scene = source_paths()[0]
                        tcp_site = "tool0"
                    observer = MujocoObserver(scene, tcp_site=tcp_site)
                    reference_q = (config.joint_start_rad if is_joint_trajectory(trajectory_id)
                                   else (-0.5, -0.5, 1.6, 0.0, -0.6, 0.0))
                    if not is_joint_trajectory(trajectory_id) and args.reference_tcp_mm is None:
                        config = replace(config, reference_tcp_mm=observer.tcp_mm(reference_q))
                    plan = generate(config)
                    plot_options["robot_links_mm"] = observer.links_mm(reference_q)
                    if is_joint_trajectory(trajectory_id):
                        plot_options["planned_xyz_mm"] = [observer.tcp_mm(point.q_rad)
                                                            for point in plan.points]
                    if trajectory_id in EXCITATION_IDS:
                        from .dynamics10_safety import validate_joint_plan
                        preview_details["preflight"] = validate_joint_plan(plan, observer)
                        preview_details["excitation"] = excitation_metadata(trajectory_id)
                        if trajectory_id in DYNAMICS10_IDS:
                            preview_details["dynamics10"] = preview_details["excitation"]
                        preview_details["trajectory_kind"] = "joint"
                        preview_details["execution_success"] = True
                        preview_details["sample_count"] = 0
                    preview_details["sim_model"] = scene.name
                    if args.gripper_type == "ag145":
                        preview_details["gripper_model"] = {
                            "configuration": "fixed open pose", "mass_kg": AG145_MASS_KG,
                            "inertia": "uncalibrated box approximation",
                            "collision": "approximate bounding boxes",
                            "tcp_site": tcp_site, "tcp_offset_mm": list(AG145_TCP_OFFSET_MM),
                            "tcp_calibrated": False,
                        }
                    elif args.gripper_type == "mdpg_1s1":
                        from mdpg_1s1_description.model import SPEC_MASS_KG, SPEC_MAX_PAYLOAD_KG
                        preview_details["preview_note"] = ("mDPG CAD visual and optional box mass estimate; "
                                                           "nominal TCP and supplied mount are uncalibrated; no ROS connection.")
                        preview_details["gripper_model"] = {
                            "mass_kg": SPEC_MASS_KG, "mass_source": "user-supplied product specification image",
                            "max_payload_rating_kg": SPEC_MAX_PAYLOAD_KG,
                            "dynamics_model": args.mdpg_dynamics_model,
                            "modeled_mass_kg": (SPEC_MASS_KG if args.mdpg_dynamics_model == "box_approx" else 0.0),
                            "inertia": ("uniform-box estimate" if args.mdpg_dynamics_model == "box_approx" else None),
                            "pneumatic_io": None,
                            "collision": None, "tcp_site": tcp_site, "tcp_calibrated": False,
                            "mount_xyz_m": list(args.mdpg_mount_xyz),
                            "mount_rpy_rad": list(args.mdpg_mount_rpy),
                            "mount_verified": False,
                        }
                else:
                    plan = generate(config)
                save_trajectory(plan, folder)
                save_trajectory_plot(plan, folder, **plot_options)
                save_metadata(folder, plan, "preview", trial_id, preview_details)
            elif args.simulation:
                from .simulation_executor import execute
                folder = execute(config, args.results_dir, args.gripper_type, payload_mass,
                                 args.sim_domain_id, args.connect_existing, args.connection_timeout,
                                 reference_explicit=args.reference_tcp_mm is not None,
                                 record_video=args.record_video,
                                 mdpg_mount_xyz=args.mdpg_mount_xyz,
                                 mdpg_mount_rpy=args.mdpg_mount_rpy,
                                 mdpg_dynamics_model=args.mdpg_dynamics_model,
                                 payload_spec_file=args.payload_spec_file)
            else:
                from .real_executor import execute
                folder, blocked = execute(config, args.results_dir, args.gripper_type, payload_mass,
                                          args.duration, args.execute, args.connection_timeout,
                                          reference_explicit=args.reference_tcp_mm is not None)
                if blocked:
                    print(f"Real movement blocked after preflight; read-only data saved: {folder}", file=sys.stderr)
                    return 3
            print(f"Saved {trajectory_id}: {folder}")
            outcomes[trajectory_id] = {"success": True, "path": str(folder)}
        except (ValueError, RuntimeError, OSError, ImportError) as error:
            print(f"{trajectory_id} failed: {error}", file=sys.stderr)
            outcomes[trajectory_id] = {"success": False, "error": str(error)}
            if not batch:
                return 1
    if not args.defer_summary and (batch or any(i in DYNAMICS10_IDS for i in ids)):
        from .dynamics10_dataset import summarize
        summarize(args.results_dir, outcomes)
    if batch:
        print(f"Dynamics10 summary: {sum(v['success'] for v in outcomes.values())}/{len(outcomes)} succeeded")
        for trajectory_id, result in outcomes.items():
            print(f"  {trajectory_id}: {'OK' if result['success'] else 'FAILED - ' + result['error']}")
    return 0 if all(item["success"] for item in outcomes.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
