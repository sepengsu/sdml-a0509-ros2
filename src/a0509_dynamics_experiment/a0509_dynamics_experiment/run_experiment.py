"""CLI entry point. Importing this module never connects to a robot."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

from .dataset_exporter import create_trial, save_metadata, save_trajectory
from .trajectory_generator import TrajectoryConfig, generate
from .trajectory_visualizer import save_trajectory_plot


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="A0509 dynamics trajectory preview and recording")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--preview", action="store_true", help="generate plots without ROS or robot connection")
    mode.add_argument("--simulation", action="store_true", help="run in MuJoCo on an isolated ROS domain")
    parser.add_argument("--execute", action="store_true", help="request real motion; denied if any safety check is missing")
    parser.add_argument("--trajectory", choices=["T1", "T2", "T3", "T4", "all"], required=True)
    parser.add_argument("--reference-tcp-mm", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--start-mm", type=float, nargs=3, metavar=("X", "Y", "Z"), default=(0, 0, 0))
    parser.add_argument("--end-mm", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--joint-start-deg", type=float, nargs=6, metavar=("J1", "J2", "J3", "J4", "J5", "J6"), default=(0,) * 6)
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
    parser.add_argument("--gripper-type", default="none")
    parser.add_argument("--payload-mass", type=float, default=0.0)
    parser.add_argument("--sim-domain-id", type=int, default=173)
    parser.add_argument("--connect-existing", action="store_true", help="connect to existing MuJoCo in the isolated domain")
    return parser


def _validate(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.preview and args.execute:
        parser.error("--preview and --execute cannot be combined")
    if args.trajectory == "all" and not (args.preview or args.simulation):
        parser.error("--trajectory all is only supported in preview or simulation")
    if args.connect_existing and not args.simulation:
        parser.error("--connect-existing requires --simulation")
    if args.payload_mass < 0 or not math.isfinite(args.payload_mass):
        parser.error("--payload-mass must be finite and nonnegative")
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
        joint_start_rad=tuple(math.radians(x) for x in args.joint_start_deg),
        joint_indices=tuple(i - 1 for i in args.joints), joint_mode=args.joint_mode,
        joint_amplitude_deg=args.amplitude_deg, speed=args.speed,
        segment_seconds=args.segment_seconds, repeats=args.repeats,
        pre_hold_seconds=args.pre_hold, post_hold_seconds=args.post_hold, sample_hz=args.sample_hz,
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    _validate(parser, args)
    if args.simulation and args.execute:
        print("--simulation takes priority; --execute has no real-robot effect.", file=sys.stderr)
    ids = ("T1", "T2", "T3", "T4") if args.trajectory == "all" else (args.trajectory,)
    try:
        for trajectory_id in ids:
            config = _config(args, trajectory_id)
            config.validate()
            if args.preview:
                plan = generate(config)
                folder, trial_id = create_trial(args.results_dir, "preview", trajectory_id)
                save_trajectory(plan, folder)
                save_trajectory_plot(plan, folder)
                save_metadata(folder, plan, "preview", trial_id, {
                    "reference_frame": "base_link",
                    "preview_note": "Coordinates are relative to the specified TCP reference; default is base origin and not a measured robot pose.",
                    "movement_commands_sent": 0,
                })
            elif args.simulation:
                from .simulation_executor import execute
                folder = execute(config, args.results_dir, args.gripper_type, args.payload_mass,
                                 args.sim_domain_id, args.connect_existing, args.connection_timeout,
                                 reference_explicit=args.reference_tcp_mm is not None)
            else:
                from .real_executor import execute
                folder, blocked = execute(config, args.results_dir, args.gripper_type, args.payload_mass,
                                          args.duration, args.execute, args.connection_timeout,
                                          reference_explicit=args.reference_tcp_mm is not None)
                if blocked:
                    print(f"Real movement blocked after preflight; read-only data saved: {folder}", file=sys.stderr)
                    return 3
            print(f"Saved {trajectory_id}: {folder}")
        return 0
    except (ValueError, RuntimeError, OSError, ImportError) as error:
        print(f"Experiment failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
