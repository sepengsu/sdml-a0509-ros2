#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
from pathlib import Path


def build_docker_down_command(profiles: list[str]) -> list[str]:
    cmd = ["docker", "compose"]
    for profile in profiles:
        cmd.extend(["--profile", profile])
    cmd.append("down")
    return cmd


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Stop Docker compose profiles and clean up the managed host gripper server.",
    )
    parser.add_argument("profiles", nargs="+", help="Docker compose profiles to stop")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parent.parent
    docker_cmd = build_docker_down_command(args.profiles)
    stop_gripper_cmd = [sys.executable, "-m", "dsr2.cli", "gripper", "--stop"]

    if args.dry_run:
        print(" ".join(shlex.quote(part) for part in docker_cmd))
        print(" ".join(shlex.quote(part) for part in stop_gripper_cmd))
        return 0

    docker_result = subprocess.run(docker_cmd, cwd=project_root)
    gripper_result = subprocess.run(stop_gripper_cmd, cwd=project_root)

    if docker_result.returncode != 0:
        return docker_result.returncode
    return gripper_result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
