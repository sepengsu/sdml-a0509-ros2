#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
from pathlib import Path

DEFAULT_SENTINELS = {"", "default"}


def add_optional_arg(cmd: list[str], flag: str, value: str) -> None:
    if value not in DEFAULT_SENTINELS:
        cmd.extend([flag, value])


def build_command(args: argparse.Namespace) -> list[str]:
    cmd: list[str] = [sys.executable, "-m", "dsr2.cli", args.mode]

    if args.build_mode == "no-build":
        cmd.append("--no-build")
    if args.detach == "true":
        cmd.append("-d")

    if args.mode == "virtual":
        add_optional_arg(cmd, "--gripper-port", args.gripper_option.strip())

    if args.mode == "real":
        add_optional_arg(cmd, "--ip", args.robot_ip)

        if args.connection_mode == "external":
            cmd.extend(["--external", "true"])
        elif args.connection_mode == "direct":
            cmd.extend(["--external", "false"])

        gripper_value = args.gripper_option.strip()
        if gripper_value == "no-server":
            cmd.extend(["--with-gripper", "false"])
        else:
            add_optional_arg(cmd, "--gripper-port", gripper_value)

    if args.mode == "gripper":
        add_optional_arg(cmd, "--gripper-port", args.gripper_option.strip())

    return cmd


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compose VS Code task inputs into a dsr2 CLI invocation.",
    )
    parser.add_argument("mode", choices=["virtual", "real", "gripper"])
    parser.add_argument("--build-mode", choices=["default", "no-build"], default="default")
    parser.add_argument("--detach", choices=["false", "true"], default="false")
    parser.add_argument("--robot-ip", default="default")
    parser.add_argument("--connection-mode", choices=["default", "external", "direct"], default="default")
    parser.add_argument("--gripper-option", default="default")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    cmd = build_command(args)

    if args.dry_run:
        print(" ".join(shlex.quote(part) for part in cmd))
        return 0

    project_root = Path(__file__).resolve().parent.parent
    completed = subprocess.run(cmd, cwd=project_root)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
