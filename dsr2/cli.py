#!/usr/bin/env python3
"""
Doosan Robot Docker Environment Starter

Usage:
    dsr2 virtual              # Start virtual mode (emulator)
    dsr2 virtual -d           # Start in background (detached)
    dsr2 real                 # Start real mode (hardware)
    dsr2 real --ip 192.168.1.100  # Override robot IP

Cross-platform: Works on Windows (PowerShell/CMD), Linux, macOS, WSL2

Configuration is loaded from .env file in project root.
CLI arguments override .env values.
"""

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import NamedTuple, Optional, Self, TypedDict

IMAGE_NAME = "sdml-doosan-robot-ros-control:latest"
MANAGED_GRIPPER_STATE_DIR = ".dsr2"
MANAGED_GRIPPER_STATE_FILE = "managed_gripper_server.json"


def get_project_root() -> Path:
    """Get the project root directory."""
    return Path(__file__).parent.parent.resolve()


class Config(NamedTuple):
    robot_ip: str
    robot_port: str
    robot_model: str
    gripper_port: str
    ros_domain_id: str

    @classmethod
    def from_dotenv(cls) -> Self:
        """Load configuration from .env file."""

        def parse_env_file(env_file: Path) -> dict[str, str]:
            """Parse .env file manually (no external dependencies)."""
            env_vars: dict[str, str] = {}
            if not env_file.exists():
                return env_vars

            with open(env_file, "r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    # Skip empty lines and comments
                    if not line or line.startswith("#"):
                        continue
                    # Parse KEY=VALUE
                    if "=" in line:
                        key, _, value = line.partition("=")
                        key = key.strip()
                        value = value.strip()
                        # Remove quotes if present
                        if (value.startswith('"') and value.endswith('"')) or (
                            value.startswith("'") and value.endswith("'")
                        ):
                            value = value[1:-1]
                        env_vars[key] = value
            return env_vars

        project_root = get_project_root()
        env_file = project_root / ".env"
        env_example = project_root / ".env.example"

        # Create .env from .env.example if not exists
        if not env_file.exists() and env_example.exists():
            print("Creating .env from .env.example...")
            shutil.copy(env_example, env_file)

        # Parse .env file
        env_vars = parse_env_file(env_file)

        # Return config with defaults (env file values override defaults)
        return cls(
            robot_ip=env_vars["ROBOT_IP"],
            robot_port=env_vars["ROBOT_PORT"],
            robot_model=env_vars["ROBOT_MODEL"],
            gripper_port=env_vars["GRIPPER_PORT"],
            ros_domain_id=env_vars["ROS_DOMAIN_ID"],
        )


class GripperState(TypedDict):
    pid: int
    gripper_port: Optional[str]
    started_at: int


def needs_rebuild(project_root: Path) -> bool:
    """Check if Docker image needs to be rebuilt.

    Returns True if:
    - Image doesn't exist
    - Any file in docker/ is newer than the image
    """
    # Check if image exists and get its creation time
    result = subprocess.run(
        ["docker", "image", "inspect", IMAGE_NAME, "--format", "{{.Created}}"],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"  Image '{IMAGE_NAME}' not found, build required.")
        return True

    # Parse image creation time (ISO format)
    from datetime import datetime

    image_time_str = result.stdout.strip()
    # Handle format: 2026-02-03T15:40:55.123456789+09:00
    image_time_str = image_time_str.split(".")[0]  # Remove nanoseconds
    image_time = datetime.fromisoformat(image_time_str)

    # Check if any docker/* file is newer than image
    docker_dir = project_root / "docker"
    for file in docker_dir.iterdir():
        if file.is_file():
            file_mtime = datetime.fromtimestamp(file.stat().st_mtime)
            # Make naive datetime for comparison (remove timezone)
            image_time_naive = image_time.replace(tzinfo=None)
            if file_mtime > image_time_naive:
                print(f"  File '{file.name}' modified after image build, rebuild required.")
                return True

    print(f"  Image '{IMAGE_NAME}' is up to date, skipping build.")
    return False


def main():
    # Load config from .env first
    config = Config.from_dotenv()

    parser = argparse.ArgumentParser(
        description="Doosan Robot Docker Environment Starter",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=f"""
Examples:
    $ dsr2 virtual                    # Start emulator mode
    $ dsr2 virtual --gripper-port mock  # Start emulator + mock gripper server
    $ dsr2 real                       # Real mode (auto-configured by OS)
    $ dsr2 real --ip 192.168.1.100    # Override robot IP
    $ dsr2 real --gripper-port COM3   # Specify gripper port (Windows/macOS)
    $ dsr2 real --gripper-port mock   # Use mock gripper
    $ dsr2 real --external false      # Force direct USB mode (Linux)
    $ dsr2 real --with-gripper false  # Disable auto-start gripper server
    $ dsr2 gripper                    # Start gripper server only
    $ dsr2 gripper --gripper-port COM3   # Gripper server on specific port
    $ dsr2 gripper --stop            # Stop managed background gripper server
    $ dsr2 virtual --no-build         # Skip Docker build
    $ dsr2 virtual -d                 # Run in background (detached)

Current config (from .env):
    ROBOT_IP:      {config.robot_ip}
    ROBOT_MODEL:   {config.robot_model}
    GRIPPER_PORT:  {config.gripper_port}
    ROS_DOMAIN_ID: {config.ros_domain_id}
        """,
    )

    parser.add_argument(
        "mode",
        choices=["virtual", "real", "gripper"],
        help="Control mode: 'virtual' for emulator, 'real' for hardware, 'gripper' to start gripper server only",
    )
    parser.add_argument(
        "--ip",
        dest="robot_ip",
        default=config.robot_ip,
        help=f"Robot IP address (default from .env: {config.robot_ip})",
    )
    parser.add_argument(
        "--no-build",
        dest="no_build",
        action="store_true",
        help="Skip Docker image build (use existing image)",
    )
    parser.add_argument(
        "-d",
        "--detach",
        dest="detach",
        action="store_true",
        help="Run containers in background (detached mode)",
    )
    # Default --external to True on Windows/macOS, False on Linux
    is_linux = sys.platform.startswith("linux")
    default_external = not is_linux
    parser.add_argument(
        "--external",
        dest="external",
        type=lambda x: x.lower() in ("true", "1", "yes"),
        default=default_external,
        metavar="BOOL",
        help=f"Use external gripper server via TCP (default: {'false' if is_linux else 'true'} on this platform)",
    )
    parser.add_argument(
        "--with-gripper",
        dest="with_gripper",
        type=lambda x: x.lower() in ("true", "1", "yes"),
        default=True,  # Will be set based on --external if not specified
        metavar="BOOL",
        help="Auto-start gripper server (default: same as --external)",
    )
    parser.add_argument(
        "--gripper-port",
        dest="gripper_port",
        default=None,
        help="Gripper serial port (e.g., COM3, /dev/ttyACM0). Use 'mock' for mock mode, 'auto' for auto-detect. In virtual mode, providing this auto-starts the gripper server.",
    )
    parser.add_argument(
        "--stop",
        dest="stop_gripper",
        action="store_true",
        help="Stop a managed background gripper server (gripper mode only)",
    )

    args = parser.parse_args()

    if args.stop_gripper and args.mode != "gripper":
        parser.error("--stop is only valid with 'gripper' mode")

    if args.stop_gripper and args.gripper_port is not None:
        parser.error("--stop cannot be combined with --gripper-port")

    project_root = get_project_root()

    if args.mode == "gripper":
        if args.stop_gripper:
            stop_managed_gripper_server(project_root)
        else:
            start_gripper_server(args.gripper_port)
        return

    # Check Docker
    print("Checking Docker...")
    if not check_docker():
        sys.exit(1)
    print("  Docker is ready!")

    # Smart build: only rebuild if image doesn't exist or docker/* files changed
    if args.no_build:
        build = False
        print("  Build skipped (--no-build)")
    else:
        build = needs_rebuild(project_root)

    if args.mode == "virtual":
        start_virtual(project_root, config, build=build, detach=args.detach, gripper_port=args.gripper_port)
    elif args.mode == "real":
        if args.external:
            start_real_external(
                project_root,
                config,
                robot_ip=args.robot_ip,
                build=build,
                detach=args.detach,
                with_gripper=args.with_gripper,
                gripper_port=args.gripper_port,
            )
        else:
            start_real(project_root, config, robot_ip=args.robot_ip, build=build, detach=args.detach)


def build_gripper_server_command(gripper_port: str | None = None) -> tuple[list[str], str]:
    """Build the gripper server command and a human-readable mode description."""
    cmd = [sys.executable, "-m", "dsr2.gripper_server"]

    if gripper_port == "mock":
        cmd.append("--mock")
        mode_label = "Mock (no hardware)"
    elif gripper_port == "auto" or gripper_port is None:
        cmd.append("--auto")
        mode_label = "Auto-detect serial port"
    else:
        cmd.extend(["--port", gripper_port])
        mode_label = f"Serial port {gripper_port}"

    return cmd, mode_label


def get_managed_gripper_state_path(project_root: Path) -> Path:
    """Return the state file path for a managed background gripper server."""
    return project_root / MANAGED_GRIPPER_STATE_DIR / MANAGED_GRIPPER_STATE_FILE


def clear_managed_gripper_server_state(project_root: Path) -> None:
    """Remove the managed gripper server state file if it exists."""
    state_path = get_managed_gripper_state_path(project_root)
    try:
        state_path.unlink()
    except FileNotFoundError:
        return


def is_process_alive(pid: int) -> bool:
    """Return True if the process is still alive."""
    if pid <= 0:
        return False

    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def wait_for_process_exit(pid: int, timeout_seconds: float) -> bool:
    """Wait for a process to exit within the timeout."""
    deadline = time.time() + timeout_seconds
    while time.time() < deadline:
        if not is_process_alive(pid):
            return True
        time.sleep(0.2)
    return not is_process_alive(pid)


def read_managed_gripper_server_state(project_root: Path) -> GripperState | None:
    """Read the managed gripper server state if it is valid and still running."""
    state_path = get_managed_gripper_state_path(project_root)
    if not state_path.exists():
        return None

    try:
        state = json.loads(state_path.read_text(encoding="utf-8"))
        state = GripperState(
            pid=int(state["pid"]),
            gripper_port=state["gripper_port"],
            started_at=int(state["started_at"]),
        )
    except (OSError, json.JSONDecodeError):
        clear_managed_gripper_server_state(project_root)
        return None

    pid = state["pid"]
    if not is_process_alive(pid):
        clear_managed_gripper_server_state(project_root)
        return None

    return state


def write_managed_gripper_server_state(project_root: Path, pid: int, gripper_port: str | None) -> None:
    """Persist the PID for a managed background gripper server."""
    state_path = get_managed_gripper_state_path(project_root)
    state_path.parent.mkdir(parents=True, exist_ok=True)
    state = GripperState(pid=pid, gripper_port=gripper_port, started_at=int(time.time()))
    state_path.write_text(json.dumps(state, indent=2), encoding="utf-8")


def stop_managed_gripper_server(project_root: Path, silent: bool = False) -> bool:
    """Stop the managed background gripper server, if any."""
    state = read_managed_gripper_server_state(project_root)
    if state is None:
        if not silent:
            print("No managed background gripper server is running.")
        return False

    pid = state["pid"]
    gripper_port = state.get("gripper_port")

    if not silent:
        mode = "auto" if gripper_port in (None, "auto") else gripper_port
        print(f"Stopping managed gripper server (PID {pid}, port={mode})...")

    try:
        if sys.platform == "win32":
            subprocess.run(
                ["taskkill", "/PID", str(pid), "/T", "/F"],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        else:
            os.kill(pid, signal.SIGTERM)

        if not wait_for_process_exit(pid, timeout_seconds=5):
            if sys.platform != "win32":
                os.kill(pid, signal.SIGKILL)
                wait_for_process_exit(pid, timeout_seconds=2)
    except OSError:
        pass
    finally:
        clear_managed_gripper_server_state(project_root)

    if not silent:
        print("Managed gripper server stopped.")
    return True


def start_background_gripper_server(
    project_root: Path,
    gripper_port: str | None = None,
) -> subprocess.Popen[bytes]:
    """Start the gripper server as a background process."""
    stop_managed_gripper_server(project_root, silent=True)

    gripper_cmd, mode_label = build_gripper_server_command(gripper_port)
    print(f"\nStarting gripper server in background... ({mode_label})")

    if sys.platform == "win32":
        gripper_process = subprocess.Popen(
            gripper_cmd,
            creationflags=subprocess.CREATE_NEW_PROCESS_GROUP,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
    else:
        gripper_process = subprocess.Popen(
            gripper_cmd,
            start_new_session=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )

    time.sleep(2)

    if gripper_process.poll() is None:
        write_managed_gripper_server_state(project_root, gripper_process.pid, gripper_port)
    else:
        clear_managed_gripper_server_state(project_root)

    return gripper_process


def stop_background_gripper_server(
    project_root: Path,
    gripper_process: subprocess.Popen[bytes] | None,
) -> None:
    """Stop a background gripper server process if it is still running."""
    if gripper_process is None:
        return

    if gripper_process.poll() is None:
        print("\nStopping gripper server...")
        gripper_process.terminate()
        try:
            gripper_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            gripper_process.kill()
            gripper_process.wait(timeout=5)

    clear_managed_gripper_server_state(project_root)


def start_virtual(
    project_root: Path,
    config: Config,
    build: bool = True,
    detach: bool = False,
    gripper_port: str | None = None,
):
    """Start virtual mode with emulator."""
    print("\n" + "=" * 50)
    print(" Starting VIRTUAL Mode")
    print("=" * 50)
    print("\nThis will start:")
    print("  - Doosan robot emulator (port 12345)")
    print(f"  - Robot model: {config.robot_model}")
    print("  - ROS 2 control stack")
    print("  - Gripper mock mode")
    if gripper_port is not None:
        _, mode_label = build_gripper_server_command(gripper_port)
        print(f"  - Host gripper server: {mode_label}")
    if detach:
        print("  - Running in background (detached)")
    print()

    setup_x11()

    os.chdir(project_root)

    # Environment is already loaded from .env by docker compose
    # We just override the mode-specific ones
    env = os.environ.copy()
    env["GRIPPER_MOCK"] = "true"
    gripper_process = None

    if gripper_port is not None:
        gripper_process = start_background_gripper_server(project_root, gripper_port)
        if gripper_process.poll() is not None:
            print("  ERROR: Gripper server process exited immediately")
            sys.exit(1)
        print("Checking gripper server connection...")
        if check_gripper_server():
            print("  Gripper server is reachable!")
        else:
            print("  ERROR: Gripper server failed to start")
            stop_background_gripper_server(project_root, gripper_process)
            sys.exit(1)

    cmd = ["docker", "compose", "--profile", "virtual", "up"]
    if build:
        cmd.append("--build")
    if detach:
        cmd.append("-d")

    print("Starting containers...")
    try:
        result = subprocess.run(cmd, env=env)
    except Exception:
        result = None
    finally:
        if not detach:
            stop_background_gripper_server(project_root, gripper_process)

    if result is not None and result.returncode != 0:
        if detach:
            stop_background_gripper_server(project_root, gripper_process)
        sys.exit(result.returncode)

    if detach:
        print("\nContainers started in background.")
        print("Use 'docker compose logs -f' to view logs.")
        print("Use 'docker compose down' to stop.")
        if gripper_process is not None:
            print("Use 'dsr2 gripper --stop' to stop the managed host gripper server.")


def start_real(project_root: Path, config: Config, robot_ip: str, build: bool = True, detach: bool = False):
    """Start real mode with hardware."""
    print("\n" + "=" * 50)
    print(" Starting REAL Mode")
    print("=" * 50)
    print()
    print("WARNING: This will connect to REAL robot hardware!")
    print("Ensure the robot is in a safe state before proceeding.")
    print()

    print(f"Robot IP: {robot_ip}")
    print(f"Robot Model: {config.robot_model}")
    print(f"Gripper Port: {config.gripper_port}")
    if detach:
        print("Running in background (detached)")

    # Test connectivity
    print("\nTesting connection to robot...")
    if ping_host(robot_ip):
        print("  Robot is reachable!")
    else:
        print(f"  ERROR: Cannot reach robot at {robot_ip}")
        print("  Please check network connection and robot power.")
        sys.exit(1)

    # Safety confirmation
    confirm = input("\nAre you sure you want to start REAL mode? (yes/no): ").strip().lower()
    if confirm != "yes":
        print("Aborted.")
        sys.exit(0)

    # Check USB devices
    gripper_port = config.gripper_port
    print(f"\nChecking for gripper serial device ({gripper_port})...")
    if check_usb_device(gripper_port):
        print(f"  Found {gripper_port}")
    else:
        print(f"  WARNING: {gripper_port} not found. Gripper may not work.")
        if sys.platform != "win32":
            print("  If using WSL2, ensure USB passthrough is configured with usbipd-win.")

    setup_x11()

    os.chdir(project_root)

    # Set environment for real mode
    env = os.environ.copy()
    env["GRIPPER_MOCK"] = "false"
    env["ROBOT_IP"] = robot_ip

    cmd = ["docker", "compose", "--profile", "real", "up"]
    if build:
        cmd.append("--build")
    if detach:
        cmd.append("-d")

    print("\nStarting containers...")
    subprocess.run(cmd, env=env)

    if detach:
        print("\nContainers started in background.")
        print("Use 'docker compose logs -f' to view logs.")
        print("Use 'docker compose down' to stop.")


def start_gripper_server(gripper_port: str | None = None):
    """Start the gripper server standalone."""
    print("\n" + "=" * 50)
    print(" Starting Gripper Server")
    print("=" * 50)
    print()

    # Build command
    cmd, mode_label = build_gripper_server_command(gripper_port)
    print(f"Mode: {mode_label}")

    print("\nStarting gripper server... (Ctrl+C to stop)")
    print()

    try:
        subprocess.run(cmd)
    except KeyboardInterrupt:
        print("\nGripper server stopped.")


def start_real_external(
    project_root: Path,
    config: Config,
    robot_ip: str,
    build: bool = True,
    detach: bool = False,
    with_gripper: bool = False,
    gripper_port: str | None = None,
):
    """Start real mode with external gripper server (Windows/macOS)."""
    print("\n" + "=" * 50)
    print(" Starting REAL Mode (External Gripper)")
    print("=" * 50)
    print()
    print("WARNING: This will connect to REAL robot hardware!")
    print("Ensure the robot is in a safe state before proceeding.")
    print()

    gripper_process = None

    if with_gripper:
        print("Gripper server will be started automatically.")
    else:
        print("NOTE: This mode uses an external gripper server.")
        print("      You must run the gripper server on the host first:")
        print()
        print("      Windows:  dsr2-gripper --port COM3")
        print("      macOS:    dsr2-gripper --port /dev/tty.usbmodem*")
        print("      Linux:    dsr2-gripper --port /dev/ttyACM0")
        print("      Mock:     dsr2-gripper --mock")
        print()
        print("  Or use --with-gripper to auto-start:")
        print("      dsr2 real --gripper-port COM3")
        print()

    print(f"Robot IP: {robot_ip}")
    print(f"Robot Model: {config.robot_model}")
    if detach:
        print("Running in background (detached)")

    # Test connectivity
    print("\nTesting connection to robot...")
    if ping_host(robot_ip):
        print("  Robot is reachable!")
    else:
        print(f"  ERROR: Cannot reach robot at {robot_ip}")
        print("  Please check network connection and robot power.")
        sys.exit(1)

    # Start gripper server if requested
    if with_gripper:
        gripper_process = start_background_gripper_server(project_root, gripper_port)
        if gripper_process.poll() is not None:
            print("  ERROR: Gripper server process exited immediately")
            sys.exit(1)

    # Check if gripper server is reachable
    print("\nChecking gripper server connection...")
    if check_gripper_server():
        print("  Gripper server is reachable!")
    else:
        if with_gripper:
            print("  ERROR: Gripper server failed to start")
            stop_background_gripper_server(project_root, gripper_process)
            sys.exit(1)
        else:
            print("  WARNING: Gripper server not detected at localhost:9999")
            print("  Make sure to start it with: dsr2-gripper --port <PORT>")

    # Safety confirmation
    confirm = input("\nAre you sure you want to start REAL mode? (yes/no): ").strip().lower()
    if confirm != "yes":
        print("Aborted.")
        stop_background_gripper_server(project_root, gripper_process)
        sys.exit(0)

    setup_x11()

    os.chdir(project_root)

    # Set environment for real-external mode
    env = os.environ.copy()
    env["GRIPPER_MOCK"] = "false"
    env["GRIPPER_EXTERNAL"] = "true"
    env["ROBOT_IP"] = robot_ip

    cmd = ["docker", "compose", "--profile", "real-external", "up"]
    if build:
        cmd.append("--build")
    if detach:
        cmd.append("-d")

    print("\nStarting containers...")
    try:
        result = subprocess.run(cmd, env=env)
    except Exception:
        result = None
    finally:
        # Clean up gripper server if we started it
        if not detach:
            stop_background_gripper_server(project_root, gripper_process)

    if result is not None and result.returncode != 0:
        if detach:
            stop_background_gripper_server(project_root, gripper_process)
        sys.exit(result.returncode)

    if detach:
        print("\nContainers started in background.")
        print("Use 'docker compose logs -f' to view logs.")
        print("Use 'docker compose down' to stop.")
        if with_gripper:
            print("Use 'dsr2 gripper --stop' to stop the managed host gripper server.")


def check_gripper_server(host: str = "localhost", port: int = 9999) -> bool:
    """Check if the gripper server is running."""
    import socket

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        result = sock.connect_ex((host, port))
        sock.close()
        return result == 0
    except Exception:
        return False


def run_command(cmd: list[str], check: bool = True, capture: bool = False) -> subprocess.CompletedProcess[str]:
    """Run a shell command."""
    print(f"  > {' '.join(cmd)}")
    return subprocess.run(cmd, check=check, capture_output=capture, text=True)


def check_docker() -> bool:
    """Check if Docker is available."""
    if not shutil.which("docker"):
        print("ERROR: Docker is not installed or not in PATH")
        return False

    result = run_command(["docker", "info"], check=False, capture=True)
    if result.returncode != 0:
        print("ERROR: Docker daemon is not running")
        return False

    return True


def setup_x11() -> None:
    """Setup X11 permissions for GUI (Linux/WSL only)."""
    if sys.platform == "win32":
        return  # Not needed on native Windows

    if shutil.which("xhost"):
        print("Setting up X11 permissions...")
        run_command(["xhost", "+local:docker"], check=False)


def ping_host(host: str) -> bool:
    """Test network connectivity to a host."""
    param = "-n" if sys.platform == "win32" else "-c"
    timeout_param = "-w" if sys.platform == "win32" else "-W"
    result = run_command(["ping", param, "1", timeout_param, "2", host], check=False, capture=True)
    return result.returncode == 0


def check_usb_device(device: str) -> bool:
    """Check if a USB device exists (Linux/WSL only)."""
    if sys.platform == "win32":
        return True  # Can't check on native Windows
    return Path(device).exists()


if __name__ == "__main__":
    main()
