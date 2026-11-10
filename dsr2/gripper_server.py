#!/usr/bin/env python3
"""
Cross-platform DH Gripper TCP Server

Runs on the host machine (Windows/macOS/Linux) and communicates with the
Docker container via TCP socket. Uses pymodbus for Modbus RTU communication
with the physical DH-Robotics gripper.

Usage:
    python -m dsr2.gripper_server --port COM3      # Windows
    python -m dsr2.gripper_server --port /dev/ttyACM0   # Linux
    python -m dsr2.gripper_server --port /dev/tty.usbmodem*  # macOS
    python -m dsr2.gripper_server --mock           # Mock mode (no hardware)
"""

import argparse
import logging
import socket
import struct
import sys
import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import IntEnum
from typing import ClassVar, Optional

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("gripper_server")


class Command(IntEnum):
    """Protocol commands (1 byte)"""

    SET_POSITION = 0x01
    GET_STATE = 0x02
    INIT = 0x03
    STOP = 0x04
    SET_FORCE = 0x05
    OPEN = 0x06
    CLOSE = 0x07


class Status(IntEnum):
    """Gripper status codes"""

    OK = 0x00
    ERROR = 0x01
    NOT_INITIALIZED = 0x02
    OBJECT_HELD = 0x03
    NO_OBJECT = 0x04


@dataclass
class GripperState:
    """Current gripper state"""

    position: int = 0  # 0-1000
    force: int = 100  # 20-100
    initialized: bool = False
    object_held: bool = False
    error: str = ""


class AbstractGripper(ABC):
    """Abstract gripper interface"""

    @abstractmethod
    def connect(self) -> bool: ...
    @abstractmethod
    def disconnect(self) -> None: ...
    @abstractmethod
    def initialize(self) -> bool: ...
    @abstractmethod
    def set_force(self, force: int) -> bool: ...
    @abstractmethod
    def set_position(self, position: int) -> bool: ...
    @abstractmethod
    def open(self) -> bool: ...
    @abstractmethod
    def close(self, position: int = 0) -> bool: ...
    @abstractmethod
    def get_state(self) -> GripperState: ...


class ModbusGripper(AbstractGripper):
    """DH Gripper Modbus RTU interface using pymodbus"""

    # Modbus registers
    REG_INIT = 0x0100
    REG_FORCE = 0x0101
    REG_POSITION = 0x0103
    REG_STATUS = 0x0120

    def __init__(self, port: str, baudrate: int = 115200, slave_id: int = 1):
        try:
            from pymodbus.client import ModbusSerialClient
        except ImportError:
            logger.error("pymodbus not installed. Run: pip install pymodbus")
            sys.exit(1)

        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.client: Optional["ModbusSerialClient"] = None
        self.state = GripperState()
        self._lock = threading.Lock()

    def connect(self) -> bool:
        """Connect to the gripper via Modbus RTU"""
        try:
            from pymodbus.client import ModbusSerialClient

            self.client = ModbusSerialClient(
                port=self.port,
                baudrate=self.baudrate,
                parity="N",
                stopbits=1,
                bytesize=8,
                timeout=1,
            )

            if not self.client.connect():
                logger.error(f"Failed to connect to {self.port}")
                return False

            logger.info(f"Connected to gripper at {self.port}")
            return True

        except ImportError:
            logger.error("pymodbus not installed. Run: pip install pymodbus")
            return False
        except Exception as e:
            logger.error(f"Connection error: {e}")
            return False

    def disconnect(self):
        """Disconnect from the gripper"""
        if self.client:
            self.client.close()
            logger.info("Disconnected from gripper")

    def initialize(self) -> bool:
        """Initialize the gripper"""
        with self._lock:
            if not self.client:
                return False
            try:
                self.client.write_register(self.REG_INIT, 0x01, device_id=self.slave_id)
                time.sleep(0.5)  # Wait for initialization
                self.state.initialized = True
                logger.info("Gripper initialized")
                return True
            except Exception as e:
                logger.error(f"Initialize error: {e}")
                self.state.error = str(e)
                return False

    def set_force(self, force: int) -> bool:
        """Set gripper force (20-100%)"""
        force = max(20, min(100, force))
        with self._lock:
            if not self.client or not self.state.initialized:
                return False
            try:
                self.client.write_register(self.REG_FORCE, force, device_id=self.slave_id)
                self.state.force = force
                logger.info(f"Set force: {force}%")
                time.sleep(0.02)  # Short delay
                return True
            except Exception as e:
                logger.error(f"Set force error: {e}")
                self.state.error = str(e)
                return False

    def set_position(self, position: int) -> bool:
        """Set gripper position (0-1000)"""
        position = max(0, min(1000, position))
        with self._lock:
            if not self.client or not self.state.initialized:
                return False
            try:
                self.client.write_register(self.REG_POSITION, position, device_id=self.slave_id)
                self.state.position = position
                logger.info(f"Set position: {position}/1000")
                return True
            except Exception as e:
                logger.error(f"Set position error: {e}")
                self.state.error = str(e)
                return False

    def read_status(self) -> bool:
        """Read gripper status"""
        with self._lock:
            if not self.client:
                return False
            try:
                result = self.client.read_holding_registers(self.REG_STATUS, count=1, device_id=self.slave_id)
                if not result.isError():
                    status = result.registers[0]
                    self.state.object_held = bool(status & 0x01)
                    return True
                return False
            except Exception as e:
                logger.error(f"Read status error: {e}")
                return False

    def open(self) -> bool:
        """Open gripper fully"""
        self.set_force(100)
        return self.set_position(1000)

    def close(self, position: int = 0) -> bool:
        """Close gripper to position"""
        self.set_force(100)
        return self.set_position(position)

    def get_state(self) -> GripperState:
        """Get current gripper state"""
        self.read_status()
        return self.state


class MockGripper(AbstractGripper):
    """Mock gripper for testing without hardware"""

    def __init__(self):
        self.state = GripperState()
        self._lock = threading.Lock()

    def connect(self) -> bool:
        logger.info("[MOCK] Gripper connected (simulated)")
        return True

    def disconnect(self):
        logger.info("[MOCK] Gripper disconnected")

    def initialize(self) -> bool:
        with self._lock:
            self.state.initialized = True
            logger.info("[MOCK] Gripper initialized (simulated)")
            return True

    def set_force(self, force: int) -> bool:
        force = max(20, min(100, force))
        with self._lock:
            self.state.force = force
            logger.info(f"[MOCK] Set force: {force}%")
            return True

    def set_position(self, position: int) -> bool:
        position = max(0, min(1000, position))
        with self._lock:
            self.state.position = position
            # Simulate object detection at low positions
            self.state.object_held = position < 200
            logger.info(f"[MOCK] Set position: {position}/1000")
            return True

    def open(self) -> bool:
        return self.set_position(1000)

    def close(self, position: int = 0) -> bool:
        return self.set_position(position)

    def get_state(self) -> GripperState:
        return self.state


@dataclass
class GripperServer:
    """TCP server for gripper communication with Docker container"""

    gripper: AbstractGripper
    host: str = "0.0.0.0"
    port: int = 9999

    server_socket: Optional[socket.socket] = None
    running: bool = field(default=False, init=False)
    clients: list[socket.socket] = field(default_factory=list[socket.socket], init=False)

    PACKET_SIZE: ClassVar[int] = 5  # Command packet(little-endian): [cmd:1][position:2][force:2] = 5 bytes
    RESPONSE_SIZE: ClassVar[int] = 6  # Response packet: [status:1][position:2][force:2][flags:1] = 6 bytes

    def start(self):
        """Start the TCP server"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        self.running = True

        logger.info(f"Gripper server listening on {self.host}:{self.port}")

        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                try:
                    client_socket, addr = self.server_socket.accept()
                except socket.timeout:
                    continue

                client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                logger.info(f"Client connected: {addr}")
                self.clients.append(client_socket)

                # Handle client in a new thread
                thread = threading.Thread(
                    target=self._handle_client,
                    args=(client_socket, addr),
                    daemon=True,
                )
                thread.start()

            except Exception as e:
                if self.running:
                    logger.error(f"Server error: {e}")

    def stop(self):
        """Stop the server"""
        self.running = False
        for client in self.clients:
            try:
                client.close()
            except Exception:
                pass
        if self.server_socket:
            self.server_socket.close()
        logger.info("Server stopped")

    def _handle_client(self, client_socket: socket.socket, addr: tuple[str, int]) -> None:
        """Handle a single client connection"""
        try:
            while self.running:
                data = client_socket.recv(self.PACKET_SIZE)
                if not data:
                    break

                if len(data) < self.PACKET_SIZE:
                    continue

                # Parse command packet: [cmd:1][position:2][force:2]
                cmd, position, force = struct.unpack("<BHH", data)

                # Process command
                match cmd:
                    case Command.INIT:
                        status = Status.OK if self.gripper.initialize() else Status.ERROR
                    case Command.SET_POSITION:
                        status = Status.OK if self.gripper.set_position(position) else Status.ERROR
                    case Command.SET_FORCE:
                        status = Status.OK if self.gripper.set_force(force) else Status.ERROR
                    case Command.OPEN:
                        status = Status.OK if self.gripper.open() else Status.ERROR
                    case Command.CLOSE:
                        status = Status.OK if self.gripper.close(position) else Status.ERROR
                    case Command.GET_STATE:
                        status = Status.OK
                    case Command.STOP:
                        self.gripper.open()
                        status = Status.OK
                    case _:
                        status = Status.ERROR

                # Get current state
                state = self.gripper.get_state()

                # Build response: [status:1][position:2][force:2][flags:1]
                flags = 0
                if state.initialized:
                    flags |= 0x01
                if state.object_held:
                    flags |= 0x02

                response = struct.pack(
                    "<BHHB",
                    status,
                    state.position,
                    state.force,
                    flags,
                )
                client_socket.send(response)

        except ConnectionResetError:
            logger.info(f"Client disconnected: {addr}")
        except Exception as e:
            logger.error(f"Client error: {e}")
        finally:
            if client_socket in self.clients:
                self.clients.remove(client_socket)
            client_socket.close()


def find_serial_port() -> Optional[str]:
    """Auto-detect gripper serial port"""
    import glob

    if sys.platform == "win32":
        # Windows: Try common COM ports
        import serial.tools.list_ports

        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if "USB" in port.description or "ACM" in port.description:
                logger.info(f"Found potential gripper port: {port.device}")
                return port.device
        # Default fallback
        return "COM3"
    elif sys.platform == "darwin":
        # macOS
        patterns = ["/dev/tty.usbmodem*", "/dev/cu.usbmodem*"]
        for pattern in patterns:
            matches = glob.glob(pattern)
            if matches:
                return matches[0]
        return None
    else:
        # Linux
        patterns = ["/dev/ttyACM*", "/dev/ttyUSB*"]
        for pattern in patterns:
            matches = glob.glob(pattern)
            if matches:
                return matches[0]
        return None


def main():
    parser = argparse.ArgumentParser(
        description="DH Gripper TCP Server",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Windows
    python -m dsr2.gripper_server --port COM3

    # Linux
    python -m dsr2.gripper_server --port /dev/ttyACM0

    # macOS
    python -m dsr2.gripper_server --port /dev/tty.usbmodem14101

    # Mock mode (no hardware)
    python -m dsr2.gripper_server --mock

    # Auto-detect port
    python -m dsr2.gripper_server --auto
        """,
    )
    parser.add_argument(
        "--port",
        "-p",
        help="Serial port for gripper (e.g., COM3, /dev/ttyACM0)",
    )
    parser.add_argument(
        "--auto",
        "-a",
        action="store_true",
        help="Auto-detect serial port",
    )
    parser.add_argument(
        "--mock",
        "-m",
        action="store_true",
        help="Run in mock mode (no hardware)",
    )
    parser.add_argument(
        "--tcp-port",
        type=int,
        default=9999,
        help="TCP port for container communication (default: 9999)",
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="TCP host to bind (default: 0.0.0.0)",
    )

    args = parser.parse_args()

    # Determine gripper mode
    if args.mock:
        logger.info("Starting in MOCK mode (no hardware)")
        gripper = MockGripper()
    else:
        # Find serial port
        serial_port = args.port
        if args.auto or not serial_port:
            serial_port = find_serial_port()
            if not serial_port:
                logger.error("Could not auto-detect gripper port. Use --port or --mock")
                sys.exit(1)
            logger.info(f"Auto-detected port: {serial_port}")

        gripper = ModbusGripper(serial_port)

    # Connect to gripper
    if not gripper.connect():
        logger.error("Failed to connect to gripper")
        sys.exit(1)

    # Start TCP server
    server = GripperServer(gripper, host=args.host, port=args.tcp_port)

    try:
        server.start()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
    finally:
        server.stop()
        gripper.disconnect()


if __name__ == "__main__":
    main()
