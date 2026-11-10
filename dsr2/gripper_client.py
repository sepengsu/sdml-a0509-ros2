"""
Gripper TCP Client

Connects to the gripper server running on the host machine.
Used by ROS 2 bridge node inside the Docker container.
"""

import logging
import socket
import threading
import time
from typing import Optional, Type

from .gripper_protocol import (
    RESPONSE_SIZE,
    Command,
    GripperState,
    Status,
    pack_command,
    unpack_response,
)

logger = logging.getLogger("gripper_client")


class GripperClient:
    """TCP client for communicating with the host gripper server"""

    def __init__(self, host: str = "host.docker.internal", port: int = 9999):
        """
        Initialize gripper client.

        Args:
            host: Gripper server host. Use "host.docker.internal" for Docker,
                  or the host's IP address.
            port: Gripper server TCP port (default: 9999)
        """
        self.host = host
        self.port = port
        self.socket: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self._connected = False
        self._last_state = GripperState()

    def connect(self, timeout: float = 5.0, retries: int = 3) -> bool:
        """
        Connect to the gripper server.

        Args:
            timeout: Connection timeout in seconds
            retries: Number of connection retries

        Returns:
            True if connected successfully
        """
        for attempt in range(retries):
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.socket.settimeout(timeout)
                self.socket.connect((self.host, self.port))
                self._connected = True
                logger.info(f"Connected to gripper server at {self.host}:{self.port}")
                return True
            except socket.error as e:
                logger.warning(f"Connection attempt {attempt + 1}/{retries} failed: {e}")
                if self.socket:
                    self.socket.close()
                time.sleep(1.0)

        logger.error(f"Failed to connect to gripper server at {self.host}:{self.port}")
        return False

    def disconnect(self):
        """Disconnect from the gripper server"""
        self._connected = False
        if self.socket:
            try:
                self.socket.close()
            except Exception:
                pass
            self.socket = None
        logger.info("Disconnected from gripper server")

    def is_connected(self) -> bool:
        """Check if connected to server"""
        return self._connected and self.socket is not None

    def _send_command(self, cmd: Command, position: int = 0, force: int = 0) -> Optional[GripperState]:
        """
        Send a command and receive response.

        Args:
            cmd: Command to send
            position: Position value (0-1000)
            force: Force value (20-100)

        Returns:
            GripperState on success, None on failure
        """
        if self.socket is None or not self.is_connected():
            logger.error("Not connected to gripper server")
            return None

        with self._lock:
            try:
                # Send command
                packet = pack_command(cmd, position, force)
                self.socket.send(packet)

                # Receive response
                data = self.socket.recv(RESPONSE_SIZE)
                if len(data) != RESPONSE_SIZE:
                    logger.error(f"Invalid response size: {len(data)}")
                    return None

                state = unpack_response(data)
                self._last_state = state
                return state

            except socket.error as e:
                logger.error(f"Communication error: {e}")
                self._connected = False
                return None

    def initialize(self) -> bool:
        """Initialize the gripper"""
        state = self._send_command(Command.INIT)
        if state and state.status == Status.OK:
            logger.info("Gripper initialized")
            return True
        return False

    def set_force(self, force: int) -> bool:
        """
        Set gripper force.

        Args:
            force: Force percentage (20-100)

        Returns:
            True on success
        """
        force = max(20, min(100, force))
        state = self._send_command(Command.SET_FORCE, force=force)
        if state and state.status == Status.OK:
            logger.debug(f"Set force: {force}%")
            return True
        return False

    def set_position(self, position: int) -> bool:
        """
        Set gripper position.

        Args:
            position: Position (0-1000, where 0=closed, 1000=open)

        Returns:
            True on success
        """
        position = max(0, min(1000, position))
        state = self._send_command(Command.SET_POSITION, position=position)
        if state and state.status == Status.OK:
            logger.debug(f"Set position: {position}/1000")
            return True
        return False

    def open(self) -> bool:
        """Open the gripper fully"""
        state = self._send_command(Command.OPEN)
        if state and state.status == Status.OK:
            logger.info("Gripper opened")
            return True
        return False

    def close(self, position: int = 0) -> bool:
        """
        Close the gripper.

        Args:
            position: Target position (0-1000)

        Returns:
            True on success
        """
        state = self._send_command(Command.CLOSE, position=position)
        if state and state.status == Status.OK:
            logger.info(f"Gripper closed to {position}")
            return True
        return False

    def stop(self) -> bool:
        """Emergency stop - opens the gripper"""
        state = self._send_command(Command.STOP)
        return state is not None

    def get_state(self) -> Optional[GripperState]:
        """Get current gripper state"""
        return self._send_command(Command.GET_STATE)

    @property
    def last_state(self) -> GripperState:
        """Get the last known gripper state"""
        return self._last_state


class GripperClientContext:
    """Context manager for gripper client"""

    def __init__(self, host: str = "host.docker.internal", port: int = 9999):
        self.client = GripperClient(host, port)

    def __enter__(self) -> GripperClient:
        if not self.client.connect():
            raise ConnectionError("Failed to connect to gripper server")
        return self.client

    def __exit__(
        self, exc_type: Type[BaseException], exc_val: BaseException, exc_tb: Optional[Type[BaseException]]
    ) -> bool:
        self.client.disconnect()
        return False
