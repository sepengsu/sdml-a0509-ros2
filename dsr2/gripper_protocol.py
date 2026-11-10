"""
Gripper TCP Protocol Definitions

Shared between gripper server (host) and gripper client (container).
"""

import struct
from dataclasses import dataclass
from enum import IntEnum


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
    status: Status = Status.OK
    position: int = 0  # 0-1000
    force: int = 100  # 20-100
    initialized: bool = False
    object_held: bool = False


# Packet formats (little-endian)
# Command packet: [cmd:1][position:2][force:2] = 5 bytes
# Response packet: [status:1][position:2][force:2][flags:1] = 6 bytes
COMMAND_FORMAT = "<BHH"
RESPONSE_FORMAT = "<BHHB"
COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)
RESPONSE_SIZE = struct.calcsize(RESPONSE_FORMAT)


def pack_command(cmd: Command, position: int = 0, force: int = 0) -> bytes:
    """Pack a command into bytes"""
    return struct.pack(COMMAND_FORMAT, cmd, position, force)


def unpack_command(data: bytes) -> tuple[Command, int, int]:
    """Unpack command from bytes -> (cmd, position, force)"""
    cmd, position, force = struct.unpack(COMMAND_FORMAT, data)
    return Command(cmd), position, force


def pack_response(state: GripperState) -> bytes:
    """Pack a response into bytes"""
    flags = 0
    if state.initialized:
        flags |= 0x01
    if state.object_held:
        flags |= 0x02
    return struct.pack(RESPONSE_FORMAT, state.status, state.position, state.force, flags)


def unpack_response(data: bytes) -> GripperState:
    """Unpack response from bytes -> GripperState"""
    status, position, force, flags = struct.unpack(RESPONSE_FORMAT, data)
    return GripperState(
        status=Status(status),
        position=position,
        force=force,
        initialized=bool(flags & 0x01),
        object_held=bool(flags & 0x02),
    )
