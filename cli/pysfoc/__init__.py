"""
Reusable PySFOC host-side helpers (PacketCommander client + velocity control helpers).

Designed to satisfy the PacketCommander interface described in AGENT_PID.md so the
PID auto-tuner can depend on a stable API.
"""

from .api import MotorState, PySFOCClient, TelemetryConfig, VelocityTelemetry
from .packet_commander import (
    BinaryPacketCommanderClient,
    PacketCommanderClient,
    TelemetrySample,
    map_control_mode,
    map_status,
)
from .constants import (
    CONTROL_MODE_IDS,
    CONTROL_MODE_NAMES,
    OPENLOOP_MODES,
    REG_CONTROL_MODE,
    REG_ENABLE,
    REG_ANGLE,
    REG_POSITION,
    REG_SENSOR_MECH_ANGLE,
    REG_TARGET,
    REG_VELOCITY,
    REG_VEL_PID_D,
    REG_VEL_PID_I,
    REG_VEL_PID_P,
    REG_VEL_PID_LIMIT,
    REG_TELEMETRY_DOWNSAMPLE,
    REG_TELEMETRY_CTRL,
    REG_TELEMETRY_REG,
)

__all__ = [
    "PySFOCClient",
    "PacketCommanderClient",
    "BinaryPacketCommanderClient",
    "MotorState",
    "TelemetrySample",
    "TelemetryConfig",
    "VelocityTelemetry",
    "map_status",
    "map_control_mode",
    "CONTROL_MODE_IDS",
    "CONTROL_MODE_NAMES",
    "OPENLOOP_MODES",
    "REG_CONTROL_MODE",
    "REG_ENABLE",
    "REG_ANGLE",
    "REG_POSITION",
    "REG_SENSOR_MECH_ANGLE",
    "REG_TARGET",
    "REG_VELOCITY",
    "REG_VEL_PID_P",
    "REG_VEL_PID_I",
    "REG_VEL_PID_D",
    "REG_VEL_PID_LIMIT",
    "REG_TELEMETRY_DOWNSAMPLE",
    "REG_TELEMETRY_CTRL",
    "REG_TELEMETRY_REG",
]
