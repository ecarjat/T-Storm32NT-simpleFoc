"""
Register and protocol constants shared by the PySFOC host utilities.

Values mirror the PacketCommander register map used by the firmware and the
interactive cli/pysimplefoc_cli.py helper.
"""

from __future__ import annotations

# PacketCommander register IDs (SimpleFOCRegisters.h)
REGISTER_IDS = {
    "status": 0x00,
    "target": 0x01,
    "enable": 0x04,
    "control_mode": 0x05,
    "torque_mode": 0x06,
    "angle": 0x09,
    "position": 0x10,
    "velocity": 0x11,
    "sensor_angle": 0x12,
    "sensor_mech_angle": 0x13,
    "sensor_velocity": 0x14,
    "sensor_ts": 0x15,
    "telemetry_reg": 0x1A,
    "telemetry_ctrl": 0x1B,
    "telemetry_downsample": 0x1C,
    "telemetry_min_elapsed": 0x1E,
    "vel_pid_p": 0x30,
    "vel_pid_i": 0x31,
    "vel_pid_d": 0x32,
    "vel_pid_limit": 0x33,
    "voltage_limit": 0x50,
    "velocity_limit": 0x52,
    "driver_voltage_limit": 0x53,
    "driver_voltage_psu": 0x55,
    "pole_pairs": 0x63,
}

# Export per-register constants (REG_FOO) for backward compatibility and linting.
REG_STATUS = REGISTER_IDS["status"]
REG_TARGET = REGISTER_IDS["target"]
REG_ENABLE = REGISTER_IDS["enable"]
REG_CONTROL_MODE = REGISTER_IDS["control_mode"]
REG_TORQUE_MODE = REGISTER_IDS["torque_mode"]
REG_ANGLE = REGISTER_IDS["angle"]
REG_POSITION = REGISTER_IDS["position"]
REG_VELOCITY = REGISTER_IDS["velocity"]
REG_SENSOR_ANGLE = REGISTER_IDS["sensor_angle"]
REG_SENSOR_MECH_ANGLE = REGISTER_IDS["sensor_mech_angle"]
REG_SENSOR_VELOCITY = REGISTER_IDS["sensor_velocity"]
REG_SENSOR_TS = REGISTER_IDS["sensor_ts"]
REG_TELEMETRY_REG = REGISTER_IDS["telemetry_reg"]
REG_TELEMETRY_CTRL = REGISTER_IDS["telemetry_ctrl"]
REG_TELEMETRY_DOWNSAMPLE = REGISTER_IDS["telemetry_downsample"]
REG_TELEMETRY_MIN_ELAPSED = REGISTER_IDS["telemetry_min_elapsed"]
REG_VEL_PID_P = REGISTER_IDS["vel_pid_p"]
REG_VEL_PID_I = REGISTER_IDS["vel_pid_i"]
REG_VEL_PID_D = REGISTER_IDS["vel_pid_d"]
REG_VEL_PID_LIMIT = REGISTER_IDS["vel_pid_limit"]
REG_VOLTAGE_LIMIT = REGISTER_IDS["voltage_limit"]
REG_VELOCITY_LIMIT = REGISTER_IDS["velocity_limit"]
REG_DRIVER_VOLTAGE_LIMIT = REGISTER_IDS["driver_voltage_limit"]
REG_DRIVER_VOLTAGE_PSU = REGISTER_IDS["driver_voltage_psu"]
REG_POLE_PAIRS = REGISTER_IDS["pole_pairs"]

READ_REGEX_PATTERN = r"r(?P<reg>\d+)\s*=\s*(?P<val>[-+0-9.eE]+)"
TELEM_HEADER_PATTERN = r"H(?P<tid>\d+)=?(?P<body>.*)"
TELEM_DATA_PATTERN = r"T(?P<tid>\d+)=?(?P<body>.*)"

DEFAULT_TELEM_REGS = [
    (0, REG_TARGET),
    (0, REG_ANGLE),
    (0, REG_POSITION),  # Sensor position (rotations + angle)
    (0, REG_VELOCITY),
    (0, REG_STATUS),
]

# Map reg ID -> friendly name (includes aliases where names overlap on purpose).
REG_NAME_MAP = {val: name for name, val in REGISTER_IDS.items()}
REG_NAME_MAP.update(
    {
        REG_POSITION: "sensor_position",  # rotations + angle
    }
)

REG_VALUE_FIELDS = {
    REG_POSITION: 2,  # position register returns rotations + angle rad
}

STATUS_NAMES = {
    0x00: "uninitialized",
    0x01: "initializing",
    0x02: "uncalibrated",
    0x03: "calibrating",
    0x04: "ready",
    0x08: "error",
    0x0E: "calib_failed",
    0x0F: "init_failed",
}

TORQUE_MODE_NAMES = {
    0x00: "voltage",
    0x01: "dc_current",
    0x02: "foc_current",
}

CONTROL_MODE_IDS = {
    "torque": 0,
    "velocity": 1,
    "angle": 2,
    "vel_openloop": 3,
    "angle_openloop": 4,
}
CONTROL_MODE_NAMES = {v: k for k, v in CONTROL_MODE_IDS.items()}
OPENLOOP_MODES = {CONTROL_MODE_IDS["vel_openloop"], CONTROL_MODE_IDS["angle_openloop"]}
CONTROL_MODE_SEQUENCE = [
    CONTROL_MODE_IDS["torque"],
    CONTROL_MODE_IDS["velocity"],
    CONTROL_MODE_IDS["angle"],
    CONTROL_MODE_IDS["vel_openloop"],
    CONTROL_MODE_IDS["angle_openloop"],
]


def reg_display_name(reg_id: int) -> str:
    return REG_NAME_MAP.get(reg_id, f"reg{reg_id}")
