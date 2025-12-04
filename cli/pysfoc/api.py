"""
PySFOC high-level interface that matches the AGENT_PID PacketCommander contract:

- vel_pid_set(P, I, D)
- vel_target_set(target_velocity)
- vel_ctrl_enable(enable=True/False)
- telemetry_config(enable, period/downsample, fields)
- poll_velocity_sample() returning target/measured velocities + host timestamp

This wraps PacketCommanderClient and keeps a lightweight MotorState for
convenient state tracking in host-side tools.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

from .constants import (
    CONTROL_MODE_IDS,
    OPENLOOP_MODES,
    REG_CONTROL_MODE,
    REG_ENABLE,
    REG_TARGET,
    REG_TELEMETRY_CTRL,
    REG_TELEMETRY_DOWNSAMPLE,
    REG_VELOCITY,
    REG_VEL_PID_D,
    REG_VEL_PID_I,
    REG_VEL_PID_P,
)
from .packet_commander import PacketCommanderClient, TelemetrySample


@dataclass
class MotorState:
    client: PacketCommanderClient
    control_mode: Optional[int] = None
    open_loop: Optional[bool] = None
    target: float = 0.0
    running: bool = False
    running_dir: int = 1
    telemetry: Optional[TelemetrySample] = None
    control_mode_val: Optional[int] = None
    torque_mode_val: Optional[int] = None
    status_val: Optional[int] = None
    enable_val: Optional[int] = None

    def _write_and_confirm(self, reg: int, value):
        resp = self.client.write_reg(reg, value)
        return resp if resp is not None else None

    def refresh_status(self):
        cm = self.client.read_reg(REG_CONTROL_MODE)
        if cm is not None:
            self.control_mode = int(cm)
            self.control_mode_val = int(cm)
            self.open_loop = self.control_mode in OPENLOOP_MODES
        en = self.client.read_reg(REG_ENABLE)
        if en is not None:
            self.enable_val = int(en)

    def set_control_mode(self, mode: int):
        resp = self._write_and_confirm(REG_CONTROL_MODE, mode)
        if resp is not None:
            mode_int = int(resp)
            self.control_mode = mode_int
            self.control_mode_val = mode_int
            self.open_loop = mode_int in OPENLOOP_MODES
        return resp

    def set_enable(self, enable: bool):
        resp = self._write_and_confirm(REG_ENABLE, 1 if enable else 0)
        if resp is not None:
            self.enable_val = int(resp)
        return resp

    def set_target(self, target: float):
        resp = self._write_and_confirm(REG_TARGET, target)
        if resp is not None:
            self.target = float(resp)
        else:
            self.target = target
        return resp


@dataclass
class TelemetryConfig:
    regs: Sequence[int]
    controller_index: int = 0
    downsample: int = 0
    base_rate_hz: float = 1000.0  # used only when caller requests a rate -> downsample conversion


@dataclass
class VelocityTelemetry:
    t_host: float
    vel_target: Optional[float]
    vel_measured: Optional[float]
    raw: TelemetrySample


class PySFOCClient:
    """
    Convenience wrapper that exposes the PacketCommander operations the PID tuner assumes.

    Methods mirror the command names in AGENT_PID.md:
      - vel_pid_set(P, I, D)
      - vel_target_set(target_velocity)
      - vel_ctrl_enable(enable=True/False, control_mode=velocity)
      - telemetry_config(...)
      - poll_velocity_sample()
    """

    def __init__(self, port: str, baud: int = 460800, timeout: float = 0.3, motor_index: int = 0):
        self.client = PacketCommanderClient(port, baud, timeout)
        self.motor_index = motor_index
        self.state = MotorState(self.client)

    # --- PID and target control --------------------------------------------
    def vel_pid_set(self, p: float, i: float, d: float):
        """VEL_PID_SET: write P/I/D to the board."""
        self.client.write_reg(REG_VEL_PID_P, p)
        self.client.write_reg(REG_VEL_PID_I, i)
        self.client.write_reg(REG_VEL_PID_D, d)

    def vel_target_set(self, target_velocity: float):
        """VEL_TARGET_SET: update motor.target (velocity mode)."""
        self.state.set_target(target_velocity)

    def vel_ctrl_enable(self, enable: bool, control_mode: int | str = CONTROL_MODE_IDS["velocity"]):
        """
        VEL_CTRL_ENABLE: enable/disable closed-loop velocity control.

        control_mode can be an int register value or a string key in CONTROL_MODE_IDS.
        """
        if isinstance(control_mode, str):
            cm_val = CONTROL_MODE_IDS.get(control_mode, CONTROL_MODE_IDS["velocity"])
        else:
            cm_val = int(control_mode)
        self.state.set_control_mode(cm_val)
        self.state.set_enable(enable)

    # --- Telemetry ---------------------------------------------------------
    def telemetry_config(
        self,
        enable: bool = True,
        regs: Optional[Sequence[int]] = None,
        controller_index: int = 0,
        downsample: Optional[int] = None,
        target_rate_hz: Optional[float] = None,
        base_rate_hz: float = 1000.0,
    ) -> TelemetryConfig:
        """
        TELEMETRY_CONFIG: configure MCU telemetry for the tuner.

        - regs: list of register IDs to include (defaults to [target, velocity]).
        - downsample: direct REG_TELEMETRY_DOWNSAMPLE value (0 = off in firmware).
        - target_rate_hz: optional host-requested sample rate; converted to downsample
          assuming base_rate_hz as the firmware publisher rate.
        """
        reg_list: List[int] = list(regs) if regs is not None else [REG_TARGET, REG_VELOCITY]
        self.client.set_telemetry_registers(reg_list, motor=self.motor_index)

        ds_val: Optional[int] = downsample
        if ds_val is None and target_rate_hz:
            if target_rate_hz <= 0:
                ds_val = 0
            else:
                # Downsample value is (publish_every_n - 1) when base rate is known.
                interval = max(1, round(base_rate_hz / target_rate_hz))
                ds_val = max(0, int(interval) - 1)
        if ds_val is not None:
            self.client.write_reg(REG_TELEMETRY_DOWNSAMPLE, int(ds_val))
        # In current firmware REG_TELEMETRY_CTRL selects controller; enabling is implicit.
        self.client.write_reg(REG_TELEMETRY_CTRL, controller_index)
        if not enable:
            # Write a downsample of 0 and re-select controller to effectively pause streaming.
            self.client.write_reg(REG_TELEMETRY_DOWNSAMPLE, 0)
        return TelemetryConfig(regs=reg_list, controller_index=controller_index, downsample=ds_val or 0, base_rate_hz=base_rate_hz)

    def poll_velocity_sample(self) -> Optional[VelocityTelemetry]:
        """
        Parse the latest telemetry frame into a VelocityTelemetry record.

        Returns None if no telemetry is available. Unknown/missing fields stay as None.
        """
        sample = self.client.poll_telemetry()
        if not sample:
            return None
        target_val = sample.values.get(REG_TARGET)
        vel_val = sample.values.get(REG_VELOCITY)
        target = float(target_val) if isinstance(target_val, (int, float)) else None
        measured = float(vel_val) if isinstance(vel_val, (int, float)) else None
        return VelocityTelemetry(t_host=sample.timestamp, vel_target=target, vel_measured=measured, raw=sample)

    # --- utility -----------------------------------------------------------
    def close(self):
        self.client.close()
