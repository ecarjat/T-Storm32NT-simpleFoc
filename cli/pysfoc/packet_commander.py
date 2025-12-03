"""
Thin PacketCommander client reused by the PID tuning tooling.

This mirrors the helpers in cli/pysimplefoc_cli.py but lives in a reusable
module so other scripts can depend on it without the UI/plotting baggage.
"""

from __future__ import annotations

import re
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Union

import serial  # type: ignore[import-untyped]  # pyserial

from .constants import (
    CONTROL_MODE_NAMES,
    READ_REGEX_PATTERN,
    REG_NAME_MAP,
    REG_VALUE_FIELDS,
    REG_VEL_PID_D,
    REG_VEL_PID_I,
    REG_VEL_PID_P,
    REG_ENABLE,
    REG_STATUS,
    REG_TELEMETRY_DOWNSAMPLE,
    REG_TELEMETRY_REG,
    REG_TARGET,
    REG_VELOCITY,
    TELEM_DATA_PATTERN,
    TELEM_HEADER_PATTERN,
    DEFAULT_TELEM_REGS,
    REG_CONTROL_MODE,
    REG_TORQUE_MODE,
    STATUS_NAMES,
)

READ_REGEX = re.compile(READ_REGEX_PATTERN, re.IGNORECASE)
TELEM_HEADER_REGEX = re.compile(TELEM_HEADER_PATTERN)
TELEM_DATA_REGEX = re.compile(TELEM_DATA_PATTERN)


@dataclass
class TelemetrySample:
    regs: List[tuple[int, int]]
    values: Dict[int, Union[float, List[float]]]
    timestamp: float = 0.0


class PacketCommanderClient:
    """
    Text PacketCommander transport.

    The firmware echoes reads/writes in `r<reg>=<value>` lines and streams
    telemetry using `H<id>=...` header + `T<id>=...` data lines.
    """

    def __init__(
        self,
        port: str,
        baud: int,
        timeout: float = 0.3,
        ser: Optional[serial.Serial] = None,
        debug: bool = False,
        logger: Optional[Callable[[str], None]] = None,
    ):
        self.ser = ser or serial.Serial(port, baudrate=baud, timeout=timeout)
        # Default mapping matches firmware telemetry_registers order in comms_streams.cpp
        self.telemetry_headers: Dict[int, List[tuple[int, int]]] = {0: DEFAULT_TELEM_REGS.copy()}
        self.debug = debug
        self._log = logger or (lambda msg: print(msg))

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    # --- basic register I/O -------------------------------------------------
    def _write_line(self, line: str):
        if self.debug:
            self._log(f">> {line}")
        self.ser.write((line + "\n").encode("ascii"))

    def _read_response(self, expect_reg: int, timeout: float = 0.5):
        deadline = time.time() + timeout
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            try:
                line = raw.decode("ascii", errors="ignore").strip()
            except Exception:
                continue
            if self.debug:
                self._log(f"<< {line}")
            m = READ_REGEX.match(line)
            if not m:
                if self.debug:
                    self._log(f"!! unmatched line while waiting for reg {expect_reg}: {line}")
                continue
            reg = int(m.group("reg"))
            if reg != expect_reg:
                if self.debug:
                    self._log(f"!! response reg {reg} != expected {expect_reg}: {line}")
                continue
            try:
                raw_val = m.group("val")
                if "," in raw_val:
                    raw_val = raw_val.split(",", 1)[0]
                return float(raw_val)
            except ValueError:
                if self.debug:
                    self._log(f"!! failed to parse value in line: {line}")
                return None
        return None

    def read_reg(self, reg: int):
        self._write_line(f"R{reg}")
        return self._read_response(reg)

    def write_reg(self, reg: int, value):
        self._write_line(f"R{reg}={value}")
        # echo is enabled in firmware; optionally read back
        return self._read_response(reg, timeout=0.5)

    # --- telemetry parsing --------------------------------------------------
    def _parse_telem_header(self, line: str):
        m = TELEM_HEADER_REGEX.match(line)
        if not m:
            return
        tid = int(m.group("tid"))
        body = m.group("body")
        if not body:
            return
        regs: List[tuple[int, int]] = []
        for part in body.split(","):
            try:
                motor_str, reg_str = part.split(":")
                regs.append((int(motor_str), int(reg_str)))
            except ValueError:
                continue
        if regs:
            self.telemetry_headers[tid] = regs

    def _parse_telem_frame(self, line: str) -> Optional[TelemetrySample]:
        m = TELEM_DATA_REGEX.match(line)
        if not m:
            return None
        tid = int(m.group("tid"))
        body = m.group("body")
        if body is None:
            return None
        reg_map = self.telemetry_headers.get(tid, DEFAULT_TELEM_REGS if tid == 0 else None)
        if not reg_map:
            return None
        values = body.split(",") if body else []
        idx = 0
        sample = TelemetrySample(regs=list(reg_map), values={}, timestamp=time.time())
        for (motor_idx, reg_id) in reg_map:
            needed = REG_VALUE_FIELDS.get(reg_id, 1)
            if idx + needed > len(values):
                return None
            chunk = values[idx : idx + needed]
            idx += needed
            parsed: List[float] = []
            for raw in chunk:
                try:
                    parsed.append(float(raw))
                except ValueError:
                    parsed.append(float("nan"))
            if motor_idx != 0:
                continue  # library tracks motor 0 by default
            if needed == 1:
                if reg_id in (REG_STATUS, REG_ENABLE, REG_TORQUE_MODE, REG_CONTROL_MODE):
                    sample.values[reg_id] = int(parsed[0])
                else:
                    sample.values[reg_id] = parsed[0]
            else:
                sample.values[reg_id] = parsed
        return sample

    def poll_telemetry(self) -> Optional[TelemetrySample]:
        """Non-blocking poll for telemetry packets. Returns the most recent sample if any."""
        latest: Optional[TelemetrySample] = None
        try:
            while self.ser.in_waiting:
                raw = self.ser.readline()
                if not raw:
                    break
                try:
                    line = raw.decode("ascii", errors="ignore").strip()
                except Exception:
                    continue
                if self.debug:
                    self._log(f"<< {line}")
                if not line:
                    continue
                if line.startswith("H"):
                    self._parse_telem_header(line)
                    continue
                if line.startswith("T"):
                    sample = self._parse_telem_frame(line)
                    if sample:
                        latest = sample
        except Exception:
            # Swallow serial errors in telemetry path to avoid breaking the caller loop
            return latest
        return latest

    def set_telemetry_registers(self, regs: List[int], motor: int = 0):
        """Configure telemetry registers on the MCU (REG_TELEMETRY_REG)."""
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        num = len(regs)
        parts = [str(num)]
        for r in regs:
            parts.append(str(motor))
            parts.append(str(r))
        line = f"R{REG_TELEMETRY_REG}=" + ",".join(parts)
        self._write_line(line)
        # Update local mapping and ask MCU to re-send header
        self.telemetry_headers[motor] = [(motor, r) for r in regs]
        self._write_line(f"R{REG_TELEMETRY_REG}")  # reading the reg triggers header resend

    # --- convenience helpers ------------------------------------------------
    def set_velocity_pid(self, p: float, i: float, d: float):
        """Set velocity PID gains using PacketCommander registers."""
        self.write_reg(REG_VEL_PID_P, p)
        self.write_reg(REG_VEL_PID_I, i)
        self.write_reg(REG_VEL_PID_D, d)

    def set_velocity_target(self, target: float):
        self.write_reg(REG_TARGET, target)

    def set_enable(self, enable: bool):
        self.write_reg(REG_ENABLE, 1 if enable else 0)

    def save_settings(self):
        # Custom packet: "S1" triggers flash save on MCU and responds with SAVE_OK/SAVE_ERR
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._write_line("S1")
        deadline = time.time() + 2.0
        while time.time() < deadline:
            line = self.ser.readline()
            if not line:
                continue
            try:
                text = line.decode("ascii", errors="ignore").strip()
            except Exception:
                continue
            if text.startswith("SAVE_OK"):
                return True
            if text.startswith("SAVE_ERR"):
                return False
        return None


def map_status(val: Optional[int]) -> str:
    if val is None:
        return "n/a"
    return STATUS_NAMES.get(int(val), f"0x{int(val):02X}")


def map_control_mode(val: Optional[int]) -> str:
    if val is None:
        return "n/a"
    return CONTROL_MODE_NAMES.get(int(val), f"0x{int(val):02X}")
