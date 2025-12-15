"""
Thin PacketCommander client reused by the PID tuning tooling.

This mirrors the helpers in cli/pysimplefoc_cli.py but lives in a reusable
module so other scripts can depend on it without the UI/plotting baggage.
"""

from __future__ import annotations

import re
import struct
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple, Union

import serial  # type: ignore[import-untyped]  # pyserial

from .constants import (
    CONTROL_MODE_NAMES,
    CONTROL_MODE_IDS,
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
    REG_POSITION,
    TELEM_DATA_PATTERN,
    TELEM_HEADER_PATTERN,
    DEFAULT_TELEM_REGS,
    REG_CONTROL_MODE,
    REG_TORQUE_MODE,
    STATUS_NAMES,
    REGISTER_IDS,
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

    def run_calibration(self, timeout: float = 10.0):
        """Trigger sensor calibration via C2 and watch for CAL_OK/CAL_SAVE_ERR text output."""
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._write_line("C2")
        deadline = time.time() + timeout
        result = None
        while time.time() < deadline and result is None:
            line = self.ser.readline()
            if not line:
                continue
            try:
                text = line.decode("ascii", errors="ignore").strip()
            except Exception:
                continue
            if not text:
                continue
            if "CAL_OK" in text:
                result = True
            elif "CAL_SAVE_ERR" in text:
                result = False
        return result


class BinaryPacketCommanderClient:
    """
    Binary PacketCommander transport (BinaryIO framing, marker 0xA5).

    Packet layout (BinaryIO):
      [0xA5][size][type][payload...]
        - size = len(type + payload) = 1 + payload_len
        - payload for register response: [reg][raw bytes...]
    """

    MARKER = 0xA5
    PACKET_REGISTER = ord("R")
    PACKET_RESPONSE = ord("r")
    PACKET_TELEM_HEADER = ord("H")
    PACKET_TELEM = ord("T")
    PACKET_SYNC = ord("S")
    PACKET_BOOT = ord("B")
    PACKET_SAVE = ord("S")
    PACKET_SAVE_RESP = ord("s")
    PACKET_CAL = ord("C")
    PACKET_CAL_RESP = ord("c")

    _INT_REGS = {
        REG_STATUS,
        REG_ENABLE,
        REG_TORQUE_MODE,
        REG_CONTROL_MODE,
        REGISTER_IDS["telemetry_ctrl"],
        REGISTER_IDS["pole_pairs"],
    }
    _INT32_REGS = {
        REG_TELEMETRY_DOWNSAMPLE,
        0x15,  # sensor timestamp
    }
    _SIZE_OVERRIDES = {
        REG_POSITION: 8,  # uint32 rotations + float angle
        REGISTER_IDS["telemetry_reg"]: None,  # dynamic
    }

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
        self.telemetry_headers: Dict[int, List[tuple[int, int]]] = {0: DEFAULT_TELEM_REGS.copy()}
        self.debug = debug
        self._log = logger or (lambda msg: print(msg))
        self._rx_buf = bytearray()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    # --- framing helpers ---------------------------------------------------
    def _register_size(self, reg: int, fallback: int = 4) -> Optional[int]:
        if reg in self._SIZE_OVERRIDES and self._SIZE_OVERRIDES[reg] is not None:
            return self._SIZE_OVERRIDES[reg]
        if reg in self._INT_REGS:
            return 1
        if reg == REG_POSITION:
            return 8
        if reg == REGISTER_IDS["telemetry_reg"]:
            return None
        # Known 4-byte registers
        return fallback

    def _send_packet(self, pkt_type: int, payload: bytes = b""):
        size = len(payload) + 1  # includes packet type
        frame = bytes([self.MARKER, size, pkt_type]) + payload
        if self.debug:
            self._log(f">> type={chr(pkt_type)} size={size} payload_len={len(payload)}")
        self.ser.write(frame)

    def _try_extract_frame(self) -> Optional[Tuple[int, bytes]]:
        buf = self._rx_buf
        while True:
            if len(buf) < 3:
                return None
            try:
                marker_idx = buf.index(self.MARKER)
            except ValueError:
                buf.clear()
                return None
            if marker_idx > 0:
                del buf[:marker_idx]
                if len(buf) < 3:
                    return None
            size_byte = buf[1]
            if size_byte == 0:
                del buf[:2]
                continue
            total_len = size_byte + 2  # marker + size + (type+payload)
            if len(buf) < total_len:
                return None
            pkt_type = buf[2]
            payload = bytes(buf[3:total_len])
            del buf[:total_len]
            if self.debug:
                self._log(f"<< type={chr(pkt_type)} payload_len={len(payload)}")
            return pkt_type, payload

    def _read_frame(self, timeout: float = 0.5) -> Optional[Tuple[int, bytes]]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            frame = self._try_extract_frame()
            if frame:
                return frame
            to_read = self.ser.in_waiting or 1
            data = self.ser.read(to_read)
            if data:
                self._rx_buf.extend(data)
                continue
        return None

    # --- register packing/parsing -----------------------------------------
    def _decode_register_payload(self, reg: int, data: bytes, multi: bool = False):
        size = len(data)
        if reg == REG_POSITION or size == 8:
            values = [struct.unpack_from("<I", data, 0)[0], struct.unpack_from("<f", data, 4)[0]]
        elif reg in self._INT32_REGS and size >= 4:
            values = [int.from_bytes(data[:4], "little")]
        elif reg in self._INT_REGS or size == 1:
            values = [int(data[0])]
        elif size == 12:
            values = list(struct.unpack("<fff", data))
        elif size == 3:
            values = list(data[:3])
        elif size % 4 == 0 and size > 0:
            fmt = "<" + "f" * (size // 4)
            values = list(struct.unpack(fmt, data))
        else:
            values = [int.from_bytes(data, "little")]

        if not multi:
            val = values[0] if values else None
            if val is None:
                return None
            if reg in self._INT_REGS:
                return int(val)
            return val
        return values

    def _encode_register_write_payload(self, reg: int, value) -> bytes:
        if reg == REGISTER_IDS["telemetry_reg"]:
            raise ValueError("Use set_telemetry_registers() for REG_TELEMETRY_REG")
        size = self._register_size(reg)
        if size == 1 or reg in self._INT_REGS:
            return bytes([reg, int(value) & 0xFF])
        if reg in self._INT32_REGS:
            return bytes([reg]) + struct.pack("<I", int(value))
        if size == 8 and isinstance(value, (tuple, list)) and len(value) >= 2:
            return bytes([reg]) + struct.pack("<If", int(value[0]) & 0xFFFFFFFF, float(value[1]))
        # default: float32 payload
        return bytes([reg]) + struct.pack("<f", float(value))

    # --- basic register I/O ------------------------------------------------
    def _wait_for_register_response(self, reg: int, timeout: float = 0.5):
        deadline = time.time() + timeout
        while time.time() < deadline:
            frame = self._read_frame(deadline - time.time())
            if not frame:
                continue
            pkt_type, payload = frame
            if pkt_type == self.PACKET_RESPONSE and payload:
                resp_reg = payload[0]
                if resp_reg != reg:
                    continue
                return self._decode_register_payload(resp_reg, payload[1:], multi=False)
            if pkt_type == self.PACKET_TELEM_HEADER:
                self._handle_telem_header(payload)
            elif pkt_type == self.PACKET_TELEM:
                # Consume telemetry silently while waiting for response
                self._handle_telem_frame(payload)
        return None

    def _wait_for_command_response(self, resp_type: int, expect_code: int, timeout: float = 1.5) -> Optional[bool]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            frame = self._read_frame(deadline - time.time())
            if not frame:
                continue
            pkt_type, payload = frame
            if pkt_type == resp_type and len(payload) >= 2:
                code = payload[0]
                if code != expect_code:
                    continue
                return payload[1] == 1
            if pkt_type == self.PACKET_TELEM_HEADER:
                self._handle_telem_header(payload)
            elif pkt_type == self.PACKET_TELEM:
                self._handle_telem_frame(payload)
        return None

    def read_reg(self, reg: int):
        payload = bytes([reg])
        self._send_packet(self.PACKET_REGISTER, payload)
        return self._wait_for_register_response(reg)

    def write_reg(self, reg: int, value):
        if reg == REGISTER_IDS["telemetry_reg"]:
            return self.set_telemetry_registers([int(v) for v in value])
        payload = self._encode_register_write_payload(reg, value)
        self._send_packet(self.PACKET_REGISTER, payload)
        return self._wait_for_register_response(reg)

    # --- telemetry parsing -------------------------------------------------
    def _handle_telem_header(self, payload: bytes):
        if not payload:
            return
        tid = payload[0]
        regs: List[tuple[int, int]] = []
        for idx in range(1, len(payload), 2):
            if idx + 1 >= len(payload):
                break
            motor = payload[idx]
            reg = payload[idx + 1]
            regs.append((motor, reg))
        if regs:
            self.telemetry_headers[tid] = regs

    def _handle_telem_frame(self, payload: bytes) -> Optional[TelemetrySample]:
        if not payload:
            return None
        tid = payload[0]
        reg_map = self.telemetry_headers.get(tid, DEFAULT_TELEM_REGS if tid == 0 else None)
        if not reg_map:
            return None
        data = memoryview(payload[1:])
        idx = 0
        values: Dict[int, Union[float, int, List[float]]] = {}
        for motor_idx, reg_id in reg_map:
            size = self._register_size(reg_id)
            if size is None:
                return None
            if idx + size > len(data):
                return None
            chunk = data[idx : idx + size].tobytes()
            idx += size
            decoded = self._decode_register_payload(reg_id, chunk, multi=True)
            if decoded is None:
                continue
            if reg_id in self._INT_REGS and isinstance(decoded, list) and decoded:
                decoded = int(decoded[0])
            needed = REG_VALUE_FIELDS.get(reg_id, 1)
            if isinstance(decoded, list) and needed == 1 and len(decoded) == 1:
                decoded_val: Union[int, float, List[float]] = decoded[0]
            else:
                decoded_val = decoded
            if motor_idx == 0:
                values[reg_id] = decoded_val
        sample = TelemetrySample(regs=list(reg_map), values=values, timestamp=time.time())
        return sample

    def poll_telemetry(self) -> Optional[TelemetrySample]:
        """Non-blocking poll for telemetry packets. Returns the most recent sample if any."""
        latest: Optional[TelemetrySample] = None
        try:
            avail = self.ser.in_waiting
        except Exception:
            avail = 0
        if avail:
            data = self.ser.read(avail)
            if data:
                self._rx_buf.extend(data)
        while True:
            frame = self._try_extract_frame()
            if not frame:
                break
            pkt_type, payload = frame
            if pkt_type == self.PACKET_TELEM_HEADER:
                self._handle_telem_header(payload)
            elif pkt_type == self.PACKET_TELEM:
                sample = self._handle_telem_frame(payload)
                if sample:
                    latest = sample
        return latest

    def set_telemetry_registers(self, regs: List[int], motor: int = 0):
        payload = bytearray()
        payload.append(REGISTER_IDS["telemetry_reg"])
        payload.append(len(regs))
        for r in regs:
            payload.append(motor)
            payload.append(int(r) & 0xFF)
        self._send_packet(self.PACKET_REGISTER, bytes(payload))
        self._wait_for_register_response(REGISTER_IDS["telemetry_reg"])
        self.telemetry_headers[motor] = [(motor, r) for r in regs]

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
        """Send the BootPacketCommander 'S1' save command and wait for binary s1 response."""
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._send_packet(self.PACKET_SAVE, bytes([1]))
        return self._wait_for_command_response(self.PACKET_SAVE_RESP, expect_code=1, timeout=2.0)

    def run_calibration(self, timeout: float = 10.0) -> Optional[bool]:
        """Send C2 calibration command and wait for binary c2 response."""
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._send_packet(self.PACKET_CAL, bytes([2]))
        return self._wait_for_command_response(self.PACKET_CAL_RESP, expect_code=2, timeout=timeout)


def map_status(val: Optional[int]) -> str:
    if val is None:
        return "n/a"
    return STATUS_NAMES.get(int(val), f"0x{int(val):02X}")


def map_control_mode(val: Optional[int]) -> str:
    if val is None:
        return "n/a"
    return CONTROL_MODE_NAMES.get(int(val), f"0x{int(val):02X}")
