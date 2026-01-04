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

MAX_BINARY_TELEM_PAYLOAD = 250  # RobustBinaryIO LEN includes type+payload+CRC32
MAX_TELEM_REGS = 8  # matches TELEMETRY_MAX_REGISTERS on firmware

READ_REGEX = re.compile(READ_REGEX_PATTERN, re.IGNORECASE)
TELEM_HEADER_REGEX = re.compile(TELEM_HEADER_PATTERN)
TELEM_DATA_REGEX = re.compile(TELEM_DATA_PATTERN)

LOG_LEVEL_NAMES = {
    0: "DEBUG",
    1: "INFO",
    2: "WARN",
    3: "ERROR",
}


@dataclass
class TelemetrySample:
    regs: List[tuple[int, int]]
    values: Dict[int, Union[float, List[float]]]
    timestamp: float = 0.0

class BinaryPacketCommanderClient:
    """
    Binary PacketCommander transport (RobustBinaryIO framing, marker 0xA5).

    Packet layout (RobustBinaryIO):
      [0xA5][LEN][TYPE][PAYLOAD...][CRC32]
        - LEN = len(type + payload + crc32) = 1 + payload_len + 4
        - CRC32 is little-endian CRC-32/MPEG-2 over LEN+TYPE+PAYLOAD
    """

    MARKER = 0xA5
    ESC = 0xDB
    ESC_MARKER = 0xDC
    ESC_ESC = 0xDD
    CRC_SIZE = 4
    PACKET_REGISTER = ord("R")
    PACKET_RESPONSE = ord("r")
    PACKET_TELEM_HEADER = ord("H")
    PACKET_TELEM = ord("T")
    PACKET_SYNC = ord("S")
    PACKET_COMMAND = ord("C")
    PACKET_COMMAND_RESP = ord("c")
    PACKET_LOG = ord("L")

    CMD_WRITE = 0x01
    CMD_CALIBRATE = 0x02
    CMD_BOOTLOADER = 0x03
    CMD_STATUS_OK = 0x00

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
        REGISTER_IDS["telemetry_min_elapsed"],
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
        log_packets: bool = False,
        logger: Optional[Callable[[str], None]] = None,
    ):
        self.ser = ser or serial.Serial(port, baudrate=baud, timeout=timeout)
        self.telemetry_headers: Dict[int, List[tuple[int, int]]] = {0: DEFAULT_TELEM_REGS.copy()}
        self.debug = debug
        self.log_packets = log_packets
        self._log = logger or (lambda msg: print(msg))
        self._rx_buf = bytearray()
        self._rx_pos = 0
        self._parse_state = "idle"
        self._parse_expected_len = 0
        self._parse_buf = bytearray()
        self._parse_esc_pending = False
        self.sync_losses = 0
        self.crc_errors = 0

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    # --- framing helpers ---------------------------------------------------
    @staticmethod
    def _crc32_mpeg2(data: bytes) -> int:
        if not data:
            return 0xFFFFFFFF
        poly = 0x04C11DB7
        crc = 0xFFFFFFFF
        pad_len = (-len(data)) % 4
        if pad_len:
            data = data + (b"\x00" * pad_len)
        for byte in data:
            crc ^= (byte << 24) & 0xFFFFFFFF
            for _ in range(8):
                if crc & 0x80000000:
                    crc = ((crc << 1) & 0xFFFFFFFF) ^ poly
                else:
                    crc = (crc << 1) & 0xFFFFFFFF
        return crc & 0xFFFFFFFF

    @classmethod
    def _escape_bytes(cls, data: bytes) -> bytes:
        out = bytearray()
        for byte in data:
            if byte == cls.MARKER:
                out.append(cls.ESC)
                out.append(cls.ESC_MARKER)
            elif byte == cls.ESC:
                out.append(cls.ESC)
                out.append(cls.ESC_ESC)
            else:
                out.append(byte)
        return bytes(out)

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
        size = len(payload) + 1 + self.CRC_SIZE  # includes packet type + CRC32
        data = bytes([size, pkt_type]) + payload
        crc = self._crc32_mpeg2(data)
        frame = bytes([self.MARKER]) + self._escape_bytes(data + crc.to_bytes(4, "little"))
        if self.debug:
            self._log(f">> type={chr(pkt_type)} size={size} payload_len={len(payload)}")
        self.ser.write(frame)
        try:
            self.ser.flush()
        except Exception:
            pass

    def _try_extract_frame(self) -> Optional[Tuple[int, bytes]]:
        buf = self._rx_buf
        while self._rx_pos < len(buf):
            byte = buf[self._rx_pos]
            self._rx_pos += 1

            if byte == self.MARKER and not self._parse_esc_pending:
                if self._parse_state not in ("idle", "len"):
                    self.sync_losses += 1
                self._parse_state = "len"
                self._parse_buf.clear()
                self._parse_expected_len = 0
                self._parse_esc_pending = False
                continue

            if self._parse_esc_pending:
                self._parse_esc_pending = False
                if byte == self.ESC_MARKER:
                    byte = self.MARKER
                elif byte == self.ESC_ESC:
                    byte = self.ESC
                else:
                    self.sync_losses += 1
                    self._parse_state = "idle"
                    self._parse_buf.clear()
                    continue
            elif byte == self.ESC:
                self._parse_esc_pending = True
                continue

            if self._parse_state == "idle":
                continue

            if self._parse_state == "len":
                self._parse_expected_len = byte
                if self._parse_expected_len < (1 + self.CRC_SIZE) or self._parse_expected_len > 255:
                    self.sync_losses += 1
                    self._parse_state = "idle"
                    self._parse_buf.clear()
                else:
                    self._parse_buf = bytearray([byte])
                    self._parse_state = "data"
                continue

            if self._parse_state == "data":
                self._parse_buf.append(byte)
                if len(self._parse_buf) >= self._parse_expected_len + 1:
                    self._parse_state = "idle"
                    received_crc = int.from_bytes(self._parse_buf[-self.CRC_SIZE :], "little")
                    computed_crc = self._crc32_mpeg2(bytes(self._parse_buf[:-self.CRC_SIZE]))
                    if computed_crc != received_crc:
                        self.crc_errors += 1
                        self._parse_buf.clear()
                        continue
                    pkt_type = self._parse_buf[1]
                    payload_len = self._parse_expected_len - 1 - self.CRC_SIZE
                    payload = bytes(self._parse_buf[2 : 2 + payload_len]) if payload_len > 0 else b""
                    self._parse_buf.clear()
                    if self.debug:
                        self._log(f"<< type={chr(pkt_type)} payload_len={len(payload)}")
                    return pkt_type, payload

        if self._rx_pos >= len(self._rx_buf):
            self._rx_buf.clear()
            self._rx_pos = 0
        return None

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
            elif pkt_type == self.PACKET_LOG:
                self._handle_log(payload)
        return None

    def _wait_for_command_response(self, cmd_id: int, timeout: float = 1.5) -> Optional[bool]:
        deadline = time.time() + timeout
        while time.time() < deadline:
            frame = self._read_frame(deadline - time.time())
            if not frame:
                continue
            pkt_type, payload = frame
            if pkt_type == self.PACKET_COMMAND_RESP and len(payload) >= 2:
                code = payload[0]
                if code != cmd_id:
                    continue
                return payload[1] == self.CMD_STATUS_OK
            if pkt_type == self.PACKET_TELEM_HEADER:
                self._handle_telem_header(payload)
            elif pkt_type == self.PACKET_TELEM:
                self._handle_telem_frame(payload)
            elif pkt_type == self.PACKET_LOG:
                self._handle_log(payload)
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
    
    def set_telemetry_rate_hz(self, hz: float):
        """Set telemetry output rate in Hz using min_elapsed_time (microseconds)."""
        if hz <= 0:
            self.write_reg(REGISTER_IDS["telemetry_min_elapsed"], 0)
            self.write_reg(REG_TELEMETRY_DOWNSAMPLE, 1)
            return
        period_us = int(1_000_000 / hz)
        if period_us < 1:
            period_us = 1
        self.write_reg(REG_TELEMETRY_DOWNSAMPLE, 1)
        self.write_reg(REGISTER_IDS["telemetry_min_elapsed"], period_us)

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
            elif pkt_type == self.PACKET_LOG:
                self._handle_log(payload)
        return latest

    def _handle_log(self, payload: bytes) -> None:
        if not self.log_packets:
            return
        if len(payload) < 2:
            return
        level = payload[0]
        tag_len = payload[1]
        idx = 2
        if idx + tag_len > len(payload):
            return
        tag = payload[idx : idx + tag_len].decode("ascii", errors="replace")
        idx += tag_len
        if idx >= len(payload):
            msg_len = 0
            msg = ""
        else:
            msg_len = payload[idx]
            idx += 1
            if idx + msg_len > len(payload):
                msg = payload[idx:].decode("ascii", errors="replace")
            else:
                msg = payload[idx : idx + msg_len].decode("ascii", errors="replace")
        level_name = LOG_LEVEL_NAMES.get(level, f"L{level}")
        self._log(f"{level_name}/{tag}: {msg}")

    def set_telemetry_registers(self, regs: List[int], motor: int = 0):
        """
        Configure telemetry registers (REG_TELEMETRY_REG) using RobustBinaryIO framing.

        Payload layout expected by SimpleFOC Telemetry::setTelemetryRegisters:
          [reg_id][count][motor,reg]*N
        """
        if len(regs) > MAX_TELEM_REGS:
            raise ValueError(f"Firmware supports at most {MAX_TELEM_REGS} telemetry registers; requested {len(regs)}.")
        est_payload_len = 1 + 1 + (2 * len(regs))  # reg + count + pairs
        if est_payload_len >= MAX_BINARY_TELEM_PAYLOAD:
            raise ValueError(f"Telemetry register payload too large ({est_payload_len} bytes). Reduce register count.")
        payload = bytearray()
        payload.append(REGISTER_IDS["telemetry_reg"])
        payload.append(len(regs))
        for r in regs:
            if r < 0 or r > 255:
                raise ValueError(f"Register id {r} out of byte range (0-255)")
            payload.append(motor)
            payload.append(int(r))
        # RobustBinaryIO packet: [0xA5][LEN][TYPE][PAYLOAD...][CRC32], LEN includes CRC32
        self._send_packet(self.PACKET_REGISTER, bytes(payload))
        resp = self._wait_for_register_response(REGISTER_IDS["telemetry_reg"])
        if resp is None:
            raise RuntimeError("No response to telemetry_reg write (RobustBinaryIO)")
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
        """Send the v2 command packet to persist settings."""
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._send_packet(self.PACKET_COMMAND, bytes([self.CMD_WRITE]))
        return self._wait_for_command_response(self.CMD_WRITE, timeout=2.0)

    def run_calibration(self, timeout: float = 10.0) -> Optional[bool]:
        """Send v2 calibration command and wait for response."""
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._send_packet(self.PACKET_COMMAND, bytes([self.CMD_CALIBRATE]))
        return self._wait_for_command_response(self.CMD_CALIBRATE, timeout=timeout)


def map_status(val: Optional[int]) -> str:
    if val is None:
        return "n/a"
    return STATUS_NAMES.get(int(val), f"0x{int(val):02X}")


def map_control_mode(val: Optional[int]) -> str:
    if val is None:
        return "n/a"
    return CONTROL_MODE_NAMES.get(int(val), f"0x{int(val):02X}")
