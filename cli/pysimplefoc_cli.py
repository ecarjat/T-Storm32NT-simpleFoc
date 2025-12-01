#!/usr/bin/env python3
"""
Interactive PySimpleFOC PacketCommander client for the T-STorM32 SimpleFOC driver.

Features:
- Full rotation CW/CCW
- 10° step CW/CCW
- Slow run CW/CCW with speed adjust via up/down arrows

It talks the text PacketCommander protocol over UART (PA9/PA10). Works with or without encoder:
- If encoder is available (control mode angle supported), uses angle moves.
- If sensorless (velocity_openloop), uses timed velocity moves.
"""

import argparse
from collections import deque
import math
import re
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Union

import serial  # pyserial

# PacketCommander register IDs (SimpleFOCRegisters.h)
REGISTER_IDS = {
    "status": 0x00,
    "target": 0x01,
    "enable": 0x04,
    "control_mode": 0x05,
    "torque_mode": 0x06,
    "angle": 0x09,
    "velocity": 0x11,
    "telemetry_ctrl": 0x1B,
    "telemetry_downsample": 0x1C,
}
REG_STATUS = REGISTER_IDS["status"]
REG_TARGET = REGISTER_IDS["target"]
REG_ENABLE = REGISTER_IDS["enable"]
REG_CONTROL_MODE = REGISTER_IDS["control_mode"]
REG_ANGLE = REGISTER_IDS["angle"]
REG_VELOCITY = REGISTER_IDS["velocity"]

READ_REGEX = re.compile(r"r(?P<reg>\d+)\s*=\s*(?P<val>[-+0-9.eE]+)")
TELEM_HEADER_REGEX = re.compile(r"H(?P<tid>\d+)=?(?P<body>.*)")
TELEM_DATA_REGEX = re.compile(r"T(?P<tid>\d+)=?(?P<body>.*)")
DEFAULT_TELEM_REGS = [
    (0, REG_TARGET),
    (0, REG_ANGLE),
    (0, REG_VELOCITY),
    (0, REG_ENABLE),
    (0, REG_STATUS),
]
REG_NAME_MAP = {val: name for name, val in REGISTER_IDS.items()}
REG_NAME_MAP.update(
    {
        0x10: "position",  # rotations + angle
        0x12: "sensor_angle",
        0x13: "sensor_mech_angle",
        0x14: "sensor_velocity",
        0x15: "sensor_ts",
        0x52: "velocity_limit",
        0x50: "voltage_limit",
        0x53: "driver_voltage_limit",
        0x55: "driver_voltage_psu",
        0x63: "pole_pairs",
    }
)
REG_VALUE_FIELDS = {
    0x10: 2,  # position register returns rotations + angle rad
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
    CONTROL_MODE_IDS["angle"],
    CONTROL_MODE_IDS["velocity"],
    CONTROL_MODE_IDS["vel_openloop"],
]


def reg_display_name(reg_id: int) -> str:
    return REG_NAME_MAP.get(reg_id, f"reg{reg_id}")


def map_control_mode(val: Optional[int]) -> str:
    if val is None:
        return "n/a"
    return CONTROL_MODE_NAMES.get(int(val), f"0x{int(val):02X}")


def map_torque_mode(val: Optional[int]) -> str:
    if val is None:
        return "n/a"
    return TORQUE_MODE_NAMES.get(int(val), f"0x{int(val):02X}")


def map_status(val: Optional[int]) -> str:
    if val is None:
        return "n/a"
    return STATUS_NAMES.get(int(val), f"0x{int(val):02X}")


@dataclass
class MotorState:
    client: "PacketCommanderClient"
    control_mode: Optional[int] = None  # populated after reading REG_CONTROL_MODE
    open_loop: Optional[bool] = None    # derived from control_mode
    run_velocity: float = 1.0  # rad/s for slow run
    running: bool = False
    running_dir: int = 1
    telemetry: Optional["TelemetrySample"] = None
    control_mode_val: Optional[int] = None
    torque_mode_val: Optional[int] = None
    status_val: Optional[int] = None
    enable_val: Optional[int] = None
    target_val: Optional[float] = None

    def _write_and_confirm(self, reg: int, value):
        """Write register and confirm via echoed response."""
        resp = self.client.write_reg(reg, value)
        return resp if resp is not None else None

    def refresh_status(self):
        cm = self.client.read_reg(REG_CONTROL_MODE)
        if cm is not None:
            self.control_mode = int(cm)
            self.control_mode_val = int(cm)
            self.open_loop = self.control_mode in OPENLOOP_MODES
        tm = self.client.read_reg(REGISTER_IDS["torque_mode"])
        if tm is not None:
            self.torque_mode_val = int(tm)
        st = self.client.read_reg(REG_STATUS)
        if st is not None:
            self.status_val = int(st)
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

    def set_torque_mode(self, mode: int):
        resp = self._write_and_confirm(REGISTER_IDS["torque_mode"], mode)
        if resp is not None:
            self.torque_mode_val = int(resp)
        return resp

    def set_enable(self, enable: bool):
        resp = self._write_and_confirm(REG_ENABLE, 1 if enable else 0)
        if resp is not None:
            self.enable_val = int(resp)
        return resp

    def set_target(self, target: float):
        resp = self._write_and_confirm(REG_TARGET, target)
        if resp is not None:
            self.target_val = float(resp)
        return resp


@dataclass
class TelemetrySample:
    regs: List[tuple[int, int]]
    values: Dict[int, Union[float, List[float]]]
    timestamp: float = 0.0


class PacketCommanderClient:
    def __init__(self, port: str, baud: int, timeout: float = 0.3):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)
        # Default mapping matches firmware telemetry_registers order in comms_streams.cpp
        self.telemetry_headers: Dict[int, List[tuple[int, int]]] = {0: DEFAULT_TELEM_REGS.copy()}

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def _write_line(self, line: str):
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
            m = READ_REGEX.match(line)
            if not m:
                continue
            reg = int(m.group("reg"))
            if reg != expect_reg:
                continue
            try:
                return float(m.group("val"))
            except ValueError:
                return None
        return None

    def read_reg(self, reg: int):
        self._write_line(f"R{reg}")
        return self._read_response(reg)

    def write_reg(self, reg: int, value):
        self._write_line(f"R{reg}={value}")
        # echo is enabled in firmware; optionally read back
        return self._read_response(reg, timeout=0.5)

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
            chunk = values[idx:idx + needed]
            idx += needed
            parsed: List[float] = []
            for raw in chunk:
                try:
                    parsed.append(float(raw))
                except ValueError:
                    parsed.append(float("nan"))
            if motor_idx != 0:
                continue  # CLI only tracks motor 0
            if needed == 1:
                if reg_id in (REG_STATUS, REG_ENABLE, 0x06, 0x05):
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
            # Swallow serial errors in telemetry path to avoid breaking UI
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
        line = "R26=" + ",".join(parts)  # 0x1A = 26 decimal
        self._write_line(line)
        # Update local mapping and ask MCU to re-send header
        self.telemetry_headers[motor] = [(motor, r) for r in regs]
        self._write_line("R26")  # reading REG_TELEMETRY_REG triggers header resend

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


def setup_motor(client: PacketCommanderClient, state: MotorState):
    # Read current control mode to decide open-loop vs closed-loop
    state.refresh_status()
    if state.control_mode is None:
        state.set_control_mode(CONTROL_MODE_IDS["velocity"])

    # Enable motor
    state.set_enable(True)

    # Default to angle control if encoder present; otherwise velocity_openloop
    if state.open_loop is False:
        state.set_control_mode(CONTROL_MODE_IDS["angle"])
    else:
        state.set_control_mode(CONTROL_MODE_IDS["vel_openloop"])


def do_full_rotation(client: PacketCommanderClient, state: MotorState, direction: int):
    cm = state.control_mode
    if cm is None:
        state.refresh_status()
        cm = state.control_mode
    if cm == CONTROL_MODE_IDS["angle"]:
        current = client.read_reg(REG_ANGLE)
        if current is None:
            current = 0.0
        target = current + direction * (2 * math.pi)
        state.set_target(target)
    else:
        # Velocity/open-loop: command velocity for one rotation duration
        speed = max(abs(state.run_velocity), 0.5) * direction
        state.set_target(speed)
        duration = 2 * math.pi / max(abs(speed), 0.1)
        time.sleep(duration)
        state.set_target(0.0)


def do_step(client: PacketCommanderClient, state: MotorState, direction: int, deg: float = 10.0):
    cm = state.control_mode
    if cm is None:
        state.refresh_status()
        cm = state.control_mode
    if cm == CONTROL_MODE_IDS["angle"]:
        current = client.read_reg(REG_ANGLE)
        if current is None:
            current = 0.0
        target = current + direction * math.radians(deg)
        state.set_target(target)
    else:
        speed = max(abs(state.run_velocity), 0.5) * direction
        step_rad = math.radians(deg)
        duration = step_rad / max(abs(speed), 0.1)
        state.set_target(speed)
        time.sleep(duration)
        state.set_target(0.0)
    state.running = False


def do_run(client: PacketCommanderClient, state: MotorState, direction: int):
    state.running = True
    state.running_dir = direction
    target = direction * state.run_velocity
    desired_mode = CONTROL_MODE_IDS["vel_openloop"] if state.open_loop else CONTROL_MODE_IDS["velocity"]
    state.set_control_mode(desired_mode)
    state.set_target(target)


def stop_run(client: PacketCommanderClient, state: MotorState):
    state.running = False
    state.set_target(0.0)


def adjust_speed(client: PacketCommanderClient, state: MotorState, delta: float):
    state.run_velocity = max(0.1, state.run_velocity + delta)
    if state.running:
        state.set_target(state.running_dir * state.run_velocity)


def draw_ui(stdscr, state: MotorState):
    stdscr.erase()
    stdscr.addstr(0, 0, "PySimpleFOC UART test")
    stdscr.addstr(1, 0, "q: quit | f/F: full CW/CCW | s/S: +10/-10 deg | r/R: run fwd/rev | SPACE: stop | m: cycle mode")
    stdscr.addstr(2, 0, "UP/DOWN: change run speed")
    cmode_display = state.control_mode_val if state.control_mode_val is not None else state.control_mode
    open_loop_flag = None
    if cmode_display is not None:
        open_loop_flag = cmode_display in OPENLOOP_MODES
    elif state.open_loop is not None:
        open_loop_flag = state.open_loop
    cmode_name = map_control_mode(cmode_display) if cmode_display is not None else "unknown"
    open_loop_txt = "yes" if open_loop_flag else ("no" if open_loop_flag is not None else "unknown")
    stdscr.addstr(4, 0, f"Control mode: {cmode_name} (open-loop: {open_loop_txt})")
    stdscr.addstr(5, 0, f"Run speed: {state.run_velocity:.2f} rad/s")
    if state.running:
        stdscr.addstr(6, 0, f"Running: {'CW' if state.running_dir > 0 else 'CCW'}")
    else:
        stdscr.addstr(6, 0, "Running: no")
    stdscr.addstr(7, 0, f"Status regs: control {map_control_mode(state.control_mode_val)}, torque {map_torque_mode(state.torque_mode_val)}, status {map_status(state.status_val)}")
    line = 8
    if state.telemetry:
        t = state.telemetry
        stdscr.addstr(line, 0, f"Telemetry (age {(time.time() - t.timestamp):.1f}s):")
        line += 1
        for (motor_idx, reg_id) in t.regs:
            if motor_idx != 0:
                continue
            name = REG_NAME_MAP.get(reg_id, f"reg{reg_id}")
            val = t.values.get(reg_id)
            if isinstance(val, list):
                disp = "/".join(f"{v:.4f}" for v in val)
            elif reg_id == REG_STATUS and isinstance(val, (int, float)):
                disp = STATUS_NAMES.get(int(val), f"0x{int(val):02X}")
            elif reg_id == REGISTER_IDS["torque_mode"] and isinstance(val, (int, float)):
                disp = TORQUE_MODE_NAMES.get(int(val), f"0x{int(val):02X}")
            elif reg_id == REG_CONTROL_MODE and isinstance(val, (int, float)):
                disp = CONTROL_MODE_NAMES.get(int(val), f"0x{int(val):02X}")
            elif isinstance(val, float):
                disp = f"{val:.4f}"
            else:
                disp = "n/a"
            stdscr.addstr(line, 0, f"  {name}: {disp}")
            line += 1
    else:
        stdscr.addstr(line, 0, "Telemetry: none received yet")
    stdscr.refresh()


def main():
    parser = argparse.ArgumentParser(description="PySimpleFOC PacketCommander test UI")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM0 or COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    client = PacketCommanderClient(args.port, args.baud)
    state = MotorState(client)
    setup_motor(client, state)

    import curses

    def run_test_ui():
        def refresh_status_regs():
            for reg, attr in (
                (REG_CONTROL_MODE, "control_mode_val"),
                (REGISTER_IDS["torque_mode"], "torque_mode_val"),
                (REG_STATUS, "status_val"),
            ):
                val = client.read_reg(reg)
                if val is not None:
                    setattr(state, attr, int(val))
                    if reg == REG_CONTROL_MODE:
                        state.control_mode = int(val)
                        state.open_loop = state.control_mode in OPENLOOP_MODES

        def loop(stdscr):
            curses.curs_set(0)
            stdscr.nodelay(True)
            refresh_status_regs()
            draw_ui(stdscr, state)
            while True:
                telem = client.poll_telemetry()
                if telem:
                    state.telemetry = telem
                ch = stdscr.getch()
                if ch == -1:
                    draw_ui(stdscr, state)
                    time.sleep(0.01)
                    continue
                if ch in (ord("q"), ord("Q")):
                    stop_run(client, state)
                    break  # return to main menu
                if ch == ord("f"):
                    state.running = True
                    state.running_dir = 1
                    draw_ui(stdscr, state)
                    do_full_rotation(client, state, direction=1)
                    state.running = False
                    draw_ui(stdscr, state)
                elif ch == ord("F"):
                    state.running = True
                    state.running_dir = -1
                    draw_ui(stdscr, state)
                    do_full_rotation(client, state, direction=-1)
                    state.running = False
                    draw_ui(stdscr, state)
                elif ch == ord("s"):
                    state.running = True
                    state.running_dir = 1
                    draw_ui(stdscr, state)
                    do_step(client, state, direction=1)
                    state.running = False
                    draw_ui(stdscr, state)
                elif ch == ord("S"):
                    state.running = True
                    state.running_dir = -1
                    draw_ui(stdscr, state)
                    do_step(client, state, direction=-1)
                    state.running = False
                    draw_ui(stdscr, state)
                elif ch == ord("r"):
                    do_run(client, state, direction=1)
                elif ch == ord("R"):
                    do_run(client, state, direction=-1)
                elif ch == curses.KEY_UP:
                    adjust_speed(client, state, 0.2)
                elif ch == curses.KEY_DOWN:
                    adjust_speed(client, state, -0.2)
                elif ch == ord(" "):
                    stop_run(client, state)
                elif ch == ord("m"):
                    # cycle through a sane subset of control modes
                    current = state.control_mode if state.control_mode is not None else CONTROL_MODE_SEQUENCE[0]
                    try:
                        idx = CONTROL_MODE_SEQUENCE.index(current)
                        next_mode = CONTROL_MODE_SEQUENCE[(idx + 1) % len(CONTROL_MODE_SEQUENCE)]
                    except ValueError:
                        next_mode = CONTROL_MODE_SEQUENCE[0]
                    state.set_control_mode(next_mode)
                    # refresh derived fields and redraw
                    state.refresh_status()
                    draw_ui(stdscr, state)
                draw_ui(stdscr, state)

        curses.wrapper(loop)

    def settings_menu():
        settings_items = [
            (
                "Pole pairs",
                0x63,
                "Number of electrical pole pairs (integer).",
                "Motor pole pairs (electrical) used to convert between electrical and mechanical angle. Must match the motor; wrong value breaks commutation.",
            ),
            (
                "Voltage limit",
                0x50,
                "Motion control voltage limit (V).",
                "Motor-level voltage_limit used by SimpleFOC when commanding torque/velocity/angle. Lower to reduce torque/heat; raise carefully.",
            ),
            (
                "Velocity limit",
                0x52,
                "Max allowed shaft velocity (rad/s).",
                "Mechanical velocity clamp in rad/s for motion control. Protects the motor from over-speed; controller will not command faster than this.",
            ),
            (
                "Driver voltage limit",
                0x53,
                "Driver voltage limit (V).",
                "Driver voltage_limit (also mirrored to motor.voltage_limit). Typically same or slightly below supply. Keep conservative for safety.",
            ),
            (
                "Driver PSU voltage",
                0x55,
                "Supply voltage of the driver (V).",
                "Actual supply voltage powering the DRV8313 (voltage_power_supply). Set to your board supply so FOC scales outputs correctly.",
            ),
            (
                "Phase resistance",
                0x64,
                "Phase-to-phase resistance (Ohms).",
                "Motor phase resistance in Ohms. Used for monitoring/estimations; optional but improves telemetry accuracy.",
            ),
            (
                "KV rating",
                0x65,
                "Motor KV (RPM/V).",
                "Motor KV in RPM/V. Informational, used by some estimators; does not directly limit outputs but should be accurate if known.",
            ),
            (
                "Vel PID P",
                0x30,
                "Velocity loop P gain.",
                "Proportional gain of the velocity PID (PID_velocity.P). Too high causes oscillation; too low feels sluggish.",
            ),
            (
                "Vel PID I",
                0x31,
                "Velocity loop I gain.",
                "Integral gain of the velocity PID (PID_velocity.I). Removes steady-state error; too high causes windup/oscillation.",
            ),
            (
                "Vel PID D",
                0x32,
                "Velocity loop D gain.",
                "Derivative gain of the velocity PID (PID_velocity.D). Dampens fast changes/noise; often kept small or zero.",
            ),
            (
                "Vel LPF Tf",
                0x35,
                "Velocity LPF time constant (s).",
                "Low-pass filter time constant for velocity (LPF_velocity.Tf). Larger = more smoothing/lag; smaller = more responsive/noisy.",
            ),
        ]
        while True:
            print("\nSettings:")
            for idx, (label, reg, short, _) in enumerate(settings_items, start=1):
                current = client.read_reg(reg)
                disp = f"{current:.4f}" if current is not None else "n/a"
                print(f"  {idx}) {label}: {disp}  - {short}")
            print("  s) Save to flash (S1)")
            print("  q) Back to main menu")
            choice = input("Select item to edit (number), 's' to save, or 'q': ").strip()
            if choice.lower() == "q":
                break
            if choice.lower() == "s":
                res = client.save_settings()
                if res is True:
                    print("Save acknowledged (SAVE_OK).")
                elif res is False:
                    print("Save failed (SAVE_ERR).")
                else:
                    print("No save response received.")
                continue
            try:
                sel = int(choice)
            except ValueError:
                continue
            if sel < 1 or sel > len(settings_items):
                continue
            label, reg, _, long_help = settings_items[sel - 1]
            print(f"\n{label}: {long_help}")
            new_val = input(f"Enter new value for {label}: ").strip()
            try:
                val = float(new_val)
            except ValueError:
                print("Invalid number.")
                continue
            client.write_reg(reg, val)
            print(f"Set {label} to {val}")

    def telemetry_menu():
        available_regs = [
            ("Target", REG_TARGET, "Commanded target (float)"),
            ("Angle", REG_ANGLE, "Shaft angle (rad)"),
            ("Position", 0x10, "Full rotations + angle (int32 + rad)"),
            ("Velocity", REG_VELOCITY, "Shaft velocity (rad/s)"),
            ("Sensor angle", 0x12, "Sensor electrical angle (rad)"),
            ("Sensor mechanical angle", 0x13, "Sensor mechanical angle (rad)"),
            ("Sensor velocity", 0x14, "Sensor velocity (rad/s)"),
            ("Sensor timestamp", 0x15, "Sensor timestamp (uint32)"),
            ("Enable", REG_ENABLE, "Motor enable flag (0/1)"),
            ("Status", REG_STATUS, "Motor status byte"),
            ("Torque mode", REGISTER_IDS["torque_mode"], "Torque control mode"),
            ("Control mode", REG_CONTROL_MODE, "Motion control mode"),
            ("Velocity limit", 0x52, "Velocity limit (rad/s)"),
            ("Voltage limit", 0x50, "Motor voltage_limit (V)"),
            ("Driver voltage limit", 0x53, "Driver voltage_limit (V)"),
            ("Driver PSU voltage", 0x55, "Driver supply voltage (V)"),
            ("Pole pairs", 0x63, "Pole pairs (int)"),
        ]
        while True:
            current = [reg for motor, reg in client.telemetry_headers.get(0, DEFAULT_TELEM_REGS) if motor == 0]
            print("\nTelemetry registers (motor 0):")
            print("  Current:", ", ".join([REG_NAME_MAP.get(r, str(r)) for r in current]) or "none")
            try:
                ctrl = client.read_reg(REGISTER_IDS["telemetry_ctrl"])
            except Exception:
                ctrl = None
            try:
                downsample_val = client.read_reg(REGISTER_IDS["telemetry_downsample"])
            except Exception:
                downsample_val = None
            if ctrl is not None:
                print(f"  Telemetry controller index (REG_TELEMETRY_CTRL): {int(ctrl)}")
            if downsample_val is not None:
                print(f"  Telemetry downsample (REG_TELEMETRY_DOWNSAMPLE): {int(downsample_val)} (0 disables)")
            print("\nAvailable choices:")
            for idx, (label, reg, desc) in enumerate(available_regs, start=1):
                mark = "*" if reg in current else " "
                print(f"  {idx}) [{mark}] {label} (reg {reg}) - {desc}")
            print("Enter comma-separated numbers to set the list in order, 'cX' to set telemetry controller (e.g. c0), 'dN' to set downsample (e.g. d50), or 'q' to return.")
            choice = input("Selection: ").strip().lower()
            if choice == "q":
                break
            if choice.startswith("c") and len(choice) > 1:
                try:
                    ctrl_val = int(choice[1:])
                    client.write_reg(REGISTER_IDS["telemetry_ctrl"], ctrl_val)
                    print(f"Set REG_TELEMETRY_CTRL to {ctrl_val}")
                except ValueError:
                    print("Invalid controller index.")
                continue
            if choice.startswith("d") and len(choice) > 1:
                try:
                    ds_val = int(choice[1:])
                    client.write_reg(REGISTER_IDS["telemetry_downsample"], ds_val)
                    print(f"Set REG_TELEMETRY_DOWNSAMPLE to {ds_val}")
                except ValueError:
                    print("Invalid downsample value.")
                continue
            if not choice:
                continue
            try:
                indices = [int(tok) for tok in choice.replace(" ", "").split(",") if tok]
            except ValueError:
                print("Invalid input.")
                continue
            if any(i < 1 or i > len(available_regs) for i in indices):
                print("Selection out of range.")
                continue
            regs = [available_regs[i - 1][1] for i in indices]
            client.set_telemetry_registers(regs)
            state.telemetry = None  # reset display until new data arrives
            print("Updated telemetry registers.")

    def plot_menu():
        # Live plot using currently configured telemetry registers (motor 0), scalar-only.
        reg_map = [reg for motor, reg in client.telemetry_headers.get(0, DEFAULT_TELEM_REGS) if motor == 0]
        scalars = []
        seen = set()
        for r in reg_map:
            if REG_VALUE_FIELDS.get(r, 1) != 1:
                continue
            if r in seen:
                continue
            scalars.append(r)
            seen.add(r)
        if not scalars:
            print("No scalar telemetry registers configured. Add some in the Telemetry menu first.")
            return
        print("\nSelect registers to plot live (comma-separated indices, blank = all):")
        for idx, reg in enumerate(scalars, start=1):
            print(f"  {idx}) {reg_display_name(reg)} (reg {reg})")
        choice = input("Selection: ").strip()
        if choice:
            try:
                idxs = [int(tok) for tok in choice.replace(" ", "").split(",") if tok]
            except ValueError:
                print("Invalid selection.")
                return
            regs_sel = []
            for i in idxs:
                if 1 <= i <= len(scalars):
                    regs_sel.append(scalars[i - 1])
            if not regs_sel:
                print("No valid registers chosen.")
                return
        else:
            regs_sel = scalars
        try:
            window = float(input("Display window (seconds, default 10): ").strip() or "10")
        except ValueError:
            print("Invalid window.")
            return
        try:
            import matplotlib.pyplot as plt  # type: ignore
            from matplotlib.animation import FuncAnimation  # type: ignore
        except Exception as e:
            print("matplotlib is required for plotting. Install with `pip install matplotlib`.")
            print(f"Import error: {e}")
            return

        print("Starting live plot. Close the plot window to stop.")
        times = {r: deque(maxlen=5000) for r in regs_sel}
        values = {r: deque(maxlen=5000) for r in regs_sel}
        t0 = time.time()

        fig, axes = plt.subplots(len(regs_sel), 1, sharex=True, figsize=(8, 2 * len(regs_sel)))
        if not isinstance(axes, (list, tuple)):
            axes = [axes]
        lines = {}
        axis_map = {}
        for ax, r in zip(axes, regs_sel):
            (line,) = ax.plot([], [], label=reg_display_name(r))
            lines[r] = line
            axis_map[r] = ax
            ax.set_ylabel(reg_display_name(r))
            ax.legend(loc="upper right")
            ax.grid(True)
        axes[-1].set_xlabel("Time (s)")
        fig.tight_layout()

        def update(frame):
            telem = client.poll_telemetry()
            now = time.time()
            if telem:
                for r in regs_sel:
                    val = telem.values.get(r)
                    if isinstance(val, (float, int)):
                        t_rel = telem.timestamp - t0
                        times[r].append(t_rel)
                        values[r].append(float(val))
            for r in regs_sel:
                # Trim to window
                while times[r] and (now - t0) - times[r][0] > window:
                    times[r].popleft()
                    values[r].popleft()
                lines[r].set_data(list(times[r]), list(values[r]))
                if times[r]:
                    axis_map[r].set_xlim(max(0, times[r][0]), times[r][-1] if times[r][-1] > window else window)
                    vmin = min(values[r])
                    vmax = max(values[r])
                    if vmin == vmax:
                        vmin -= 0.1
                        vmax += 0.1
                    axis_map[r].set_ylim(vmin, vmax)
            return list(lines.values())

        anim = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
        try:
            plt.show()
        finally:
            if getattr(anim, "event_source", None):
                anim.event_source.stop()
            plt.close(fig)

    try:
        while True:
            print("\n=== Main Menu ===")
            print("1) Settings")
            print("2) Test")
            print("3) Telemetry")
            print("4) Live plot telemetry")
            print("q) Quit")
            choice = input("Select option: ").strip().lower()
            if choice == "1":
                settings_menu()
            elif choice == "2":
                run_test_ui()
            elif choice == "3":
                telemetry_menu()
            elif choice == "4":
                plot_menu()
            elif choice == "q":
                break
    finally:
        stop_run(client, state)
        client.close()


if __name__ == "__main__":
    if not sys.platform.startswith("linux") and not sys.platform.startswith("darwin") and not sys.platform.startswith("win"):
        print("Warning: curses UI may not be supported on this platform.")
    main()
