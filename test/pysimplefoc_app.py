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
import math
import re
import sys
import time
from dataclasses import dataclass

import serial  # pyserial

# PacketCommander register IDs (SimpleFOCRegisters.h)
REG_TARGET = 0x01
REG_ENABLE = 0x04
REG_CONTROL_MODE = 0x05
REG_ANGLE = 0x09
REG_VELOCITY = 0x11

# MotionControlType values (FOCMotor.h)
MOTION_TORQUE = 0
MOTION_VELOCITY = 1
MOTION_ANGLE = 2
MOTION_VELOCITY_OPENLOOP = 3
MOTION_ANGLE_OPENLOOP = 4

READ_REGEX = re.compile(r"r(?P<reg>\d+)\s*=\s*(?P<val>[-+0-9.eE]+)")


@dataclass
class MotorState:
    control_mode: int = MOTION_VELOCITY
    open_loop: bool = False
    run_velocity: float = 1.0  # rad/s for slow run
    running: bool = False
    running_dir: int = 1


class PacketCommanderClient:
    def __init__(self, port: str, baud: int, timeout: float = 0.3):
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout)

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
        _ = self._read_response(reg, timeout=0.2)


def setup_motor(client: PacketCommanderClient, state: MotorState):
    # Read current control mode to decide open-loop vs closed-loop
    cmode = client.read_reg(REG_CONTROL_MODE)
    if cmode is None:
        cmode = MOTION_VELOCITY
    state.control_mode = int(cmode)
    state.open_loop = state.control_mode in (MOTION_VELOCITY_OPENLOOP, MOTION_ANGLE_OPENLOOP)

    # Enable motor
    client.write_reg(REG_ENABLE, 1)

    # Default to angle control if encoder present; otherwise velocity_openloop
    if not state.open_loop:
        client.write_reg(REG_CONTROL_MODE, MOTION_ANGLE)
        state.control_mode = MOTION_ANGLE
    else:
        client.write_reg(REG_CONTROL_MODE, MOTION_VELOCITY_OPENLOOP)
        state.control_mode = MOTION_VELOCITY_OPENLOOP


def do_full_rotation(client: PacketCommanderClient, state: MotorState, direction: int):
    if state.control_mode == MOTION_ANGLE:
        current = client.read_reg(REG_ANGLE)
        if current is None:
            current = 0.0
        target = current + direction * (2 * math.pi)
        client.write_reg(REG_TARGET, target)
    else:
        # Velocity/open-loop: command velocity for one rotation duration
        speed = max(abs(state.run_velocity), 0.5) * direction
        client.write_reg(REG_TARGET, speed)
        duration = 2 * math.pi / max(abs(speed), 0.1)
        time.sleep(duration)
        client.write_reg(REG_TARGET, 0.0)


def do_step(client: PacketCommanderClient, state: MotorState, direction: int, deg: float = 10.0):
    if state.control_mode == MOTION_ANGLE:
        current = client.read_reg(REG_ANGLE)
        if current is None:
            current = 0.0
        target = current + direction * math.radians(deg)
        client.write_reg(REG_TARGET, target)
    else:
        speed = max(abs(state.run_velocity), 0.5) * direction
        step_rad = math.radians(deg)
        duration = step_rad / max(abs(speed), 0.1)
        client.write_reg(REG_TARGET, speed)
        time.sleep(duration)
        client.write_reg(REG_TARGET, 0.0)


def do_run(client: PacketCommanderClient, state: MotorState, direction: int):
    state.running = True
    state.running_dir = direction
    target = direction * state.run_velocity
    client.write_reg(REG_CONTROL_MODE, MOTION_VELOCITY if not state.open_loop else MOTION_VELOCITY_OPENLOOP)
    client.write_reg(REG_TARGET, target)


def stop_run(client: PacketCommanderClient, state: MotorState):
    state.running = False
    client.write_reg(REG_TARGET, 0.0)


def adjust_speed(client: PacketCommanderClient, state: MotorState, delta: float):
    state.run_velocity = max(0.1, state.run_velocity + delta)
    if state.running:
        client.write_reg(REG_TARGET, state.running_dir * state.run_velocity)


def draw_ui(stdscr, state: MotorState):
    stdscr.erase()
    stdscr.addstr(0, 0, "PySimpleFOC UART test")
    stdscr.addstr(1, 0, "q: quit | f/F: full CW/CCW | s/S: +10/-10 deg | r/R: run fwd/rev | SPACE: stop")
    stdscr.addstr(2, 0, "UP/DOWN: change run speed")
    stdscr.addstr(4, 0, f"Control mode: {'open-loop velocity' if state.open_loop else 'angle'}")
    stdscr.addstr(5, 0, f"Run speed: {state.run_velocity:.2f} rad/s")
    if state.running:
        stdscr.addstr(6, 0, f"Running: {'CW' if state.running_dir > 0 else 'CCW'}")
    else:
        stdscr.addstr(6, 0, "Running: no")
    stdscr.refresh()


def main():
    parser = argparse.ArgumentParser(description="PySimpleFOC PacketCommander test UI")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM0 or COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    client = PacketCommanderClient(args.port, args.baud)
    state = MotorState()
    setup_motor(client, state)

    import curses

    def loop(stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        draw_ui(stdscr, state)
        while True:
            ch = stdscr.getch()
            if ch == -1:
                time.sleep(0.01)
                continue
            if ch in (ord("q"), ord("Q")):
                stop_run(client, state)
                break
            if ch == ord("f"):
                do_full_rotation(client, state, direction=1)
            elif ch == ord("F"):
                do_full_rotation(client, state, direction=-1)
            elif ch == ord("s"):
                do_step(client, state, direction=1)
            elif ch == ord("S"):
                do_step(client, state, direction=-1)
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
            draw_ui(stdscr, state)

    try:
        curses.wrapper(loop)
    finally:
        stop_run(client, state)
        client.close()


if __name__ == "__main__":
    if not sys.platform.startswith("linux") and not sys.platform.startswith("darwin") and not sys.platform.startswith("win"):
        print("Warning: curses UI may not be supported on this platform.")
    main()

