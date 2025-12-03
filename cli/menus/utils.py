from __future__ import annotations

import time
import math
from typing import Optional

from pysfoc import (
    PacketCommanderClient,  # type: ignore[import-not-found]
    MotorState,
    CONTROL_MODE_IDS,
)
from pysfoc.constants import CONTROL_MODE_SEQUENCE, REG_ANGLE, REG_STATUS, REG_TARGET, REGISTER_IDS  # type: ignore[import-not-found]
from pysfoc.constants import REG_NAME_MAP, TORQUE_MODE_NAMES  # type: ignore[import-not-found]


def reg_display_name(reg_id: int) -> str:
    return REG_NAME_MAP.get(reg_id, f"reg{reg_id}")


def map_torque_mode(val: Optional[int]) -> str:
    if val is None:
        return "n/a"
    return TORQUE_MODE_NAMES.get(int(val), f"0x{int(val):02X}")


def setup_motor(client: PacketCommanderClient, state: MotorState) -> None:
    state.refresh_status()
    if state.control_mode is None:
        state.set_control_mode(CONTROL_MODE_IDS["velocity"])
    state.set_enable(False)
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
        speed = max(abs(state.target), 0.5) * direction
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
        speed = max(abs(state.target), 0.5) * direction
        step_rad = math.radians(deg)
        duration = step_rad / max(abs(speed), 0.1)
        state.set_target(speed)
        time.sleep(duration)
        state.set_target(0.0)
    state.running = False


def do_run(client: PacketCommanderClient, state: MotorState, direction: int):
    state.running = True
    state.running_dir = direction
    target = direction * state.target
    state.set_target(target)


def stop_run(client: PacketCommanderClient, state: MotorState):
    state.running = False
    state.set_target(0.0)


def adjust_target(client: PacketCommanderClient, state: MotorState, delta: float):
    state.target = max(0.0, state.target + delta)
    if state.running:
        state.set_target(state.running_dir * state.target)


def draw_ui(stdscr, state: MotorState) -> None:
    stdscr.erase()
    stdscr.addstr(0, 0, "PySimpleFOC UART test")
    stdscr.addstr(1, 0, "q: quit | f/F: full CW/CCW | s/S: +10/-10 deg | r/R: run fwd/rev | SPACE: stop | m: cycle mode")
    stdscr.addstr(2, 0, "UP/DOWN: change target")
    cmode_display = state.control_mode_val if state.control_mode_val is not None else state.control_mode
    open_loop_flag = None
    if cmode_display is not None:
        open_loop_flag = cmode_display in CONTROL_MODE_SEQUENCE[-2:]
    elif state.open_loop is not None:
        open_loop_flag = state.open_loop
    cmode_name = state.control_mode and str(state.control_mode) or "unknown"
    open_loop_txt = "yes" if open_loop_flag else ("no" if open_loop_flag is not None else "unknown")
    stdscr.addstr(4, 0, f"Control mode: {cmode_name} (open-loop: {open_loop_txt})")
    stdscr.addstr(5, 0, f"Target: {state.target:.2f}")
    if state.running:
        stdscr.addstr(6, 0, f"Running: {'CW' if state.running_dir > 0 else 'CCW'}")
    else:
        stdscr.addstr(6, 0, "Running: no")
    stdscr.addstr(7, 0, f"Status regs: control {state.control_mode_val}, torque {state.torque_mode_val}, status {state.status_val}")
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
                disp = str(int(val))
            elif reg_id == REGISTER_IDS["torque_mode"] and isinstance(val, (int, float)):
                disp = TORQUE_MODE_NAMES.get(int(val), f"0x{int(val):02X}")
            elif reg_id == REG_CONTROL_MODE and isinstance(val, (int, float)):
                disp = CONTROL_MODE_SEQUENCE[int(val)] if int(val) < len(CONTROL_MODE_SEQUENCE) else str(val)
            elif isinstance(val, float):
                disp = f"{val:.4f}"
            else:
                disp = "n/a"
            stdscr.addstr(line, 0, f"  {name}: {disp}")
            line += 1
    else:
        stdscr.addstr(line, 0, "Telemetry: none received yet")
    stdscr.refresh()
