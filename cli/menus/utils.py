from __future__ import annotations

import time
import math
from typing import Optional

from pysfoc import (  # type: ignore[import-not-found]
    PacketCommanderClient,
    MotorState,
    CONTROL_MODE_IDS,
    CONTROL_MODE_NAMES,
)
from pysfoc.constants import (  # type: ignore[import-not-found]
    CONTROL_MODE_SEQUENCE,
    REG_ANGLE,
    REG_CONTROL_MODE,
    REG_STATUS,
    REG_TARGET,
    REGISTER_IDS,
    REG_VELOCITY,
    REG_POSITION,
    REG_VEL_PID_P,
    REG_VEL_PID_I,
    REG_VEL_PID_D,
    REG_VEL_PID_LIMIT,
)
from pysfoc.constants import REG_NAME_MAP, TORQUE_MODE_NAMES, STATUS_NAMES  # type: ignore[import-not-found]


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


def draw_ui(stdscr, state: MotorState, message: str = "") -> None:
    def _as_int(val: object):
        return int(val) if isinstance(val, (int, float)) else None

    stdscr.erase()
    # Top bar
    cmode_display = state.control_mode_val if state.control_mode_val is not None else state.control_mode
    cmode_name = CONTROL_MODE_NAMES.get(cmode_display, str(cmode_display)) if cmode_display is not None else "unknown"
    enabled_txt = "ENABLED" if state.enable_val else "disabled"
    dir_txt = "CW" if state.running_dir > 0 else ("CCW" if state.running_dir < 0 else "stop")
    stdscr.addstr(0, 0, f"Mode: {cmode_name} | Motor: {enabled_txt} | Target: {state.target:.3f} | Dir: {dir_txt}")

    # Keybindings
    stdscr.addstr(1, 0, "SPACE enable/disable | m cycle mode | ↑/↓ target | PgUp/PgDn big step | f/F full rot | s/S step | r/R run | g plot | q quit")

    # Telemetry pane
    line = 3
    if state.telemetry:
        t = state.telemetry
        age = (time.time() - t.timestamp)
        stdscr.addstr(line, 0, f"Telemetry age {age:.1f}s")
        line += 1
        tgt = t.values.get(REG_TARGET)
        vel = t.values.get(REG_VELOCITY)
        ang = t.values.get(REG_ANGLE)
        pos = t.values.get(REG_POSITION)
        stdscr.addstr(
            line,
            0,
            f"  Target: {tgt!s:>10}  Velocity: {vel!s:>10}  Angle: {ang!s:>10}  Position: {pos!s:>10}",
        )
        line += 1
        en = t.values.get(REGISTER_IDS["enable"])
        status = t.values.get(REG_STATUS)
        torque = t.values.get(REGISTER_IDS["torque_mode"])
        cmod = t.values.get(REG_CONTROL_MODE)
        # Fallback to last known state values if telemetry missing
        if en is None and state.enable_val is not None:
            en = state.enable_val
        if status is None and state.status_val is not None:
            status = state.status_val
        if torque is None and state.torque_mode_val is not None:
            torque = state.torque_mode_val
        if cmod is None and state.control_mode_val is not None:
            cmod = state.control_mode_val
        status_int = _as_int(status)
        torque_int = _as_int(torque)
        cmod_int = _as_int(cmod)
        status_txt = STATUS_NAMES.get(status_int, status_int) if status_int is not None else "n/a"
        torque_txt = TORQUE_MODE_NAMES.get(torque_int, torque_int) if torque_int is not None else "n/a"
        cmod_txt = CONTROL_MODE_NAMES.get(cmod_int, cmod_int) if cmod_int is not None else "n/a"
        en_txt = "ON" if en else "OFF"
        stdscr.addstr(
            line,
            0,
            f"  Enable: {en_txt:>6}  Status: {status_txt!s:>10}  Torque: {torque_txt!s:>10}  Control: {cmod_txt!s:>10}",
        )
        line += 1
        vlim = state.client.read_reg(0x50)
        vellim = state.client.read_reg(0x52)
        pidp = state.client.read_reg(REG_VEL_PID_P)
        pidi = state.client.read_reg(REG_VEL_PID_I)
        pidd = state.client.read_reg(REG_VEL_PID_D)
        pidlim = state.client.read_reg(REG_VEL_PID_LIMIT)
        stdscr.addstr(line, 0, f"  V_limit: {vlim!s:>10}  Vel_limit: {vellim!s:>10}")
        line += 1
        stdscr.addstr(
            line, 0, f"  PID P: {pidp!s:>8}  I: {pidi!s:>8}  D: {pidd!s:>8}  Limit: {pidlim!s:>8}"
        )
        line += 1
    else:
        stdscr.addstr(line, 0, "Telemetry: none received yet")
        line += 1

    if message:
        stdscr.addstr(line + 1, 0, f"Status: {message}")
    stdscr.refresh()
