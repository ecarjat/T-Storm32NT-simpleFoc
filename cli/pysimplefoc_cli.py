#!/usr/bin/env python3
"""
Interactive PySimpleFOC PacketCommander client for the T-STorM32 SimpleFOC driver.

Features:
- Full rotation CW/CCW
- 10° step CW/CCW
- Slow run CW/CCW with target adjust via up/down arrows
- Velocity PID tuning helper (P/p I/i D/d adjust, live target/velocity graph)

It talks the RobustBinaryIO PacketCommander protocol over UART (PA9/PA10). Works with or without encoder:
- If encoder is available (control mode angle supported), uses angle moves.
- If sensorless (velocity_openloop), uses timed velocity moves.
"""

import argparse
from collections import deque
import math
import sys
import time
from typing import Dict, List, Optional, Union

from pysfoc import (  # type: ignore[import-not-found]
    BinaryPacketCommanderClient,
    MotorState,
    TelemetrySample,
    CONTROL_MODE_IDS,
    CONTROL_MODE_NAMES,
    OPENLOOP_MODES,
    REG_CONTROL_MODE,
    REG_ENABLE,
    REG_TARGET,
    REG_VELOCITY,
    REG_VEL_PID_P,
    REG_VEL_PID_I,
    REG_VEL_PID_D,
    REG_VEL_PID_LIMIT,
    map_control_mode,
    map_status,
)
from pysfoc.constants import (  # type: ignore[import-not-found]
    CONTROL_MODE_SEQUENCE,
    DEFAULT_TELEM_REGS,
    REG_ANGLE,
    REG_NAME_MAP,
    REG_POSITION,
    REG_SENSOR_MECH_ANGLE,
    REG_STATUS,
    REG_TELEMETRY_CTRL,
    REG_TELEMETRY_DOWNSAMPLE,
    REG_VALUE_FIELDS,
    STATUS_NAMES,
    REGISTER_IDS,
    TORQUE_MODE_NAMES,
)
from menus import (  # type: ignore[import-not-found]
    pid_tune_mode,
    sensor_calibration_menu,
    telemetry_menu as telemetry_menu_handler,
    plot_menu as plot_menu_handler,
    sensor_plot_mode as sensor_plot_handler,
    settings_menu as settings_menu_handler,
    run_test_ui as run_test_ui_handler,
    setup_motor,
)


def reg_display_name(reg_id: int) -> str:
    return REG_NAME_MAP.get(reg_id, f"reg{reg_id}")


def map_torque_mode(val: Optional[int]) -> str:
    if val is None:
        return "n/a"
    return TORQUE_MODE_NAMES.get(int(val), f"0x{int(val):02X}")


# def setup_motor(client: PacketCommanderClient, state: MotorState) -> None:
#     # Read current control mode to decide open-loop vs closed-loop
#     state.refresh_status()
#     if state.control_mode is None:
#         state.set_control_mode(CONTROL_MODE_IDS["velocity"])

#     # Ensure motor stays disabled on startup; menus/actions will enable explicitly.
#     state.set_enable(False)

#     # Default to angle control if encoder present; otherwise velocity_openloop
#     if state.open_loop is False:
#         state.set_control_mode(CONTROL_MODE_IDS["angle"])
#     else:
#         state.set_control_mode(CONTROL_MODE_IDS["vel_openloop"])


def do_full_rotation(client: BinaryPacketCommanderClient, state: MotorState, direction: int):
    cm = state.control_mode
    if cm is None:
        state.refresh_status()
        cm = state.control_mode
    if cm == CONTROL_MODE_IDS["angle"]:
        current = client.read_reg(REG_ANGLE)
        if current is None:
            current = 0.0
        target = current + direction * (2 * math.pi)
        state.set_target(target, track_command=True)
    else:
        # Velocity/open-loop: command velocity for one rotation duration
        speed = max(abs(state.commanded_target), 0.5) * direction
        state.set_target(speed)
        duration = 2 * math.pi / max(abs(speed), 0.1)
        time.sleep(duration)
        state.set_target(0.0)


def do_step(client: BinaryPacketCommanderClient, state: MotorState, direction: int, deg: float = 10.0):
    cm = state.control_mode
    if cm is None:
        state.refresh_status()
        cm = state.control_mode
    if cm == CONTROL_MODE_IDS["angle"]:
        current = client.read_reg(REG_ANGLE)
        if current is None:
            current = 0.0
        target = current + direction * math.radians(deg)
        state.set_target(target, track_command=True)
    else:
        speed = max(abs(state.commanded_target), 0.5) * direction
        step_rad = math.radians(deg)
        duration = step_rad / max(abs(speed), 0.1)
        state.set_target(speed)
        time.sleep(duration)
        state.set_target(0.0)
    state.running = False


def do_run(client: BinaryPacketCommanderClient, state: MotorState, direction: int):
    state.running = True
    state.running_dir = direction
    target = direction * state.commanded_target
    state.set_target(target)


def stop_run(client: BinaryPacketCommanderClient, state: MotorState):
    state.running = False
    state.set_target(0.0)


def adjust_target(client: BinaryPacketCommanderClient, state: MotorState, delta: float):
    state.commanded_target = max(0.0, state.commanded_target + delta)
    if state.running:
        state.set_target(state.running_dir * state.commanded_target)


def draw_ui(stdscr, state: MotorState) -> None:
    stdscr.erase()
    stdscr.addstr(0, 0, "PySimpleFOC UART test")
    stdscr.addstr(1, 0, "q: quit | f/F: full CW/CCW | s/S: +10/-10 deg | r/R: run fwd/rev | SPACE: stop | m: cycle mode")
    stdscr.addstr(2, 0, "UP/DOWN: change target")
    cmode_display = state.control_mode_val if state.control_mode_val is not None else state.control_mode
    open_loop_flag = None
    if cmode_display is not None:
        open_loop_flag = cmode_display in OPENLOOP_MODES
    elif state.open_loop is not None:
        open_loop_flag = state.open_loop
    cmode_name = map_control_mode(cmode_display) if cmode_display is not None else "unknown"
    open_loop_txt = "yes" if open_loop_flag else ("no" if open_loop_flag is not None else "unknown")
    driver_target = None
    if state.telemetry:
        telem_target = state.telemetry.values.get(REG_TARGET)
        if isinstance(telem_target, (int, float)):
            driver_target = float(telem_target)
    if driver_target is None:
        driver_target = state.target
    driver_txt = f"{driver_target:.2f}" if driver_target is not None else "n/a"
    stdscr.addstr(4, 0, f"Control mode: {cmode_name} (open-loop: {open_loop_txt})")
    stdscr.addstr(5, 0, f"Cmd target: {state.commanded_target:.2f} (driver: {driver_txt})")
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
                disp = map_status(int(val))
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
    parser.add_argument("--baud", type=int, default=460800, help="Baud rate (default: 460800)")
    parser.add_argument(
        "--rbinary",
        action="store_true",
        help="Use BinaryPacketCommanderClient (RobustBinaryIO protocol, default)",
    )
    args = parser.parse_args()

    client: BinaryPacketCommanderClient = BinaryPacketCommanderClient(args.port, args.baud)
    state = MotorState(client)
    setup_motor(client, state)

    # import curses

    def settings_menu():
        from menus.settings import settings_menu
        settings_menu(client)

    def telemetry_menu():
        from menus.telemetry import telemetry_menu as _tm
        _tm(client, state)

    def plot_menu():
        from menus.live_plot import plot_menu as _plot
        _plot(client, state)

    def sensor_plot_mode():
        from menus.sensor_plot import sensor_plot_mode as _sp
        _sp(client, state, stop_run)


    try:
        while True:
            print("\n=== Main Menu ===")
            print("1) Settings")
            print("2) Test")
            print("3) Telemetry")
            print("4) Live plot telemetry")
            print("5) Velocity PID tune")
            print("6) Sensor plot")
            print("7) Sensor calibration (C2)")
            print("q) Quit")
            choice = input("Select option: ").strip().lower()
            if choice == "1":
                settings_menu_handler(client)
            elif choice == "2":
                run_test_ui_handler(client, state)
            elif choice == "3":
                telemetry_menu_handler(client, state)
            elif choice == "4":
                plot_menu_handler(client, state)
            elif choice == "5":
                pid_tune_mode(client, state)
            elif choice == "6":
                sensor_plot_handler(client, state, stop_run)
            elif choice == "7":
                sensor_calibration_menu(client, state)
            elif choice == "q":
                break
    finally:
        stop_run(client, state)
        client.close()


if __name__ == "__main__":
    if not sys.platform.startswith("linux") and not sys.platform.startswith("darwin") and not sys.platform.startswith("win"):
        print("Warning: curses UI may not be supported on this platform.")
    main()
