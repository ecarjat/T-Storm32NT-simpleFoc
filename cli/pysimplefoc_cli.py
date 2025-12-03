#!/usr/bin/env python3
"""
Interactive PySimpleFOC PacketCommander client for the T-STorM32 SimpleFOC driver.

Features:
- Full rotation CW/CCW
- 10° step CW/CCW
- Slow run CW/CCW with target adjust via up/down arrows
- Velocity PID tuning helper (P/p I/i D/d adjust, live target/velocity graph)

It talks the text PacketCommander protocol over UART (PA9/PA10). Works with or without encoder:
- If encoder is available (control mode angle supported), uses angle moves.
- If sensorless (velocity_openloop), uses timed velocity moves.
"""

import argparse
from collections import deque
import math
import sys
import time
from typing import Dict, List, Optional, Union

from pysfoc import (
    PacketCommanderClient,
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
    map_control_mode,
    map_status,
)
from pysfoc.constants import (
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


def reg_display_name(reg_id: int) -> str:
    return REG_NAME_MAP.get(reg_id, f"reg{reg_id}")


def map_torque_mode(val: Optional[int]) -> str:
    if val is None:
        return "n/a"
    return TORQUE_MODE_NAMES.get(int(val), f"0x{int(val):02X}")


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


def draw_ui(stdscr, state: MotorState):
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
    stdscr.addstr(4, 0, f"Control mode: {cmode_name} (open-loop: {open_loop_txt})")
    stdscr.addstr(5, 0, f"Target: {state.target:.2f}")
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
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    args = parser.parse_args()

    client = PacketCommanderClient(args.port, args.baud)
    state = MotorState(client)
    # Ensure motor starts disabled for safety.
    try:
        state.set_enable(False)
    except Exception:
        pass
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
                    state.set_enable(True)
                    state.running = True
                    state.running_dir = 1
                    draw_ui(stdscr, state)
                    do_full_rotation(client, state, direction=1)
                    state.running = False
                    draw_ui(stdscr, state)
                    state.set_enable(False)
                elif ch == ord("F"):
                    state.set_enable(True)
                    state.running = True
                    state.running_dir = -1
                    draw_ui(stdscr, state)
                    do_full_rotation(client, state, direction=-1)
                    state.running = False
                    draw_ui(stdscr, state)
                    state.set_enable(False)
                elif ch == ord("s"):
                    state.set_enable(True)
                    state.running = True
                    state.running_dir = 1
                    draw_ui(stdscr, state)
                    do_step(client, state, direction=1)
                    state.running = False
                    draw_ui(stdscr, state)
                    state.set_enable(False)
                elif ch == ord("S"):
                    state.set_enable(True)
                    state.running = True
                    state.running_dir = -1
                    draw_ui(stdscr, state)
                    do_step(client, state, direction=-1)
                    state.running = False
                    draw_ui(stdscr, state)
                    state.set_enable(False)
                elif ch == ord("r"):
                    state.set_enable(True)
                    do_run(client, state, direction=1)
                elif ch == ord("R"):
                    state.set_enable(True)
                    do_run(client, state, direction=-1)
                elif ch == curses.KEY_UP:
                    adjust_target(client, state, 0.2)
                elif ch == curses.KEY_DOWN:
                    adjust_target(client, state, -0.2)
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
                REG_VEL_PID_P,
                "Velocity loop P gain.",
                "Proportional gain of the velocity PID (PID_velocity.P). Too high causes oscillation; too low feels sluggish.",
            ),
            (
                "Vel PID I",
                REG_VEL_PID_I,
                "Velocity loop I gain.",
                "Integral gain of the velocity PID (PID_velocity.I). Removes steady-state error; too high causes windup/oscillation.",
            ),
            (
                "Vel PID D",
                REG_VEL_PID_D,
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

    def sensor_plot_mode():
        """
        Sensor plot helper:
        - Disables the motor.
        - Sets control mode to torque (closed loop) and target to 1.
        - Configures telemetry to stream shaft angle + sensor mechanical angle, both wrapped to [0, 2π).
        - Opens a live graph with keyboard controls:
          T/t = increase/decrease target, SPACE = enable/disable motor, q/Esc = exit.
        """
        print("\nEntering Sensor Plot mode:")
        print("  - Motor disabled, control mode set to torque, target set to 1.")
        print("  - Telemetry: shaft angle + sensor mechanical angle (both normalized 0..2π).")
        state.refresh_status()
        prev_control_mode = state.control_mode_val if state.control_mode_val is not None else state.control_mode
        prev_target = state.target
        prev_regs = [reg for motor, reg in client.telemetry_headers.get(0, DEFAULT_TELEM_REGS) if motor == 0]
        try:
            prev_downsample = client.read_reg(REGISTER_IDS["telemetry_downsample"])
        except Exception:
            prev_downsample = None

        # Disable motor and stop motion, then set torque mode and target.
        state.set_enable(False)
        stop_run(client, state)
        state.set_control_mode(CONTROL_MODE_IDS["torque"])
        target_val = 1.0
        state.target = target_val
        state.set_target(target_val)
        client.set_telemetry_registers([REG_ANGLE, REG_SENSOR_MECH_ANGLE])
        client.write_reg(REGISTER_IDS["telemetry_downsample"], 100)
        state.refresh_status()
        state.running = False
        state.open_loop = False
        state.telemetry = None

        try:
            import matplotlib.pyplot as plt  # type: ignore
            from matplotlib.animation import FuncAnimation  # type: ignore
        except Exception as e:
            print("matplotlib is required for sensor plotting. Install with `pip install matplotlib`.")
            print(f"Import error: {e}")
            return

        window = 10.0
        regs_sel = [REG_ANGLE, REG_SENSOR_MECH_ANGLE]
        times = {r: deque(maxlen=5000) for r in regs_sel}
        values = {r: deque(maxlen=5000) for r in regs_sel}
        t0 = time.time()

        fig, ax = plt.subplots(figsize=(9, 4))
        colors = {REG_ANGLE: "C0", REG_SENSOR_MECH_ANGLE: "C1"}
        labels = {REG_ANGLE: "shaft_angle", REG_SENSOR_MECH_ANGLE: "sensor_mech_angle"}
        lines = {r: ax.plot([], [], label=labels[r], color=colors[r])[0] for r in regs_sel}
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Angle (rad, 0..2π)")
        ax.legend(loc="upper right")
        ax.grid(True)
        ax.set_ylim(0, 2 * math.pi)

        TARGET_STEP = 0.1
        running_flag = False

        status_text = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", ha="left")

        def norm_angle(rad_val: float) -> float:
            wrapped = math.fmod(rad_val, 2 * math.pi)
            if wrapped < 0:
                wrapped += 2 * math.pi
            return wrapped

        def update_status_text():
            status_text.set_text(
                f"Target {target_val:.3f} | Motor {'ENABLED' if running_flag else 'disabled'}"
            )

        def set_motor_enabled(enabled: bool):
            nonlocal running_flag
            running_flag = enabled
            state.set_enable(enabled)
            if enabled:
                state.set_target(target_val)
            else:
                state.set_target(0.0)
            update_status_text()

        def adjust_target(delta: float):
            nonlocal target_val
            target_val += delta
            state.target = target_val
            state.set_target(target_val)
            update_status_text()

        def on_key(event):
            if not event.key:
                return
            key = event.key
            if key in (" ", "space"):
                set_motor_enabled(not running_flag)
            elif key in ("q", "escape"):
                plt.close(fig)
            elif key == "T":
                adjust_target(TARGET_STEP)
            elif key == "t":
                adjust_target(-TARGET_STEP)

        def update(frame):
            telem = client.poll_telemetry()
            now = time.time()
            if telem:
                state.telemetry = telem
                for r in regs_sel:
                    val = telem.values.get(r)
                    if isinstance(val, (int, float)):
                        t_rel = telem.timestamp - t0
                        times[r].append(t_rel)
                        values[r].append(norm_angle(float(val)))
            for r in regs_sel:
                while times[r] and (now - t0) - times[r][0] > window:
                    times[r].popleft()
                    values[r].popleft()
                lines[r].set_data(list(times[r]), list(values[r]))
            max_time = max((times[r][-1] for r in times if times[r]), default=window)
            ax.set_xlim(max(0, max_time - window), max(max_time, window))
            return list(lines.values())

        update_status_text()
        cid = fig.canvas.mpl_connect("key_press_event", on_key)
        anim = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
        try:
            plt.show()
        finally:
            try:
                fig.canvas.mpl_disconnect(cid)
            except Exception:
                pass
            if getattr(anim, "event_source", None):
                anim.event_source.stop()
            plt.close(fig)
            # Restore prior settings
            stop_run(client, state)
            state.set_enable(False)
            if prev_control_mode is not None:
                state.set_control_mode(prev_control_mode)
            if prev_target is not None:
                state.set_target(prev_target)
            if prev_regs:
                client.set_telemetry_registers(prev_regs)
            if prev_downsample is not None:
                try:
                    client.write_reg(REGISTER_IDS["telemetry_downsample"], int(prev_downsample))
                except Exception:
                    pass
            state.refresh_status()

    def pid_tune_mode():
        """
        Velocity PID tuning helper:
        - Disables the motor.
        - Zeros velocity PID P/I/D registers.
        - Sets target to 5 rad/s and control mode to closed-loop velocity.
        - Configures telemetry to Target + Velocity.
        - Opens a live graph with keyboard controls:
          P/p, I/i, D/d = increase/decrease respective gains, SPACE = enable/disable motor, q/Esc = exit.
        """
        print("\nEntering PID tuning mode:")
        print("  - Motor will be disabled, velocity PID gains zeroed, target set to 5 rad/s.")
        print("  - Telemetry will stream target and velocity. Use matplotlib window keys:")
        print("    P/p, I/i, D/d to adjust gains; SPACE to enable/disable; q or Esc to quit.\n")
        state.refresh_status()
        prev_control_mode = state.control_mode_val if state.control_mode_val is not None else state.control_mode
        prev_target = state.target
        prev_regs = [reg for motor, reg in client.telemetry_headers.get(0, DEFAULT_TELEM_REGS) if motor == 0]
        try:
            prev_downsample = client.read_reg(REGISTER_IDS["telemetry_downsample"])
        except Exception:
            prev_downsample = None

        # 1) Disable motor and stop motion.
        state.set_enable(False)
        stop_run(client, state)

        # 2) Zero PID gains.
        for reg in (REG_VEL_PID_P, REG_VEL_PID_I, REG_VEL_PID_D):
            client.write_reg(reg, 0.0)

        # 3) Set target to 5 rad/s and switch to closed-loop velocity.
        pid_target = 5.0
        state.target = pid_target
        state.set_target(pid_target)
        client.set_telemetry_registers([REG_TARGET, REG_VELOCITY])
        client.write_reg(REGISTER_IDS["telemetry_downsample"], 100)
        state.set_control_mode(CONTROL_MODE_IDS["velocity"])
        state.refresh_status()
        state.running = False
        state.open_loop = False
        state.telemetry = None

        try:
            import matplotlib.pyplot as plt  # type: ignore
            from matplotlib.animation import FuncAnimation  # type: ignore
        except Exception as e:
            print("matplotlib is required for PID tuning graph. Install with `pip install matplotlib`.")
            print(f"Import error: {e}")
            return

        window = 10.0
        times = {REG_TARGET: deque(maxlen=5000), REG_VELOCITY: deque(maxlen=5000)}
        values = {REG_TARGET: deque(maxlen=5000), REG_VELOCITY: deque(maxlen=5000)}
        t0 = time.time()

        fig, ax = plt.subplots(figsize=(9, 4))
        lines = {
            REG_TARGET: ax.plot([], [], label="target", color="C0")[0],
            REG_VELOCITY: ax.plot([], [], label="velocity", color="C1")[0],
        }
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Value")
        ax.legend(loc="upper right")
        ax.grid(True)

        def read_gain(reg, fallback=0.0):
            """Read a gain; fall back to last cached value instead of zeroing on timeout."""
            val = client.read_reg(reg)
            return float(val) if val is not None else fallback

        pid_values = {
            REG_VEL_PID_P: read_gain(REG_VEL_PID_P, 0.0),
            REG_VEL_PID_I: read_gain(REG_VEL_PID_I, 0.0),
            REG_VEL_PID_D: read_gain(REG_VEL_PID_D, 0.0),
        }
        running_flag = False

        status_text = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", ha="left")

        def update_status_text():
            status_text.set_text(
                f"P {pid_values[REG_VEL_PID_P]:.3f}  I {pid_values[REG_VEL_PID_I]:.3f}  D {pid_values[REG_VEL_PID_D]:.3f}\n"
                f"Motor {'ENABLED' if running_flag else 'disabled'} | Target {pid_target:.2f} rad/s"
            )

        def set_motor_enabled(enabled: bool):
            nonlocal running_flag
            running_flag = enabled
            state.set_enable(enabled)
            if enabled:
                state.set_target(pid_target)
            else:
                state.set_target(0.0)
            update_status_text()

        def adjust_gain(reg: int, delta: float):
            # Use integer tick steps so repeated keypresses don't depend on MCU readback.
            step = step_map.get(reg, delta if delta != 0 else 0.01)
            ticks = pid_ticks.get(reg, int(round(pid_values.get(reg, 0.0) / step)))
            delta_ticks = int(round(delta / step)) if step != 0 else 0
            if delta_ticks == 0:
                delta_ticks = 1 if delta > 0 else -1
            ticks += delta_ticks
            pid_ticks[reg] = ticks
            new_val = ticks * step
            pid_values[reg] = new_val
            client.write_reg(reg, new_val)
            update_status_text()

        P_STEP = 0.01
        I_STEP = 0.01
        D_STEP = 0.01
        step_map = {
            REG_VEL_PID_P: P_STEP,
            REG_VEL_PID_I: I_STEP,
            REG_VEL_PID_D: D_STEP,
        }
        pid_ticks = {
            reg: int(round(pid_values[reg] / step_map[reg]))
            for reg in pid_values
            if reg in step_map and step_map[reg] != 0
        }

        def on_key(event):
            if not event.key:
                return
            key = event.key
            if key in (" ", "space"):
                set_motor_enabled(not running_flag)
            elif key in ("q", "escape"):
                plt.close(fig)
            elif key == "P":
                adjust_gain(REG_VEL_PID_P, P_STEP)
            elif key == "p":
                adjust_gain(REG_VEL_PID_P, -P_STEP)
            elif key == "I":
                adjust_gain(REG_VEL_PID_I, I_STEP)
            elif key == "i":
                adjust_gain(REG_VEL_PID_I, -I_STEP)
            elif key == "D":
                adjust_gain(REG_VEL_PID_D, D_STEP)
            elif key == "d":
                adjust_gain(REG_VEL_PID_D, -D_STEP)

        def update(frame):
            telem = client.poll_telemetry()
            now = time.time()
            if telem:
                state.telemetry = telem
                for reg in (REG_TARGET, REG_VELOCITY):
                    val = telem.values.get(reg)
                    if isinstance(val, (int, float)):
                        t_rel = telem.timestamp - t0
                        times[reg].append(t_rel)
                        values[reg].append(float(val))
            for reg in (REG_TARGET, REG_VELOCITY):
                while times[reg] and (now - t0) - times[reg][0] > window:
                    times[reg].popleft()
                    values[reg].popleft()
                lines[reg].set_data(list(times[reg]), list(values[reg]))
            # Adjust axes
            max_time = max((times[reg][-1] for reg in times if times[reg]), default=window)
            ax.set_xlim(max(0, max_time - window), max(max_time, window))
            all_vals = [val for reg_vals in values.values() for val in reg_vals]
            if all_vals:
                vmin = min(all_vals)
                vmax = max(all_vals)
                if vmin == vmax:
                    vmin -= 0.1
                    vmax += 0.1
                ax.set_ylim(vmin, vmax)
            return list(lines.values())

        update_status_text()
        cid = fig.canvas.mpl_connect("key_press_event", on_key)
        anim = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
        try:
            plt.show()
        finally:
            try:
                fig.canvas.mpl_disconnect(cid)
            except Exception:
                pass
            if getattr(anim, "event_source", None):
                anim.event_source.stop()
            plt.close(fig)
            # Restore prior settings
            stop_run(client, state)
            state.set_enable(False)
            if prev_control_mode is not None:
                state.set_control_mode(prev_control_mode)
            if prev_target is not None:
                state.set_target(prev_target)
            if prev_regs:
                client.set_telemetry_registers(prev_regs)
            if prev_downsample is not None:
                try:
                    client.write_reg(REGISTER_IDS["telemetry_downsample"], int(prev_downsample))
                except Exception:
                    pass
            state.refresh_status()

    try:
        while True:
            print("\n=== Main Menu ===")
            print("1) Settings")
            print("2) Test")
            print("3) Telemetry")
            print("4) Live plot telemetry")
            print("5) Velocity PID tune")
            print("6) Sensor plot")
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
            elif choice == "5":
                pid_tune_mode()
            elif choice == "6":
                sensor_plot_mode()
            elif choice == "q":
                break
    finally:
        stop_run(client, state)
        client.close()


if __name__ == "__main__":
    if not sys.platform.startswith("linux") and not sys.platform.startswith("darwin") and not sys.platform.startswith("win"):
        print("Warning: curses UI may not be supported on this platform.")
    main()
