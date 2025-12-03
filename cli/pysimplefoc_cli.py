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

from pysfoc import (  # type: ignore[import-not-found]
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


def setup_motor(client: PacketCommanderClient, state: MotorState) -> None:
    # Read current control mode to decide open-loop vs closed-loop
    state.refresh_status()
    if state.control_mode is None:
        state.set_control_mode(CONTROL_MODE_IDS["velocity"])

    # Ensure motor stays disabled on startup; menus/actions will enable explicitly.
    state.set_enable(False)

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
    setup_motor(client, state)

    import curses

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
        LPF_REG = 0x35
        lpf_tf = read_gain(LPF_REG, 0.01)
        running_flag = False

        status_text = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", ha="left")

        def update_status_text():
            status_text.set_text(
                f"P {pid_values[REG_VEL_PID_P]:.3f}  I {pid_values[REG_VEL_PID_I]:.3f}  D {pid_values[REG_VEL_PID_D]:.4f}\n"
                f"Motor {'ENABLED' if running_flag else 'disabled'} | Target {pid_target:.2f} rad/s | LPF Tf {lpf_tf:.3f}s"
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
        D_STEP = 0.0001
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

        def adjust_target(delta: float):
            nonlocal pid_target
            pid_target = max(0.0, pid_target + delta)
            if running_flag:
                state.set_target(pid_target)
            update_status_text()

        def bump_lpf():
            nonlocal lpf_tf
            lpf_tf += 0.01
            client.write_reg(LPF_REG, lpf_tf)
            update_status_text()

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
            elif key in ("up",):
                adjust_target(1.0)
            elif key in ("down",):
                adjust_target(-1.0)
            elif key in ("pageup",):
                adjust_target(10.0)
            elif key in ("pagedown",):
                adjust_target(-10.0)
            elif key in ("L", "l"):
                bump_lpf()

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

    def sensor_calibration_menu():
        """
        Trigger sensor calibration via PacketCommander:
        - Stop telemetry (R28=0).
        - Send C2 command.
        - Stream serial output until CAL_OK or CAL_SAVE_ERR is seen (or timeout).
        - Display LUT table output lines if present.
        """
        print("\nStarting sensor calibration (C2)...")
        try:
            client.write_reg(REGISTER_IDS["telemetry_downsample"], 0)
        except Exception:
            pass
        try:
            client.ser.reset_input_buffer()
        except Exception:
            pass
        client._write_line("C2")
        deadline = time.time() + 10.0
        result = None
        print("Waiting for calibration output (up to 10s):")
        while time.time() < deadline and result is None:
            line = client.ser.readline()
            if not line:
                continue
            try:
                text = line.decode("ascii", errors="ignore").strip()
            except Exception:
                continue
            if not text:
                continue
            print(f"  {text}")
            if "CAL_OK" in text:
                result = "CAL_OK"
            elif "CAL_SAVE_ERR" in text:
                result = "CAL_SAVE_ERR"
        if result is None:
            print("Calibration did not report completion before timeout.")
        elif result == "CAL_OK":
            print("Calibration successful (CAL_OK).")
        else:
            print("Calibration reported CAL_SAVE_ERR (save to flash failed).")

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
