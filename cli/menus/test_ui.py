from __future__ import annotations

import curses
import time
from collections import deque
from typing import Deque, Tuple

from pysfoc import CONTROL_MODE_IDS, OPENLOOP_MODES, REG_ANGLE, REG_SENSOR_MECH_ANGLE  # type: ignore[import-not-found]
from pysfoc.constants import CONTROL_MODE_SEQUENCE, REG_CONTROL_MODE, REG_ENABLE, REG_POSITION, REG_STATUS, REG_TARGET, REG_TORQUE_MODE, REG_VELOCITY, REGISTER_IDS, REG_NAME_MAP  # type: ignore[import-not-found]
from pysfoc.packet_commander import BinaryPacketCommanderClient  # type: ignore[import-not-found]
from pysfoc.api import MotorState  # type: ignore[import-not-found]

from .utils import draw_ui, do_full_rotation, do_run, do_step, adjust_target, stop_run


def run_test_ui(client: BinaryPacketCommanderClient, state: MotorState) -> None:
    def refresh_status_regs() -> None:
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

    def loop(stdscr) -> None:
        curses.curs_set(0)
        stdscr.nodelay(True)
        refresh_status_regs()
        try:
            try:
                client.ser.reset_input_buffer()
            except Exception:
                pass
            client.set_telemetry_registers(
                [
                    REGISTER_IDS["target"],
                    REGISTER_IDS["enable"],
                    REGISTER_IDS["velocity"],
                    REGISTER_IDS["angle"],
                    REGISTER_IDS["position"],
                    REGISTER_IDS["status"],
                    REGISTER_IDS["torque_mode"],
                    REGISTER_IDS["control_mode"],
                ]
            )
            # Make sure telemetry is enabled (downsample 0 disables streaming).
            # client.write_reg(REGISTER_IDS["telemetry_ctrl"], 0)
            # Use a modest downsample to avoid saturating the STM32 loop.
            client.write_reg(REGISTER_IDS["telemetry_downsample"], 5)
        except Exception:
            pass
        history: Deque[Tuple[float, float, float]] = deque(maxlen=5000)  # (t, target, velocity)
        t0 = time.time()
        message = "SPACE toggles enable/disable; g opens plot"
        last_telem = time.time()
        command_history: Deque[str] = deque(maxlen=3)
        log_history: Deque[str] = deque(maxlen=10)
        spinner_phase = 0
        spinner_chars = "|/-\\"
        interesting_regs = {
            REGISTER_IDS["target"],
            REGISTER_IDS["enable"],
            REGISTER_IDS["control_mode"],
            REGISTER_IDS["torque_mode"],
        }

        def format_value(val):
            try:
                return f"{float(val):.3f}"
            except Exception:
                return str(val)

        def log_command(reg: int, value):
            reg_name = REG_NAME_MAP.get(reg, f"reg{reg}")
            command_history.appendleft(f"{reg_name:<14} {format_value(value)}")

        orig_write_reg = client.write_reg

        def wrapped_write_reg(reg, value):
            resp = orig_write_reg(reg, value)
            if reg in interesting_regs:
                log_command(reg, value)
            return resp

        client.write_reg = wrapped_write_reg

        try:
            prev_log_packets = getattr(client, "log_packets", False)
            prev_logger = getattr(client, "_log", None)

            def log_handler(msg: str) -> None:
                clean = msg.replace("\n", " ").strip()
                if clean:
                    log_history.appendleft(clean)

            client.log_packets = True
            client._log = log_handler

            while True:
                telem = client.poll_telemetry()
                if telem:
                    state.telemetry = telem
                    tgt = telem.values.get(REG_TARGET)
                    vel = telem.values.get(REG_VELOCITY)
                    if isinstance(tgt, (int, float)) and isinstance(vel, (int, float)):
                        history.append((telem.timestamp - t0, float(tgt), float(vel)))
                    last_telem = time.time()

                draw_ui(stdscr, state, message, list(command_history), spinner_phase, list(log_history))
                spinner_phase = (spinner_phase + 1) % len(spinner_chars)
                message = ""

                # Allow matplotlib animations to process GUI events without blocking the curses UI.
                try:
                    import matplotlib.pyplot as plt  # type: ignore

                    if hasattr(plt, "_testui_anims"):  # type: ignore[attr-defined]
                        plt.pause(0.001)
                except Exception:
                    pass

                ch = stdscr.getch()
                if ch == -1:
                    time.sleep(0.01)
                    continue
                if ch in (ord("q"), ord("Q")):
                    stop_run(client, state)
                    break
                elif ch in (ord(" "),):
                    if state.enable_val:
                        stop_run(client, state)
                        state.set_enable(False)
                        message = "Motor disabled"
                    else:
                        state.set_enable(True)
                        state.set_target(state.commanded_target)
                        message = "Motor enabled"
                    continue
                elif ch == ord("f"):
                    state.set_enable(True)
                    state.running = True
                    state.running_dir = 1
                    do_full_rotation(client, state, direction=1)
                    state.running = False
                    state.set_enable(False)
                    message = "Full rotation CW"
                elif ch == ord("F"):
                    state.set_enable(True)
                    state.running = True
                    state.running_dir = -1
                    do_full_rotation(client, state, direction=-1)
                    state.running = False
                    state.set_enable(False)
                    message = "Full rotation CCW"
                elif ch == ord("s"):
                    state.set_enable(True)
                    state.running = True
                    state.running_dir = 1
                    do_step(client, state, direction=1)
                    state.running = False
                    state.set_enable(False)
                    message = "+10 deg"
                elif ch == ord("S"):
                    state.set_enable(True)
                    state.running = True
                    state.running_dir = -1
                    do_step(client, state, direction=-1)
                    state.running = False
                    state.set_enable(False)
                    message = "-10 deg"
                elif ch == ord("r"):
                    state.set_enable(True)
                    do_run(client, state, direction=1)
                    message = "Run forward"
                elif ch == ord("R"):
                    state.set_enable(True)
                    do_run(client, state, direction=-1)
                    message = "Run reverse"
                elif ch == curses.KEY_UP:
                    step = 1.0 if state.control_mode == CONTROL_MODE_IDS["velocity"] else 0.01
                    state.commanded_target += step
                    state.set_target(state.commanded_target)
                elif ch == curses.KEY_DOWN:
                    step = 1.0 if state.control_mode == CONTROL_MODE_IDS["velocity"] else 0.01
                    state.commanded_target = max(0.0, state.commanded_target - step)
                    state.set_target(state.commanded_target)
                elif ch == curses.KEY_PPAGE:  # PageUp
                    step = 10.0 if state.control_mode == CONTROL_MODE_IDS["velocity"] else 1.0
                    state.commanded_target += step
                    state.set_target(state.commanded_target)
                elif ch == curses.KEY_NPAGE:  # PageDown
                    step = 10.0 if state.control_mode == CONTROL_MODE_IDS["velocity"] else 1.0
                    state.commanded_target = max(0.0, state.commanded_target - step)
                    state.set_target(state.commanded_target)
                elif ch == ord("m"):
                    current = state.control_mode if state.control_mode is not None else CONTROL_MODE_SEQUENCE[0]
                    try:
                        idx = CONTROL_MODE_SEQUENCE.index(current)
                        next_mode = CONTROL_MODE_SEQUENCE[(idx + 1) % len(CONTROL_MODE_SEQUENCE)]
                    except ValueError:
                        next_mode = CONTROL_MODE_SEQUENCE[0]
                    state.set_control_mode(next_mode)
                    state.refresh_status()
                    message = f"Mode -> {next_mode}"
                elif ch in (ord("g"), ord("G")):
                    def launch_plot():
                        try:
                            import matplotlib.pyplot as plt  # type: ignore
                            from matplotlib.animation import FuncAnimation  # type: ignore
                        except Exception:
                            return
                        fig, ax = plt.subplots(figsize=(8, 4))
                        line_tgt, = ax.plot([], [], label="target", color="C1")
                        line_vel, = ax.plot([], [], label="velocity", color="C0")
                        ax.set_xlabel("Time (s)")
                        ax.set_ylabel("Velocity (rad/s)")
                        ax.legend(loc="upper right")
                        ax.grid(True)

                        def update(frame):
                            xs = [h[0] for h in history]
                            ys_t = [h[1] for h in history]
                            ys_v = [h[2] for h in history]
                            line_tgt.set_data(xs, ys_t)
                            line_vel.set_data(xs, ys_v)
                            if xs:
                                ax.set_xlim(max(0, xs[-1] - 10), xs[-1] if xs[-1] > 10 else 10)
                                vals = ys_t + ys_v
                                if vals:
                                    vmin = min(vals)
                                    vmax = max(vals)
                                    if vmin == vmax:
                                        vmin -= 0.1
                                        vmax += 0.1
                                    ax.set_ylim(vmin, vmax)
                            return line_tgt, line_vel

                        anim = FuncAnimation(fig, update, interval=200, blit=False, cache_frame_data=False)
                        # Keep references so the animation and figure aren't garbage-collected.
                        if not hasattr(plt, "_testui_anims"):
                            plt._testui_anims = []  # type: ignore[attr-defined]
                        plt._testui_anims.append((fig, anim))  # type: ignore[attr-defined]
                        plt.ion()
                        fig.show()
                        plt.show(block=False)
                        # Give the GUI loop a tick so the window actually appears.
                        plt.pause(0.001)

                    # On macOS (and other GUI backends) window creation must be on the main thread.
                    launch_plot()
                    message = "Plot window opened"
        finally:
            client.write_reg = orig_write_reg
            try:
                client.log_packets = prev_log_packets
                client._log = prev_logger
            except Exception:
                pass

    curses.wrapper(loop)
