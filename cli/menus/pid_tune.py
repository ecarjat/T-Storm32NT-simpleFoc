from __future__ import annotations

import time
from collections import deque
from typing import Dict

from pysfoc import (
    CONTROL_MODE_IDS,
    REG_TARGET,
    REG_VELOCITY,
    REG_VEL_PID_D,
    REG_VEL_PID_I,
    REG_VEL_PID_P,
)
from pysfoc.constants import DEFAULT_TELEM_REGS, REGISTER_IDS, REG_NAME_MAP
from pysfoc.packet_commander import BinaryPacketCommanderClient
from pysfoc.api import MotorState


def pid_tune_mode(client: BinaryPacketCommanderClient, state: MotorState) -> None:
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
    prev_target = state.commanded_target
    prev_regs = [reg for motor, reg in client.telemetry_headers.get(0, DEFAULT_TELEM_REGS) if motor == 0]
    try:
        prev_downsample = client.read_reg(REGISTER_IDS["telemetry_downsample"])
    except Exception:
        prev_downsample = None

    # 1) Disable motor and stop motion.
    state.set_enable(False)
    from cli.pysimplefoc_cli import stop_run  # lazy import to avoid cycle

    stop_run(client, state)

    # 2) Zero PID gains.
    for reg in (REG_VEL_PID_P, REG_VEL_PID_I, REG_VEL_PID_D):
        client.write_reg(reg, 0.0)

    # 3) Set target to 5 rad/s and switch to closed-loop velocity.
    pid_target = 5.0
    state.set_target(pid_target, track_command=True)
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
    times: Dict[int, deque] = {REG_TARGET: deque(maxlen=5000), REG_VELOCITY: deque(maxlen=5000)}
    values: Dict[int, deque] = {REG_TARGET: deque(maxlen=5000), REG_VELOCITY: deque(maxlen=5000)}
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

    def adjust_gain(reg: int, delta: float):
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

    def adjust_target(delta: float):
        nonlocal pid_target
        pid_target = max(0.0, pid_target + delta)
        state.commanded_target = pid_target
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
        stop_run(client, state)
        state.set_enable(False)
        if prev_control_mode is not None:
            state.set_control_mode(prev_control_mode)
        if prev_target is not None:
            state.set_target(prev_target, track_command=True)
        if prev_regs:
            client.set_telemetry_registers(prev_regs)
        if prev_downsample is not None:
            try:
                client.write_reg(REGISTER_IDS["telemetry_downsample"], int(prev_downsample))
            except Exception:
                pass
        state.refresh_status()
