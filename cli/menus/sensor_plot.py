from __future__ import annotations

import math
from collections import deque

from pysfoc import CONTROL_MODE_IDS, REG_ANGLE, REG_SENSOR_MECH_ANGLE, REG_TARGET  # type: ignore[import-not-found]
from pysfoc.constants import DEFAULT_TELEM_REGS, REGISTER_IDS, REG_NAME_MAP  # type: ignore[import-not-found]
from pysfoc.packet_commander import PacketCommanderClient  # type: ignore[import-not-found]
from pysfoc.api import MotorState  # type: ignore[import-not-found]


def reg_display_name(reg_id: int) -> str:
    return REG_NAME_MAP.get(reg_id, f"reg{reg_id}")


def sensor_plot_mode(client: PacketCommanderClient, state: MotorState, stop_run_fn) -> None:
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

    state.set_enable(False)
    stop_run_fn(client, state)
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
    times: dict[int, deque] = {r: deque(maxlen=5000) for r in regs_sel}
    values: dict[int, deque] = {r: deque(maxlen=5000) for r in regs_sel}
    t0 = None

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

    def update_status_text():
        status_text.set_text(
            f"Target {state.target:.2f} | Motor {'ENABLED' if running_flag else 'disabled'}"
        )

    def set_motor_enabled(enabled: bool):
        nonlocal running_flag
        running_flag = enabled
        state.set_enable(enabled)
        if not enabled:
            state.set_target(0.0)
        else:
            state.set_target(state.target)
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
            state.target += TARGET_STEP
            state.set_target(state.target)
            update_status_text()
        elif key == "t":
            state.target -= TARGET_STEP
            state.set_target(state.target)
            update_status_text()

    def update(frame):
        nonlocal t0
        telem = client.poll_telemetry()
        now = None
        if telem:
            if t0 is None:
                t0 = telem.timestamp
            now = telem.timestamp
            for r in regs_sel:
                raw_val = telem.values.get(r)
                if isinstance(raw_val, (float, int)):
                    val = raw_val % (2 * math.pi)
                    t_rel = telem.timestamp - t0
                    times[r].append(t_rel)
                    values[r].append(val)
        if t0 is None:
            return list(lines.values())
        for r in regs_sel:
            while times[r] and times[r][-1] - times[r][0] > window:
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
        stop_run_fn(client, state)
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
