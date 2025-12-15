#!/usr/bin/env python3
"""
Live plot of PacketCommander telemetry (target vs measured velocity).

Usage:
  python plot_telemetry.py --port /dev/ttyACM0 --baud 460800
"""

import argparse
import time
from collections import deque

from pysfoc import PySFOCClient, REG_TARGET, REG_VELOCITY


def main():
    parser = argparse.ArgumentParser(description="Live plot of target/measured velocity via PacketCommander telemetry.")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=460800, help="Baud rate (default 460800)")
    parser.add_argument("--window", type=float, default=10.0, help="Plot window in seconds (default 10)")
    parser.add_argument("--telemetry-rate", type=float, default=200.0, help="Requested telemetry rate (Hz)")
    parser.add_argument("--interval", type=int, default=50, help="Plot refresh interval in ms (default 50)")
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt  # type: ignore
        from matplotlib.animation import FuncAnimation  # type: ignore
    except Exception as e:
        print("matplotlib is required for plotting. Install with `pip install matplotlib`.")
        print(f"Import error: {e}")
        return

    client = PySFOCClient(port=args.port, baud=args.baud)
    client.telemetry_config(regs=[REG_TARGET, REG_VELOCITY], target_rate_hz=args.telemetry_rate)

    times = deque(maxlen=5000)
    targets = deque(maxlen=5000)
    measured = deque(maxlen=5000)
    t0 = time.monotonic()

    fig, ax = plt.subplots(figsize=(8, 4))
    line_target, = ax.plot([], [], label="target", color="C1")
    line_measured, = ax.plot([], [], label="velocity", color="C0")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (rad/s)")
    ax.grid(True)
    ax.legend(loc="upper right")

    def update(frame):
        sample = client.poll_velocity_sample()
        if sample:
            t_rel = sample.t_host - t0
            if sample.vel_target is not None:
                times.append(t_rel)
                targets.append(sample.vel_target)
            if sample.vel_measured is not None:
                # ensure lists stay aligned
                while len(measured) < len(times) - 1:
                    measured.append(measured[-1] if measured else 0.0)
                measured.append(sample.vel_measured)
        # trim window
        while times and (times[-1] - times[0]) > args.window:
            times.popleft()
            targets.popleft() if targets else None
            measured.popleft() if measured else None
        line_target.set_data(list(times), list(targets))
        line_measured.set_data(list(times), list(measured))
        if times:
            ax.set_xlim(max(0, times[0]), times[-1])
            all_vals = list(targets) + list(measured)
            if all_vals:
                vmin = min(all_vals)
                vmax = max(all_vals)
                if vmin == vmax:
                    vmin -= 0.1
                    vmax += 0.1
                ax.set_ylim(vmin, vmax)
        return line_target, line_measured

    anim = FuncAnimation(fig, update, interval=args.interval, blit=False, cache_frame_data=False)
    try:
        plt.show()
    finally:
        if getattr(anim, "event_source", None):
            anim.event_source.stop()
        plt.close(fig)
        client.close()


if __name__ == "__main__":
    main()
