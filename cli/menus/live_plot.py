from __future__ import annotations

from collections import deque

from pysfoc import PacketCommanderClient  # type: ignore[import-not-found]
from pysfoc.constants import DEFAULT_TELEM_REGS, REG_NAME_MAP, REG_VALUE_FIELDS  # type: ignore[import-not-found]


def reg_display_name(reg_id: int) -> str:
    return REG_NAME_MAP.get(reg_id, f"reg{reg_id}")


def plot_menu(client: PacketCommanderClient, state) -> None:
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
    times: dict[int, deque] = {r: deque(maxlen=5000) for r in regs_sel}
    values: dict[int, deque] = {r: deque(maxlen=5000) for r in regs_sel}
    t0 = None

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
        nonlocal t0
        telem = client.poll_telemetry()
        now = None
        if telem:
            if t0 is None:
                t0 = telem.timestamp
            now = telem.timestamp
            for r in regs_sel:
                val = telem.values.get(r)
                if isinstance(val, (float, int)):
                    t_rel = telem.timestamp - t0
                    times[r].append(t_rel)
                    values[r].append(float(val))
        if t0 is None:
            return list(lines.values())
        for r in regs_sel:
            while times[r] and times[r][-1] - times[r][0] > window:
                times[r].popleft()
                values[r].popleft()
            lines[r].set_data(list(times[r]), list(values[r]))
            if times[r]:
                axis_map[r].set_xlim(max(0, times[r][0]), times[r][-1] if times[r][-1] > window else window)
                if values[r]:
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
