from __future__ import annotations

from pysfoc import BinaryPacketCommanderClient  # type: ignore[import-not-found]
from pysfoc.constants import (  # type: ignore[import-not-found]
    DEFAULT_TELEM_REGS,
    REG_NAME_MAP,
    REG_VALUE_FIELDS,
    REGISTER_IDS,
    TORQUE_MODE_NAMES,
    CONTROL_MODE_NAMES,
    REG_CONTROL_MODE,
)


def reg_display_name(reg_id: int) -> str:
    return REG_NAME_MAP.get(reg_id, f"reg{reg_id}")


def telemetry_menu(client: BinaryPacketCommanderClient, state) -> None:
    available_regs = [
        ("Target", 0x01, "Commanded target (float)"),
        ("Angle", 0x09, "Shaft angle (rad)"),
        ("Position", 0x10, "Full rotations + angle (int32 + rad)"),
        ("Velocity", 0x11, "Shaft velocity (rad/s)"),
        ("Sensor angle", 0x12, "Sensor electrical angle (rad)"),
        ("Sensor mechanical angle", 0x13, "Sensor mechanical angle (rad)"),
        ("Sensor velocity", 0x14, "Sensor velocity (rad/s)"),
        ("Sensor timestamp", 0x15, "Sensor timestamp (uint32)"),
        ("Enable", 0x04, "Motor enable flag (0/1)"),
        ("Status", 0x00, "Motor status byte"),
        ("Torque mode", REGISTER_IDS["torque_mode"], "Torque control mode"),
        ("Control mode", 0x05, "Motion control mode"),
        ("Velocity limit", 0x52, "Velocity limit (rad/s)"),
        ("Voltage limit", 0x50, "Motor voltage_limit (V)"),
        ("Driver voltage limit", 0x53, "Driver voltage_limit (V)"),
        ("Driver PSU voltage", 0x55, "Driver supply voltage (V)"),
        ("Pole pairs", 0x63, "Pole pairs (int)"),
    ]
    while True:
        current = [reg for motor, reg in client.telemetry_headers.get(0, DEFAULT_TELEM_REGS) if motor == 0]
        invalid_regs = [r for r in current if r not in REG_NAME_MAP]
        if not current or invalid_regs:
            # Auto-fix obvious bad header (e.g. stray 0xFF) by re-sending a sane default list.
            default_regs = [REGISTER_IDS["target"], REGISTER_IDS["enable"], REGISTER_IDS["velocity"], REGISTER_IDS["angle"], REGISTER_IDS["status"]]
            client.set_telemetry_registers(default_regs)
            try:
                client.set_telemetry_rate_hz(200.0)
                client.write_reg(REGISTER_IDS["telemetry_ctrl"], 0)
            except Exception:
                pass
            current = default_regs
            print("\nTelemetry header looked invalid; reset to default register set.")
        print("\nTelemetry registers (motor 0):")
        print("  Current:", ", ".join([REG_NAME_MAP.get(r, str(r)) for r in current]) or "none")
        try:
            ctrl = client.read_reg(REGISTER_IDS["telemetry_ctrl"])
        except Exception:
            ctrl = None
        try:
            min_elapsed = client.read_reg(REGISTER_IDS["telemetry_min_elapsed"])
        except Exception:
            min_elapsed = None
        if ctrl is not None:
            print(f"  Telemetry controller index (REG_TELEMETRY_CTRL): {int(ctrl)}")
        if min_elapsed is not None:
            min_elapsed = int(min_elapsed)
            if min_elapsed > 0:
                hz = 1_000_000.0 / min_elapsed
                print(f"  Telemetry rate (REG_TELEMETRY_MIN_ELAPSED): {hz:.2f} Hz ({min_elapsed} us)")
            else:
                print("  Telemetry rate (REG_TELEMETRY_MIN_ELAPSED): disabled (0 us)")
        print("\nAvailable choices:")
        for idx, (label, reg, desc) in enumerate(available_regs, start=1):
            mark = "*" if reg in current else " "
            print(f"  {idx}) [{mark}] {label} (reg {reg}) - {desc}")
        print("Enter comma-separated numbers to set the list in order, 'cX' to set telemetry controller (e.g. c0), 'hN' to set Hz (e.g. h200), or 'q' to return.")
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
        if choice.startswith("h") and len(choice) > 1:
            try:
                hz_val = float(choice[1:])
                client.set_telemetry_rate_hz(hz_val)
                if hz_val <= 0:
                    print("Telemetry disabled.")
                else:
                    print(f"Set telemetry rate to {hz_val:g} Hz")
            except ValueError:
                print("Invalid Hz value.")
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
        state.telemetry = None
        print("Updated telemetry registers.")
