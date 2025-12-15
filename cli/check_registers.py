#!/usr/bin/env python3
"""
Quick sanity tool to read/write PacketCommander registers using the PySFOC library.

Usage examples:
  python check_registers.py --port /dev/ttyACM0
  python check_registers.py --port /dev/ttyACM0 --write vel_pid_p 0.2 --write vel_pid_i 4.0
  python check_registers.py --port /dev/ttyACM0 --regs control_mode,target,velocity
"""

import argparse
from typing import Dict, List

from pysfoc import PacketCommanderClient
from pysfoc.constants import (
    REGISTER_IDS,
    REG_NAME_MAP,
    REG_CONTROL_MODE,
    REG_VEL_PID_P,
    REG_VEL_PID_I,
    REG_VEL_PID_D,
)


# Expand the name->ID map with the extra telemetry-only entries present in REG_NAME_MAP
NAME_TO_REG: Dict[str, int] = {name: rid for name, rid in REGISTER_IDS.items()}
for reg_id, reg_name in REG_NAME_MAP.items():
    # Keep existing canonical names; add missing ones
    if reg_name not in NAME_TO_REG:
        NAME_TO_REG[reg_name] = reg_id


DEFAULT_REG_NAMES: List[str] = [
    "status",
    "control_mode",
    "torque_mode",
    "enable",
    "target",
    "velocity",
    "angle",
    "position",
    "sensor_mech_angle",
    "vel_pid_p",
    "vel_pid_i",
    "vel_pid_d",
    "velocity_limit",
    "voltage_limit",
    "driver_voltage_limit",
    "driver_voltage_psu",
    "pole_pairs",
]


def resolve_regs(reg_names: List[str]) -> List[int]:
    reg_ids: List[int] = []
    for name in reg_names:
        key = name.strip()
        if not key:
            continue
        if key not in NAME_TO_REG:
            raise ValueError(f"Unknown register name: {key}")
        reg_ids.append(NAME_TO_REG[key])
    return reg_ids


def main():
    parser = argparse.ArgumentParser(description="Read/write PacketCommander registers via PySFOC.")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM0)")
    parser.add_argument("--baud", type=int, default=460800, help="Baud rate (default 460800)")
    parser.add_argument(
        "--regs",
        default=",".join(DEFAULT_REG_NAMES),
        help="Comma-separated register names to read (default: a curated set).",
    )
    parser.add_argument(
        "--write",
        action="append",
        nargs=2,
        metavar=("REG", "VALUE"),
        help="Write a value before reading (can be repeated).",
    )
    parser.add_argument("--debug", action="store_true", help="Dump raw UART traffic for troubleshooting.")
    args = parser.parse_args()

    reg_names = [tok.strip() for tok in args.regs.split(",") if tok.strip()]
    reg_ids = resolve_regs(reg_names)

    client = PacketCommanderClient(args.port, args.baud, debug=args.debug)
    try:
        # Apply writes first so the subsequent read reflects them
        if args.write:
            for reg_name, value_str in args.write:
                reg_name = reg_name.strip()
                if reg_name not in NAME_TO_REG:
                    raise SystemExit(f"Unknown register name for write: {reg_name}")
                reg_id = NAME_TO_REG[reg_name]
                try:
                    # Interpret as float but allow ints too
                    if reg_id in (REG_CONTROL_MODE,):
                        val = int(float(value_str))
                    else:
                        val = float(value_str)
                except ValueError:
                    raise SystemExit(f"Invalid value for {reg_name}: {value_str}")
                echoed = client.write_reg(reg_id, val)
                print(f"Wrote {reg_name} (reg {reg_id}) = {val} (echo={echoed})")

        print("\nReading registers:")
        for name, reg_id in zip(reg_names, reg_ids):
            val = client.read_reg(reg_id)
            display = f"{val:.6f}" if isinstance(val, float) else str(val)
            print(f"  {name:20s} (reg {reg_id:02d}) = {display}")

        # Convenience: read current velocity PID set for quick verification
        p = client.read_reg(REG_VEL_PID_P)
        i = client.read_reg(REG_VEL_PID_I)
        d = client.read_reg(REG_VEL_PID_D)
        print(f"\nVelocity PID: P={p} I={i} D={d}")
    finally:
        client.close()


if __name__ == "__main__":
    main()
