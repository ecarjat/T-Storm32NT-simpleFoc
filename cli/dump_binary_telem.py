#!/usr/bin/env python3
"""
Small helper to read BinaryIO (0xA5-framed) telemetry directly from the UART
and print decoded register values.

Usage:
  python cli/dump_binary_telem.py --port /dev/ttyACM0 [--baud 460800] [--regs target,velocity,angle,status] [--downsample 10]
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import List

from pysfoc import BinaryPacketCommanderClient  # type: ignore[import-not-found]
from pysfoc.constants import REGISTER_IDS, REG_NAME_MAP  # type: ignore[import-not-found]


DEFAULT_REG_LIST = [REGISTER_IDS["target"], REGISTER_IDS["velocity"], REGISTER_IDS["angle"], REGISTER_IDS["status"], REGISTER_IDS["enable"]]


def parse_reg_list(reg_str: str) -> List[int]:
    regs: List[int] = []
    tokens = [tok.strip() for tok in reg_str.split(",") if tok.strip()]
    name_to_id = {name.lower(): reg for reg, name in REG_NAME_MAP.items()}
    for tok in tokens:
        # Allow hex (0x..), decimal, or register name
        low = tok.lower()
        if low in name_to_id:
            regs.append(name_to_id[low])
            continue
        try:
            if low.startswith("0x"):
                regs.append(int(low, 16))
            else:
                regs.append(int(low))
        except ValueError:
            raise argparse.ArgumentTypeError(f"Unknown register token: {tok}")
    return regs


def main():
    parser = argparse.ArgumentParser(description="Dump BinaryIO PacketCommander telemetry as decoded values.")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM0 or COM3)")
    parser.add_argument("--baud", type=int, default=460800, help="Baud rate (default 460800)")
    parser.add_argument(
        "--regs",
        type=parse_reg_list,
        default=DEFAULT_REG_LIST,
        help="Comma-separated register names/IDs to stream (default: target,velocity,angle,status,enable)",
    )
    parser.add_argument("--downsample", type=int, default=10, help="Telemetry downsample value (default 10)")
    parser.add_argument("--controller", type=int, default=0, help="Telemetry controller index (default 0)")
    parser.add_argument("--debug-frames", action="store_true", help="Print BinaryIO frame directions for troubleshooting")
    parser.add_argument("--rearm-seconds", type=float, default=2.0, help="If no telemetry for this many seconds, reapply register list")
    parser.add_argument("--quiet", action="store_true", help="Only print values; suppress connection banner")
    parser.add_argument("--listen-only", action="store_true", help="Do not configure telemetry; just parse incoming frames using the provided reg list")
    parser.add_argument("--log-only", action="store_true", help="Only display BinaryIO log packets (type 'L'); ignore telemetry output")
    args = parser.parse_args()

    if args.log_only:
        args.listen_only = True
    client = BinaryPacketCommanderClient(args.port, args.baud, debug=args.debug_frames, log_packets=args.log_only)
    try:
        client.ser.reset_input_buffer()
    except Exception:
        pass

    if not args.listen_only:
        # Configure telemetry similarly to sensor_plot: set registers, downsample, and controller.
        client.set_telemetry_registers(args.regs)
        client.write_reg(REGISTER_IDS["telemetry_downsample"], max(0, args.downsample))
        client.write_reg(REGISTER_IDS["telemetry_ctrl"], args.controller)

    reg_names = {reg: REG_NAME_MAP.get(reg, f"reg{reg}") for reg in args.regs}

    if not args.quiet:
        if args.log_only:
            print(f"Logs on {args.port} @ {args.baud} baud")
        else:
            print(f"Telemetry on {args.port} @ {args.baud} baud")
            print("Registers:", ", ".join(f"{reg_names[r]}({r})" for r in args.regs))
        print("Press Ctrl-C to stop.\n")

    try:
        last_telem = time.monotonic()
        while True:
            sample = client.poll_telemetry()
            if not sample:
                if not args.listen_only and (time.monotonic() - last_telem > args.rearm_seconds):
                    # Re-apply telemetry config in case the MCU stopped streaming.
                    client.set_telemetry_registers(args.regs)
                    client.write_reg(REGISTER_IDS["telemetry_downsample"], max(0, args.downsample))
                    client.write_reg(REGISTER_IDS["telemetry_ctrl"], args.controller)
                    if not args.quiet:
                        print(f"[rearm {time.monotonic():.3f}] reapplied regs={args.regs} downsample={args.downsample} ctrl={args.controller}")
                    last_telem = time.monotonic()
                time.sleep(0.01)
                continue
            if args.log_only:
                last_telem = time.monotonic()
                continue
            last_telem = time.monotonic()
            parts: List[str] = []
            for motor_idx, reg in sample.regs:
                if motor_idx != 0:
                    continue
                name = reg_names.get(reg, f"reg{reg}")
                val = sample.values.get(reg)
                if isinstance(val, list):
                    disp = "/".join(f"{v:.4f}" if isinstance(v, (float, int)) else str(v) for v in val)
                elif isinstance(val, float):
                    disp = f"{val:.4f}"
                else:
                    disp = str(val)
                parts.append(f"{name}={disp}")
            if parts:
                print(f"{sample.timestamp:.3f} " + "  ".join(parts))
    except KeyboardInterrupt:
        pass
    finally:
        client.close()


if __name__ == "__main__":
    if not sys.platform.startswith(("linux", "darwin", "win")):
        print("Warning: binary serial may not be supported on this platform.")
    main()
