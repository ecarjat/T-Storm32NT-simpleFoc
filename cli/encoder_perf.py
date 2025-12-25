#!/usr/bin/env python3
"""
Encoder performance logger (BinaryIO only).

- Sets telemetry to [target, velocity, sensor_mech_angle]
- Sets telemetry rate to 500 Hz
- Puts motor in velocity mode, target = 5
- Runs for 30 seconds and logs every telemetry packet to CSV:
  time,target,velocity,angle

Console output: prints start/end only.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

from pysfoc import REG_VELOCITY, REG_TARGET, REG_SENSOR_MECH_ANGLE  # type: ignore[import-not-found]
from pysfoc.constants import REGISTER_IDS  # type: ignore[import-not-found]
from pysfoc.packet_commander import BinaryPacketCommanderClient  # type: ignore[import-not-found]


def main():
    parser = argparse.ArgumentParser(description="Capture encoder telemetry at 500 Hz for 30s and write CSV.")
    parser.add_argument("--port", required=True, help="Serial port (e.g. /dev/ttyACM0 or COM3)")
    parser.add_argument("--baud", type=int, default=460800, help="Baud rate (default 460800)")
    parser.add_argument("--duration", type=float, default=30.0, help="Capture duration in seconds (default 30)")
    parser.add_argument("--target", type=float, default=5.0, help="Velocity target (default 5)")
    parser.add_argument("--outfile", type=Path, default=Path("encoder_perf.csv"), help="CSV output path")
    args = parser.parse_args()

    client = BinaryPacketCommanderClient(args.port, args.baud, debug=False, log_packets=False)
    try:
        client.ser.reset_input_buffer()
    except Exception:
        pass

    # Configure telemetry: target, velocity, sensor mechanical angle
    regs = [REGISTER_IDS["target"], REGISTER_IDS["velocity"], REGISTER_IDS["sensor_mech_angle"]]
    client.set_telemetry_registers(regs)
    client.set_telemetry_rate_hz(500.0)
    client.write_reg(REGISTER_IDS["control_mode"], 1)  # velocity mode
    client.write_reg(REGISTER_IDS["enable"], 1)
    client.write_reg(REGISTER_IDS["target"], args.target)

    print(f"Starting capture: {args.duration:.1f}s @ 500 Hz -> {args.outfile}")
    start = time.time()
    deadline = start + args.duration
    rows = []
    try:
        while time.time() < deadline:
            telem = client.poll_telemetry()
            if not telem:
                time.sleep(0.001)
                continue
            t = telem.timestamp
            tgt = telem.values.get(REG_TARGET)
            vel = telem.values.get(REG_VELOCITY)
            ang = telem.values.get(REG_SENSOR_MECH_ANGLE)
            rows.append((t, tgt, vel, ang))
    finally:
        client.write_reg(REGISTER_IDS["enable"], 0)
        client.close()

    args.outfile.parent.mkdir(parents=True, exist_ok=True)
    with args.outfile.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "target", "velocity", "angle"])
        writer.writerows(rows)
    print(f"Done. Wrote {len(rows)} rows to {args.outfile}")


if __name__ == "__main__":
    if not sys.platform.startswith(("linux", "darwin", "win")):
        print("Warning: binary serial may not be supported on this platform.")
    main()
