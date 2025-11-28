#!/usr/bin/env python3
"""
Flash the bootloader binary to STM32F103 via st-flash (STLink).
Default assumes PlatformIO bootloader build at:
  bootloader/.pio/build/tstorm32_bootloader/firmware.bin
"""
import argparse
import os
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description="Flash bootloader via st-flash (STLink)")
    parser.add_argument(
        "--bin",
        default=os.path.join("..", "bootloader", ".pio", "build", "tstorm32_bootloader", "firmware.bin"),
        help="Path to bootloader .bin (default: ../bootloader/.pio/build/tstorm32_bootloader/firmware.bin)",
    )
    parser.add_argument(
        "--addr",
        default="0x08000000",
        help="Flash start address (default: 0x08000000)",
    )
    parser.add_argument(
        "--stflash",
        default="st-flash",
        help="st-flash executable (default: st-flash in PATH)",
    )
    args = parser.parse_args()

    bin_path = os.path.abspath(args.bin)
    if not os.path.isfile(bin_path):
        sys.stderr.write(f"Bootloader binary not found: {bin_path}\n")
        sys.exit(1)

    cmd = [args.stflash, "write", bin_path, args.addr]
    print(f"Flashing bootloader: {' '.join(cmd)}")
    try:
        subprocess.check_call(cmd)
    except subprocess.CalledProcessError as e:
        sys.stderr.write(f"st-flash failed with code {e.returncode}\n")
        sys.exit(e.returncode)

    print("Done.")


if __name__ == "__main__":
    main()

