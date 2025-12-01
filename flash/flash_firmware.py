#!/usr/bin/env python3
"""
Host-side updater for the firmware via either UART bootloader or STLink (st-flash).
UART protocol:
 1) Optionally send a reset-to-bootloader command (ASCII "BOOT\n") to the app.
 2) Bootloader listens for "UPD0" + <uint32 length> + <uint32 crc32>, replies "OK",
    then accepts the firmware bytes, and replies "OK" on success.

STLink path:
 Uses st-flash to write the app binary directly to the app start address (default 0x08002000).

Examples:
  python flash_firmware.py --port /dev/ttyACM0 --bin ../.pio/build/tstorm32_simplefoc/firmware.bin
  python flash_firmware.py --stlink --bin ../.pio/build/tstorm32_simplefoc/firmware.bin --addr 0x08002000
"""
import argparse
import binascii
import os
import struct
import subprocess
import sys
import time

import serial  # pyserial


MAGIC = b"UPD0"
RESET_CMD = b"BOOT\n"
DEFAULT_BAUD = 115200
DEFAULT_APP_ADDR = "0x08002000"


def read_bin(path):
    with open(path, "rb") as f:
        return f.read()


def send_reset(ser):
    ser.write(RESET_CMD)
    ser.flush()


def wait_for_ok(ser, timeout=30.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline()
        if not line:
            continue
        print(f"bootloader said: {line!r}")
        if b"OK" in line or b"ACK" in line:
            return True
        if any(err in line for err in [b"ER", b"CRC", b"PFAIL", b"TIMEOUT", b"ERASE", b"ERCHK"]):
            sys.stderr.write(f"Bootloader error: {line!r}\n")
            return False
    sys.stderr.write(f"No OK before timeout ({timeout}s)\n")
    return False


def flash(port, baud, bin_path, reset_first):
    fw = read_bin(bin_path)
    length = len(fw)
    crc = binascii.crc32(fw) & 0xFFFFFFFF

    print(f"Connecting to {port} @ {baud}...")
    with serial.Serial(port, baudrate=baud, timeout=0.5) as ser:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        if reset_first:
            print("Requesting reset to bootloader...")
            send_reset(ser)
            time.sleep(1.0)

        header = MAGIC + struct.pack("<II", length, crc)
        print(f"Sending header: len={length}, crc=0x{crc:08X}")
        ser.write(header)
        ser.flush()

        if not wait_for_ok(ser, timeout=5.0):
            sys.stderr.write("No OK from bootloader after header/erase\n")
            return False
        else :
            print("Bootloader OK after header/erase, sending firmware...")

        print("Streaming firmware...")
        offset = 0
        page = 0
        while offset < len(fw):
            chunk = fw[offset:offset+256]
            ser.write(chunk)
            ser.flush()
            page += 1
            # wait for ACK or error for this chunk
            if not wait_for_ok(ser, timeout=5.0):
                sys.stderr.write(f"No ACK/OK for chunk {page}\n")
                return False
            print(f"Sent page {page}, bytes {offset + len(chunk)}/{len(fw)}")
            offset += len(chunk)

        if not wait_for_ok(ser, timeout=15.0):
            sys.stderr.write("No final OK from bootloader\n")
            return False

        print("Update complete.")
        return True


def main():
    parser = argparse.ArgumentParser(description="Flash firmware via UART bootloader or STLink")
    parser.add_argument("--port", help="Serial port (e.g. /dev/ttyACM0 or COM3) for UART mode")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate (default 115200)")
    parser.add_argument(
        "--bin",
        default=os.path.join("..", ".pio", "build", "tstorm32_simplefoc", "firmware.bin"),
        help="Path to app firmware .bin",
    )
    parser.add_argument("--no-reset", action="store_true", help="Do not send BOOT reset command first (UART)")
    parser.add_argument("--stlink", action="store_true", help="Use st-flash (STLink) instead of UART bootloader")
    parser.add_argument("--stflash", default="st-flash", help="st-flash executable (default: st-flash in PATH)")
    parser.add_argument("--addr", default=DEFAULT_APP_ADDR, help="Flash address for st-flash (default 0x08002000)")
    args = parser.parse_args()

    bin_path = os.path.abspath(args.bin)
    if not os.path.isfile(bin_path):
        sys.stderr.write(f"Firmware binary not found: {bin_path}\n")
        sys.exit(1)

    if args.stlink:
        cmd = [args.stflash, "write", bin_path, args.addr]
        print(f"Flashing via st-flash: {' '.join(cmd)}")
        try:
            subprocess.check_call(cmd)
            print("Done.")
            sys.exit(0)
        except subprocess.CalledProcessError as e:
            sys.stderr.write(f"st-flash failed with code {e.returncode}\n")
            sys.exit(e.returncode)

    if not args.port:
        sys.stderr.write("UART mode selected but --port not provided\n")
        sys.exit(1)

    ok = flash(args.port, args.baud, bin_path, reset_first=not args.no_reset)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
