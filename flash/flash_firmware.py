#!/usr/bin/env python3
"""
Host-side updater for the firmware via either UART bootloader or STLink (st-flash).
UART protocol:
 1) Optionally send a reset-to-bootloader command (ASCII "B6\n") to the app (uses the firmware baud).
 2) Reopen the port at the bootloader baud, wait for "BOOT", then send "UPD0" + <len> + <crc32>.
 3) Bootloader replies "OK", accepts the firmware bytes, and replies "OK" on success.

STLink path:
 Uses st-flash to write the app binary directly to the app start address (default 0x08002000).

Examples:
  python flash_firmware.py --port /dev/ttyACM0 --bin ../.pio/build/tstorm32_simplefoc/firmware.bin            # UART (reset @460800, boot @115200)
  python flash_firmware.py --stlink --bin ../.pio/build/tstorm32_simplefoc/firmware.bin --addr 0x08002000     # STLink
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
RESET_CMD = b"B6\n"
DEFAULT_RESET_BAUD = 460800
DEFAULT_BOOTLOADER_BAUD = 115200
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
        if b"OK" in line or b"ACK" in line:
            return True
        if any(err in line for err in [b"ER", b"CRC", b"PFAIL", b"TIMEOUT", b"ERASE", b"ERCHK"]):
            sys.stderr.write(f"Bootloader error: {line!r}\n")
            return False
    sys.stderr.write(f"No OK before timeout ({timeout}s)\n")
    return False

def wait_for_boot(ser, timeout=10.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline()
        if not line:
            continue
        if b"BOOT" in line:
            print("Target reset to bootloader")
            return True
    sys.stderr.write(f"No BOOT/OK before timeout ({timeout}s)\n")
    return False


def flash(port, reset_baud, boot_baud, bin_path, reset_first):
    fw = read_bin(bin_path)
    length = len(fw)
    crc = binascii.crc32(fw) & 0xFFFFFFFF

    if reset_first:
        print(f"Requesting reset to bootloader @ {reset_baud}...")
        with serial.Serial(port, baudrate=reset_baud, timeout=0.5) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            send_reset(ser)
        # Reopen at bootloader baud to catch BOOT banner
        # time.sleep(0.2)

    # print(f"Connecting to bootloader on {port} @ {boot_baud}...")
    with serial.Serial(port, baudrate=boot_baud, timeout=0.5) as ser:
        # ser.reset_input_buffer()
        # ser.reset_output_buffer()
        if reset_first:
            time.sleep(0.2)
            # if not wait_for_boot(ser, timeout=5.0):
            #     sys.stderr.write("No BOOT/OK from target after reset\n")
            #     return False

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
        total_pages = (len(fw) + 255) // 256
        while offset < len(fw):
            chunk = fw[offset:offset+256]
            ser.write(chunk)
            ser.flush()
            page += 1
            # wait for ACK or error for this chunk
            if not wait_for_ok(ser, timeout=5.0):
                sys.stderr.write(f"No ACK/OK for chunk {page}\n")
                return False
            sys.stdout.write(f"\rSent {page}/{total_pages} pages")
            sys.stdout.flush()
            offset += len(chunk)
        sys.stdout.write("\n")

        if not wait_for_ok(ser, timeout=15.0):
            sys.stderr.write("No final OK from bootloader\n")
            return False

        print("Update complete.")
        return True


def main():
    parser = argparse.ArgumentParser(description="Flash firmware via UART bootloader or STLink")
    parser.add_argument("--port", help="Serial port (e.g. /dev/ttyACM0 or COM3) for UART mode")
    parser.add_argument(
        "--boot-baud",
        "--baud",
        dest="boot_baud",
        type=int,
        default=DEFAULT_BOOTLOADER_BAUD,
        help="Bootloader baud rate for transfer (default 115200)",
    )
    parser.add_argument(
        "--reset-baud",
        type=int,
        default=DEFAULT_RESET_BAUD,
        help="Firmware baud used to send BOOT reset command (default 460800)",
    )
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

    ok = flash(args.port, args.reset_baud, args.boot_baud, bin_path, reset_first=not args.no_reset)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
