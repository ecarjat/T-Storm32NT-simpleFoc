# Flash tools

- `flash_bootloader.py`: flash the bootloader via STLink (`st-flash write ... 0x08000000`).
- `flash_firmware.py`: update the application via the UART bootloader protocol.

## flash_firmware.py (UART or STLink)

Prereqs:
- Bootloader already flashed for UART mode.
- Pyserial installed (`pip install pyserial`) for UART mode.
- st-flash installed (`brew install stlink`) for STLink mode.

Usage:
```bash
cd flash
python flash_firmware.py --port /dev/ttyACM0 --baud 115200 --bin ../.pio/build/tstorm32_simplefoc/firmware.bin   # UART
python flash_firmware.py --stlink --bin ../.pio/build/tstorm32_simplefoc/firmware.bin --addr 0x08002000            # STLink
```

Flow:
1) Sends a reset-to-bootloader token (`BOOT\n`) to the running app (unless `--no-reset`).
2) Bootloader sees BKP magic and stays in update mode.
3) Host sends header: `UPD0` + `<len>` + `<crc32>` (little-endian), waits for `OK`.
4) Streams firmware bytes, waits for final `OK`.

Options:
- `--bin` path to app firmware (defaults to PlatformIO app build).
- `--no-reset` skips the BOOT command if you’ve already reset into the bootloader.
- `--baud` baud rate (default 115200).
- `--port` serial port (UART mode, required unless using --stlink).
- `--stlink` use st-flash instead of UART.
- `--stflash` path to st-flash (default `st-flash`).
- `--addr` flash address for st-flash (default 0x08002000).

## flash_bootloader.py (STLink)

Uses `st-flash` to program the bootloader binary to 0x08000000.

```bash
cd flash
python flash_bootloader.py --bin ../bootloader/.pio/build/tstorm32_bootloader/firmware.bin
```

Options:
- `--bin` path to bootloader `.bin` (default: PlatformIO bootloader build).
- `--addr` flash address (default 0x08000000).
- `--stflash` override `st-flash` executable.
