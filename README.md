# SimpleFOC firmware for T-STorM32 NT Motor Module v2.51E

Firmware, bootloader, and host tools to run SimpleFOC on the [T-STorM32 NT Motor Module v2.51E](https://github.com/olliw42/storm32bgc/tree/main/storm32-nt/storm32-nt-motor-v25E) (STM32F103T8, DRV8313, TLE5012B encoder, UART on PA9/PA10).

## Hardware reference
- Board/pin map and quirks: `Hardware.md` (source of truth).
- MCU: STM32F103T8, 8 MHz HSE, 3.3 V IO.
- Driver: DRV8313 3-PWM (IN1/2/3 on PA3/PB0/PB1), always enabled (no fault/enable pins).
- Encoder: TLE5012B 3-wire SPI (PA5/PA6/PA7, CS=PA8).
- UART: PA9/PA10 @460800 for PacketCommander/Telemetry (app). Bootloader listens at 115200.

## Project layout
- `src/` – application firmware (PlatformIO env `tstorm32_simplefoc`), SimpleFOC + Arduino-FOC-drivers streams.
- `bootloader/` – minimal UART bootloader (env `tstorm32_bootloader`), LED fast blink in boot mode.
- `flash/` – host flashing tools:
  - `flash_bootloader.py` – flash bootloader via STLink (`st-flash`).
  - `flash_firmware.py` – flash app via UART bootloader or STLink. 
- `cli/` – `pysimplefoc_cli.py` menu-driven host CLI (settings editor + motion tests) using PacketCommander.
- `ldscripts/app_offset.ld` – app link script (app starts at 0x08002000; last 1 KB flash page reserved for settings).

## Building
Requires PlatformIO. Envs are defined in `platformio.ini`.

```bash
# Application
pio run -e tstorm32_simplefoc

# Bootloader
pio run -e tstorm32_bootloader
```

## Flashing
- **Bootloader (STLink):**
  ```bash
  python flash/flash_bootloader.py --bin .pio/build/tstorm32_bootloader/firmware.bin
  ```
- **App via STLink:**
  ```bash
  python flash/flash_firmware.py --stlink --bin .pio/build/tstorm32_simplefoc/firmware.bin --addr 0x08002000
  ```
- **App via UART bootloader (bootloader must already be installed):**
  ```bash
  python flash/flash_firmware.py --port /dev/tty.usbserial-XXXX --bin .pio/build/tstorm32_simplefoc/firmware.bin   # reset @460800, boot @115200
  ```

## Host CLI (settings + tests)
Interactive PacketCommander client in `cli/pysimplefoc_cli.py`:
```bash
cd cli
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python pysimplefoc_cli.py --port /dev/tty.usbserial-XXXX --baud 460800
```
Main menu:
- **Settings** – view/edit key registers (pole pairs, voltage/velocity limits, driver supply/limit, phase resistance, KV, velocity PID gains/LPF) and send `S1` to persist to flash.
- **Test** – curses UI for motion commands (full rotations, steps, slow run with speed adjust). `q` returns to menu.

## Persistence
A reserved flash page (0x0800FC00) stores selected motor/driver settings. `S1` packets (or the CLI “Save” action) invoke `save_settings_to_flash` in firmware. On boot the app prints `SETTINGS_LOADED` when valid data is applied, otherwise `SETTINGS_DEFAULT`.

## Notes
- Heartbeat/status LED is on PA1.
- Bootloader uses the same UART at 115200; slow blink indicates boot mode, solid/on-then-blink indicates app running.
- Always refer to `Hardware.md` for pin mappings and constraints. No FAULT/ENABLE lines and no current sensing are present on this hardware.
