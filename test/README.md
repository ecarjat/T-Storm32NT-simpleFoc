# PySimpleFOC test client

Simple interactive client for the SimpleFOC PacketCommander/Telemetry UART on PA9/PA10.

## Setup

```bash
cd test
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
python pysimplefoc_app.py --port /dev/tty.usbmodemXXXX --baud 115200
```

## Controls (curses UI)

- `f` → full rotation CW (360°)
- `F` → full rotation CCW
- `s` → +10° step
- `S` → -10° step
- `r` → run forward slowly (velocity mode)
- `R` → run reverse slowly
- `SPACE` → stop motor (target = 0)
- `UP/DOWN` arrows → increase/decrease run speed while running
- `q` → quit

Notes:
- Assumes UART text PacketCommander (default firmware). Telemetry packets are ignored.
- If the firmware is configured sensorless (BOARD_USE_ENCODER=false), the app stays in velocity_openloop and uses timed moves for “full rotation”/steps.
- If an encoder is available, the app switches to angle control for precise moves.

