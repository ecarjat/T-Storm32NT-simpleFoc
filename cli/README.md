# PySimpleFOC CLI

Simple interactive client for the SimpleFOC PacketCommander/Telemetry UART on PA9/PA10.

## Setup

```bash
cd cli
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
python pysimplefoc_cli.py --port /dev/tty.usbmodemXXXX --baud 115200
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

## PySFOC host library

PID tuning and other host automation can reuse the lightweight PacketCommander wrapper in `cli/pysfoc/`:

```python
from pysfoc import PySFOCClient, REG_TARGET, REG_VELOCITY

client = PySFOCClient(port="/dev/ttyUSB0", baud=115200)
client.vel_ctrl_enable(True)                  # enable closed-loop velocity
client.vel_pid_set(0.1, 2.0, 0.01)            # push PID gains
client.vel_target_set(5.0)                    # command target velocity (rad/s)
client.telemetry_config(regs=[REG_TARGET, REG_VELOCITY], target_rate_hz=200)
sample = client.poll_velocity_sample()        # host timestamp + target/measured velocity
client.close()
```

The API mirrors the PacketCommander interface assumed in `AGENT_PID.md` (`VEL_PID_SET`, `VEL_TARGET_SET`, `VEL_CTRL_ENABLE`, `TELEMETRY_CONFIG`), and builds on the same register map as the CLI.
