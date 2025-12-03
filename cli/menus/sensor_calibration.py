from __future__ import annotations

import time

from pysfoc.packet_commander import PacketCommanderClient  
from pysfoc.api import MotorState  
from pysfoc.constants import REGISTER_IDS  


def sensor_calibration_menu(client: PacketCommanderClient, state: MotorState) -> None:
    """
    Trigger sensor calibration via PacketCommander:
    - Stop telemetry (R28=0).
    - Send C2 command.
    - Stream serial output until CAL_OK or CAL_SAVE_ERR is seen (or timeout).
    - Display LUT table output lines if present.
    """
    print("\nStarting sensor calibration (C2)...")
    try:
        client.write_reg(REGISTER_IDS["telemetry_downsample"], 0)
    except Exception:
        pass
    try:
        client.ser.reset_input_buffer()
    except Exception:
        pass
    client._write_line("C2")
    deadline = time.time() + 10.0
    result = None
    print("Waiting for calibration output (up to 10s):")
    while time.time() < deadline and result is None:
        line = client.ser.readline()
        if not line:
            continue
        try:
            text = line.decode("ascii", errors="ignore").strip()
        except Exception:
            continue
        if not text:
            continue
        print(f"  {text}")
        if "CAL_OK" in text:
            result = "CAL_OK"
        elif "CAL_SAVE_ERR" in text:
            result = "CAL_SAVE_ERR"
    if result is None:
        print("Calibration did not report completion before timeout.")
    elif result == "CAL_OK":
        print("Calibration successful (CAL_OK).")
    else:
        print("Calibration reported CAL_SAVE_ERR (save to flash failed).")
