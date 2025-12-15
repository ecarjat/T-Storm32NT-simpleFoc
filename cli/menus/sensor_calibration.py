from __future__ import annotations

import time

from pysfoc.packet_commander import PacketCommanderClient  # type: ignore[import-not-found]
from pysfoc.api import MotorState  # type: ignore[import-not-found]
from pysfoc.constants import REGISTER_IDS  # type: ignore[import-not-found]


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
    result = client.run_calibration(timeout=10.0)
    if result is True:
        print("Calibration successful (C2 => 1).")
    elif result is False:
        print("Calibration failed (C2 => 0).")
    else:
        print("Calibration did not report completion before timeout.")
