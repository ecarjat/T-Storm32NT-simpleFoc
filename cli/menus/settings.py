from __future__ import annotations

from typing import Optional

from pysfoc import PacketCommanderClient  # type: ignore[import-not-found]
from pysfoc.constants import (
    REG_NAME_MAP,
    REGISTER_IDS,
    REG_VEL_PID_P,
    REG_VEL_PID_I,
    REG_VEL_PID_D,
    REG_VEL_PID_LIMIT,
)  # type: ignore[import-not-found]


def reg_display_name(reg_id: int) -> str:
    return REG_NAME_MAP.get(reg_id, f"reg{reg_id}")


def settings_menu(client: PacketCommanderClient) -> None:
    settings_items = [
        (
            "Pole pairs",
            0x63,
            "Number of electrical pole pairs (integer).",
            "Motor pole pairs (electrical) used to convert between electrical and mechanical angle. Must match the motor; wrong value breaks commutation.",
        ),
        (
            "Voltage limit",
            0x50,
            "Motion control voltage limit (V).",
            "Motor-level voltage_limit used by SimpleFOC when commanding torque/velocity/angle. Lower to reduce torque/heat; raise carefully.",
        ),
        (
            "Velocity limit",
            0x52,
            "Max allowed shaft velocity (rad/s).",
            "Mechanical velocity clamp in rad/s for motion control. Protects the motor from over-speed; controller will not command faster than this.",
        ),
        (
            "Driver voltage limit",
            0x53,
            "Driver voltage limit (V).",
            "Driver voltage_limit (also mirrored to motor.voltage_limit). Typically same or slightly below supply. Keep conservative for safety.",
        ),
        (
            "Driver PSU voltage",
            0x55,
            "Supply voltage of the driver (V).",
            "Actual supply voltage powering the DRV8313 (voltage_power_supply). Set to your board supply so FOC scales outputs correctly.",
        ),
        (
            "Phase resistance",
            0x64,
            "Phase-to-phase resistance (Ohms).",
            "Motor phase resistance in Ohms. Used for monitoring/estimations; optional but improves telemetry accuracy.",
        ),
        (
            "KV rating",
            0x65,
            "Motor KV (RPM/V).",
            "Motor KV in RPM/V. Informational, used by some estimators; does not directly limit outputs but should be accurate if known.",
        ),
        (
            "Vel PID P",
            REG_VEL_PID_P,
            "Velocity loop P gain.",
            "Proportional gain of the velocity PID (PID_velocity.P). Too high causes oscillation; too low feels sluggish.",
        ),
        (
            "Vel PID I",
            REG_VEL_PID_I,
            "Velocity loop I gain.",
            "Integral gain of the velocity PID (PID_velocity.I). Removes steady-state error; too high causes windup/oscillation.",
        ),
        (
            "Vel PID D",
            REG_VEL_PID_D,
            "Velocity loop D gain.",
            "Derivative gain of the velocity PID (PID_velocity.D). Dampens fast changes/noise; often kept small or zero.",
        ),
        (
            "Vel PID limit",
            REG_VEL_PID_LIMIT,
            "Velocity PID output limit.",
            "Clamp for PID_velocity.limit (torque/voltage). Typically set to motor voltage_limit in voltage mode; reduce if tuning causes saturation.",
        ),
        (
            "Vel LPF Tf",
            0x35,
            "Velocity LPF time constant (s).",
            "Low-pass filter time constant for velocity (LPF_velocity.Tf). Larger = more smoothing/lag; smaller = more responsive/noisy.",
        ),
    ]
    while True:
        print("\nSettings:")
        for idx, (label, reg, short, _) in enumerate(settings_items, start=1):
            current = client.read_reg(reg)
            disp = f"{current:.4f}" if current is not None else "n/a"
            print(f"  {idx}) {label}: {disp}  - {short}")
        print("  s) Save to flash (S1)")
        print("  q) Back to main menu")
        choice = input("Select item to edit (number), 's' to save, or 'q': ").strip()
        if choice.lower() == "q":
            break
        if choice.lower() == "s":
            res = client.save_settings()
            if res is True:
                print("Save acknowledged (SAVE_OK).")
            elif res is False:
                print("Save failed (SAVE_ERR).")
            else:
                print("No save response received.")
            continue
        try:
            sel = int(choice)
        except ValueError:
            continue
        if sel < 1 or sel > len(settings_items):
            continue
        label, reg, _, long_help = settings_items[sel - 1]
        print(f"\n{label}: {long_help}")
        new_val = input(f"Enter new value for {label}: ").strip()
        try:
            val = float(new_val)
        except ValueError:
            print("Invalid number.")
            continue
        client.write_reg(reg, val)
        print(f"Set {label} to {val}")
