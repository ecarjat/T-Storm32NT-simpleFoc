#pragma once

#include <cstddef>

// Reserve the last 1KB flash page (64KB device) for settings.
// Bootloader is at 0x08000000-0x08001FFF, app starts at 0x08002000, app size ~50KB,
// so 0x0800FC00 should remain unused by app/bootloader.
constexpr uint32_t SETTINGS_ADDR = 0x0800FC00UL;
constexpr uint32_t SETTINGS_MAGIC = 0x53544631; // "STF1"
constexpr uint32_t SETTINGS_VERSION = 4;

// Motor and driver configuration placeholders.
// Tune these values when the target motor is known.
namespace motor_config {

// Calibration lookup table sizing
constexpr size_t CAL_LUT_SIZE = 50;

// Electrical parameters (update based on motor datasheet/measurement)
constexpr int POLE_PAIRS = 7;               // TODO: replace with actual pole pairs
constexpr float PHASE_RESISTANCE = 8.0f;    // Ohms, optional for monitoring
constexpr float KV_RATING = 200.0f;           // RPM/V, optional for reference

// Voltage configuration
constexpr float SUPPLY_VOLTAGE = 12.6f;     // Input supply powering DRV8313
constexpr float DRIVER_VOLTAGE_LIMIT = 8.0f; // Conservative starting limit
constexpr float MOTOR_VOLTAGE_LIMIT = 4.0f; // Conservative starting limit

// Control targets (safe defaults)
constexpr float VELOCITY_LIMIT = 60.0f;      // rad/s, adjust after tuning
constexpr float PID_P = 0.15f;               // velocity PID P
constexpr float PID_I = 7.0f;              // velocity PID I
constexpr float PID_D = 0.0005f;               // velocity PID D
constexpr float LPF_TF = 0.08f;             // velocity low-pass filter time constant
constexpr float PID_LIMIT = MOTOR_VOLTAGE_LIMIT; // velocity PID output clamp (voltage mode)

} // namespace motor_config
