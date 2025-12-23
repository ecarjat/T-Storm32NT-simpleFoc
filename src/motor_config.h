#pragma once

#include <cstddef>

// Reserve the last 1KB flash page (64KB device) for settings.
// Bootloader is at 0x08000000-0x08001FFF, app starts at 0x08002000, app size ~50KB,
// so 0x0800FC00 should remain unused by app/bootloader.
constexpr uint32_t SETTINGS_ADDR = 0x0800FC00UL;
constexpr uint32_t SETTINGS_MAGIC = 0x53544631; // "STF1"
constexpr uint32_t SETTINGS_VERSION = 5;

// Motor and driver configuration placeholders.
// Tune these values when the target motor is known.
namespace motor_config {

// Calibration lookup table sizing
constexpr size_t CAL_LUT_SIZE = 50;

// Driver parameters
constexpr unsigned long DRIVER_PWM_FREQUENCY = 25000; // Hz, adjust per DRV8313/efficiency

// Electrical parameters (update based on motor datasheet/measurement)
constexpr int POLE_PAIRS = 7;               // TODO: replace with actual pole pairs
constexpr float PHASE_RESISTANCE = 12.0f;    // Ohms, optional for monitoring
constexpr float KV_RATING = 130.0f;           // RPM/V, optional for reference

// Voltage configuration
constexpr float SUPPLY_VOLTAGE = 12.0f;     // Input supply powering DRV8313
constexpr float DRIVER_VOLTAGE_LIMIT = 12.0f; // Conservative starting limit
constexpr float MOTOR_VOLTAGE_LIMIT = 4.0f; // Conservative starting limit

// Control targets (safe defaults)
constexpr float VELOCITY_LIMIT = 60.0f;      // rad/s, adjust after tuning
constexpr float PID_P = 0.12f;               // velocity PID P
constexpr float PID_I = 7.0f;              // velocity PID I
constexpr float PID_D = 0.0005f;               // velocity PID D
constexpr float LPF_TF = 0.01f;             // velocity low-pass filter time constant
constexpr float PID_VELOCITY_LIMIT = MOTOR_VOLTAGE_LIMIT; // velocity PID output clamp (voltage mode)

constexpr float OUTPUT_RAMP = 300.0f;       // voltage/sec output ramp rate
constexpr float MOTION_DOWNSAMPLE = 5.0f;    // motion control

} // namespace motor_config
