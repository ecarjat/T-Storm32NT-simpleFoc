#pragma once

// Motor and driver configuration placeholders.
// Tune these values when the target motor is known.
namespace motor_config {

// Electrical parameters (update based on motor datasheet/measurement)
constexpr int POLE_PAIRS = 7;               // TODO: replace with actual pole pairs
constexpr float PHASE_RESISTANCE = 0.0f;    // Ohms, optional for monitoring
constexpr float KV_RATING = 0.0f;           // RPM/V, optional for reference

// Voltage configuration
constexpr float SUPPLY_VOLTAGE = 12.0f;     // Input supply powering DRV8313
constexpr float DRIVER_VOLTAGE_LIMIT = 6.0f; // Conservative starting limit

// Control targets (safe defaults)
constexpr float VELOCITY_LIMIT = 5.0f;      // rad/s, adjust after tuning
constexpr float PID_P = 0.5f;               // velocity PID P
constexpr float PID_I = 20.0f;              // velocity PID I
constexpr float PID_D = 0.0f;               // velocity PID D
constexpr float LPF_TF = 0.01f;             // velocity low-pass filter time constant

} // namespace motor_config

