#pragma once

#include <cstddef>
#include <cstdint>

// Reserve the last 3KB flash pages (64KB device) for settings.
// Bootloader is at 0x08000000-0x080017FF (6KB), app starts at 0x08001800, app size ~55KB,
// so 0x0800F400 should remain unused by app/bootloader.
constexpr uint32_t SETTINGS_ADDR = 0x0800F400UL;
constexpr uint32_t SETTINGS_MAGIC = 0x53544631; // "STF1"
constexpr uint32_t SETTINGS_VERSION = 7; // Bumped for new LUT format

// Motor and driver configuration placeholders.
// Tune these values when the target motor is known.
namespace motor_config {

// Sensor configuration (TLE5012B)
constexpr int32_t CPR = 32768;              // Counts per revolution for TLE5012B

// Calibration lookup table sizing (power of two for efficient index calculation)
// 1024 entries @ 2 bytes = 2048 bytes, requires 3KB flash for settings
constexpr size_t CAL_LUT_SIZE = 1024;       // LUT entries (power of two)
constexpr int32_t CAL_BIN_COUNTS = CPR / CAL_LUT_SIZE;  // Counts per LUT bin (32)

// Calibration process parameters
constexpr int CAL_N_POS = 5;                // Samples per electrical cycle
constexpr int CAL_N2_TICKS = 5;             // Microsteps between samples (smoothing)
constexpr int CAL_SETTLE_TIME_MS = 30;      // Settle delay per tick (ms)
constexpr float CAL_VOLTAGE = 3.0f;         // Calibration voltage (V)
constexpr int CAL_N_SAMPLES_PER_TICK = 10;  // Samples to average per tick position
constexpr int CAL_SAMPLE_DELAY_MS = 1;      // Delay between samples (ms)

// Multi-pass calibration parameters
constexpr int CAL_MAX_PASS_PAIRS = 3;       // Max forward+backward pass pairs (3F+3B = 6 total)
constexpr float CAL_STOP_RMS_COUNTS = 1.0f; // RMS threshold for early stop (counts)
constexpr float CAL_STOP_MAX_COUNTS = 3.0f; // Max diff threshold for early stop (counts)
constexpr int CAL_STOP_CONSECUTIVE = 2;     // Consecutive passes below threshold to stop

// Output precision reduction (post-LUT)
constexpr int CAL_DROP_BITS = 0;            // LSBs to drop from corrected counts (0/1/2)

// Driver parameters
constexpr unsigned long DRIVER_PWM_FREQUENCY = 25000; // Hz, adjust per DRV8313/efficiency

// Electrical parameters (update based on motor datasheet/measurement)
constexpr int POLE_PAIRS = 7;               // TODO: replace with actual pole pairs
constexpr float PHASE_RESISTANCE = 12.0f;    // Ohms, optional for monitoring
constexpr float KV_RATING = 130.0f;           // RPM/V, optional for reference

// Voltage configuration
constexpr float SUPPLY_VOLTAGE = DEF_POWER_SUPPLY;     // Input supply powering DRV8313
constexpr float DRIVER_VOLTAGE_LIMIT = DEF_POWER_SUPPLY; // nominal driver voltage limit
constexpr float MOTOR_VOLTAGE_LIMIT = 4.0f; // Conservative starting limit
constexpr float MOTOR_CURRENT_LIMIT = 2.0f; // Amps, adjust based on motor specs

// Control targets (safe defaults)
constexpr float VELOCITY_LIMIT = 60.0f;      // rad/s, adjust after tuning

// Velocity PID parameters
constexpr float V_PID_P = 0.2f;               // velocity PID P
constexpr float V_PID_I = 3.0f;              // velocity PID I
constexpr float V_PID_D = 0.0003f;               // velocity PID D
constexpr float V_LPF_TF = 0.018f;             // velocity low-pass filter time constant
constexpr float V_PID_VELOCITY_LIMIT = MOTOR_VOLTAGE_LIMIT; // velocity PID output clamp (voltage mode)
constexpr float V_OUTPUT_RAMP = 300.0f;       // voltage/sec output ramp rate

// AnglePID parameters
constexpr float A_PID_P = 15.0f;               // velocity PID P
constexpr float A_PID_I = 10.0f;              // velocity PID I
constexpr float A_PID_D = 0.0003f;               // velocity PID D
constexpr float A_LPF_TF = 0.0f;             // velocity low-pass filter time constant
constexpr float A_PID_OUTPUT_LIMIT = DEF_VEL_LIM; // velocity PID output clamp (default value)
constexpr float A_OUTPUT_RAMP = 0.0f;       // voltage/sec output ramp rate

constexpr float MOTION_DOWNSAMPLE = DEF_MOTION_DOWNSMAPLE;    // motion control 

} // namespace motor_config
