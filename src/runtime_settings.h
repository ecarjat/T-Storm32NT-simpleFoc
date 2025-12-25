#pragma once

#include <SimpleFOC.h>
#include <cstdint>
#include "motor_config.h"

// Persistent calibration payload (stored in flash).
// LUT stores correction offsets in COUNTS (int16_t), not radians.
// To apply: corrected_counts = wrapCounts(raw_counts - lut[idx])
// NOTE: No default initializers to keep struct in .bss (not .data), saving ~2KB flash.
struct SensorCalibrationData {
  bool valid;
  uint16_t lut_size;
  float zero_electric_angle;
  int32_t direction;
  int16_t lut_counts[motor_config::CAL_LUT_SIZE];
};

// Runtime-settings container for parameters we want to persist across resets.
// NOTE: No default initializers - use init_defaults() to set values.
// This keeps the struct in .bss instead of .data, saving ~2KB flash.
struct RuntimeSettings {
  float motor_voltage_limit;
  float motor_current_limit;
  float velocity_limit;
  float driver_voltage_limit;
  int pole_pairs;
  float phase_resistance;
  float kv_rating;
  float supply_voltage;
  float motion_downsample;

  // Velocity PID parameters
  float v_pid_p;
  float v_pid_i;
  float v_pid_d;
  float v_pid_velocity_limit;
  float v_lpf_tf;
  float v_output_ramp;

  // Angle PID parameters
  float a_pid_p;
  float a_pid_i;
  float a_pid_d;
  float a_pid_output_limit;
  float a_lpf_tf;
  float a_output_ramp;

  SensorCalibrationData calibration;
};

// Accessor for the singleton runtime settings.
RuntimeSettings &runtime_settings();

// Initialize runtime_settings() with default values from motor_config.h.
// Call this before load_settings_from_flash() or if flash load fails.
void init_settings_defaults();

// Load settings from flash (if present/valid) into the runtime_settings().
// Returns true if valid settings were loaded, false otherwise.
// If false, caller should call init_settings_defaults().
bool load_settings_from_flash();

// Capture current motor/driver values into runtime_settings() and persist to flash.
bool save_settings_to_flash(const BLDCMotor &motor, const BLDCDriver3PWM &driver);

// Update calibration payload in runtime_settings().
void set_calibration_data(const SensorCalibrationData &data);

// Clear calibration data and mark invalid.
void clear_calibration_data();

// Apply runtime_settings() values to motor/driver before init.
void apply_settings_to_objects(BLDCMotor &motor, BLDCDriver3PWM &driver);
