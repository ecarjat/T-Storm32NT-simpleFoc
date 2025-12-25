#pragma once

#include <SimpleFOC.h>
#include <cstdint>
#include "motor_config.h"

// Persistent calibration payload (stored in flash).
// LUT stores correction offsets in COUNTS (int16_t), not radians.
// To apply: corrected_counts = wrapCounts(raw_counts - lut[idx])
struct SensorCalibrationData {
  bool valid = false;
  uint16_t lut_size = motor_config::CAL_LUT_SIZE;  // Should be 1024
  float zero_electric_angle = 0.0f;
  int32_t direction = static_cast<int32_t>(Direction::CW);
  int16_t lut_counts[motor_config::CAL_LUT_SIZE] = {};  // Offset in counts (int16_t)
};

// Runtime-settings container for parameters we want to persist across resets.
struct RuntimeSettings {
  float motor_voltage_limit = motor_config::MOTOR_VOLTAGE_LIMIT;
  float motor_current_limit = motor_config::MOTOR_CURRENT_LIMIT;
  float velocity_limit = motor_config::VELOCITY_LIMIT;
  float driver_voltage_limit = motor_config::DRIVER_VOLTAGE_LIMIT;
  int pole_pairs = motor_config::POLE_PAIRS;
  float phase_resistance = motor_config::PHASE_RESISTANCE;
  float kv_rating = motor_config::KV_RATING;
  float supply_voltage = motor_config::SUPPLY_VOLTAGE;
  float motion_downsample = motor_config::MOTION_DOWNSAMPLE;
  
  // Velocity PID parameters
  float v_pid_p = motor_config::V_PID_P;
  float v_pid_i = motor_config::V_PID_I;
  float v_pid_d = motor_config::V_PID_D;
  float v_pid_velocity_limit = motor_config::V_PID_VELOCITY_LIMIT;
  float v_lpf_tf = motor_config::V_LPF_TF;
  float v_output_ramp = motor_config::V_OUTPUT_RAMP;

  // Angle PID parameters
  float a_pid_p = motor_config::A_PID_P;
  float a_pid_i = motor_config::A_PID_I;
  float a_pid_d = motor_config::A_PID_D;
  float a_pid_output_limit = motor_config::A_PID_OUTPUT_LIMIT;
  float a_lpf_tf = motor_config::A_LPF_TF;
  float a_output_ramp = motor_config::A_OUTPUT_RAMP;


  SensorCalibrationData calibration{};
};

// Accessor for the singleton runtime settings.
RuntimeSettings &runtime_settings();

// Load settings from flash (if present/valid) into the runtime_settings().
bool load_settings_from_flash();

// Capture current motor/driver values into runtime_settings() and persist to flash.
bool save_settings_to_flash(const BLDCMotor &motor, const BLDCDriver3PWM &driver);

// Update calibration payload in runtime_settings().
void set_calibration_data(const SensorCalibrationData &data);

// Clear calibration data and mark invalid.
void clear_calibration_data();

// Apply runtime_settings() values to motor/driver before init.
void apply_settings_to_objects(BLDCMotor &motor, BLDCDriver3PWM &driver);
