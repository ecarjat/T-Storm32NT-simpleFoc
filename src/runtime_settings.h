#pragma once

#include <SimpleFOC.h>
#include "motor_config.h"

// Runtime-settings container for parameters we want to persist across resets.
struct RuntimeSettings {
  int pole_pairs = motor_config::POLE_PAIRS;
  float phase_resistance = motor_config::PHASE_RESISTANCE;
  float kv_rating = motor_config::KV_RATING;
  float supply_voltage = motor_config::SUPPLY_VOLTAGE;
  float driver_voltage_limit = motor_config::DRIVER_VOLTAGE_LIMIT;
  float velocity_limit = motor_config::VELOCITY_LIMIT;
  float pid_p = motor_config::PID_P;
  float pid_i = motor_config::PID_I;
  float pid_d = motor_config::PID_D;
  float lpf_tf = motor_config::LPF_TF;
};

// Accessor for the singleton runtime settings.
RuntimeSettings &runtime_settings();

// Load settings from flash (if present/valid) into the runtime_settings().
bool load_settings_from_flash();

// Capture current motor/driver values into runtime_settings() and persist to flash.
bool save_settings_to_flash(const BLDCMotor &motor, const BLDCDriver3PWM &driver);

// Apply runtime_settings() values to motor/driver before init.
void apply_settings_to_objects(BLDCMotor &motor, BLDCDriver3PWM &driver);
