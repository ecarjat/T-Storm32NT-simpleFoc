#pragma once

#include <SimpleFOC.h>
#include "runtime_settings.h"

// Wrapper sensor that applies a persisted calibration LUT (stored in flash).
class StoredCalibratedSensor : public Sensor {
public:
  explicit StoredCalibratedSensor(Sensor &wrapped);

  void setCalibrationData(SensorCalibrationData *data);

  void update() override;
  void init() override;
  float getSensorAngle() override;

private:
  Sensor &wrapped_;
  SensorCalibrationData *calibration_;
};

// Run calibration against the wrapped sensor and fill SensorCalibrationData (LUT + zero angle + direction).
// Returns true on success, false on failure (e.g. FOC alignment failed).
bool calibrate_sensor(Sensor &wrapped, BLDCMotor &motor, SensorCalibrationData &out, int settle_time_ms = 30);
