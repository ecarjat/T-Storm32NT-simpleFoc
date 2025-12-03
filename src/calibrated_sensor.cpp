#include "calibrated_sensor.h"

#include <Arduino.h>
#include <cmath>
#include <vector>

#include "motor_config.h"

StoredCalibratedSensor::StoredCalibratedSensor(Sensor &wrapped)
    : wrapped_(wrapped), calibration_(nullptr) {}

void StoredCalibratedSensor::setCalibrationData(SensorCalibrationData *data) {
  calibration_ = data;
}

void StoredCalibratedSensor::update() {
  wrapped_.update();
  this->Sensor::update();
}

void StoredCalibratedSensor::init() {
  // assume wrapped sensor has already been initialized
  this->Sensor::init();
}

float StoredCalibratedSensor::getSensorAngle() {
  if (!calibration_ || !calibration_->valid || calibration_->lut_size == 0) {
    return wrapped_.getMechanicalAngle();
  }

  const float *lut = calibration_->lut;
  const int lut_size = static_cast<int>(calibration_->lut_size);

  float raw_angle = fmodf(wrapped_.getMechanicalAngle(), _2PI);
  if (raw_angle < 0) {
    raw_angle += _2PI;
  }

  const float lut_resolution = _2PI / lut_size;
  const int lut_index = static_cast<int>(raw_angle / lut_resolution);
  const float y0 = lut[lut_index];
  const float y1 = lut[(lut_index + 1) % lut_size];
  const float distance = (raw_angle - lut_index * lut_resolution) / lut_resolution;
  const float offset = (1.0f - distance) * y0 + distance * y1;

  return raw_angle - offset;
}

static void filter_error(std::vector<float> &error, float &error_mean, int n_ticks, int window) {
  std::vector<float> window_buffer(window, 0.0f);
  float window_sum = 0.0f;
  int buffer_index = 0;
  for (int i = 0; i < window; i++) {
    int ind = n_ticks - window / 2 - 1 + i;
    float v = error[ind % n_ticks];
    window_buffer[i] = v;
    window_sum += v;
  }

  error_mean = 0.0f;
  for (int i = 0; i < n_ticks; i++) {
    window_sum -= window_buffer[buffer_index];
    window_buffer[buffer_index] = error[(i + window / 2) % n_ticks];
    window_sum += window_buffer[buffer_index];
    buffer_index = (buffer_index + 1) % window;

    error[i] = window_sum / static_cast<float>(window);
    error_mean += error[i] / static_cast<float>(n_ticks);
  }
}

bool calibrate_sensor(Sensor &wrapped, BLDCMotor &motor, SensorCalibrationData &out, int settle_time_ms) {
  out.valid = false;
  out.lut_size = motor_config::CAL_LUT_SIZE;

  Print *dbg = motor.monitor_port;
  if (dbg) dbg->println("CAL: start");

  const int n_pos = 5;
  const int n_pp = motor.pole_pairs;
  const int n_ticks = n_pos * n_pp;
  const int n2_ticks = 5;
  const float deltaElectricalAngle = _2PI * n_pp / (n_ticks * n2_ticks);
  std::vector<float> error(n_ticks, 0.0f);
  const int window = n_pos;

  CurrentSense *current_sense = motor.current_sense;
  motor.current_sense = nullptr;
  motor.linkSensor(&wrapped);
  if (!motor.initFOC()) {
    if (dbg) dbg->println("CAL: initFOC failed");
    motor.current_sense = current_sense;
    return false;
  }
  motor.current_sense = current_sense;

  motor.setPhaseVoltage(1, 0, 0);
  _delay(1000);
  wrapped.update();
  float theta_init = wrapped.getAngle();
  float theta_absolute_init = wrapped.getMechanicalAngle();

  float elec_angle = 0.0f;
  float avg_elec_angle = 0.0f;
  float zero_angle_prev = 0.0f;

  if (dbg) dbg->println("CAL: forward");
  for (int i = 0; i < n_ticks; i++) {
    for (int j = 0; j < n2_ticks; j++) {
      wrapped.update();
      elec_angle += deltaElectricalAngle;
      motor.setPhaseVoltage(motor.voltage_sensor_align, 0, elec_angle);
    }
    _delay(settle_time_ms);
    wrapped.update();
    float theta_actual = static_cast<int>(motor.sensor_direction) * (wrapped.getAngle() - theta_init);
    error[i] = 0.5f * (theta_actual - elec_angle / n_pp);

    float zero_angle = (static_cast<int>(motor.sensor_direction) * wrapped.getMechanicalAngle() * n_pp) - (elec_angle + _PI_2);
    zero_angle = _normalizeAngle(zero_angle);
    if (zero_angle - zero_angle_prev > _PI) {
      zero_angle -= _2PI;
    } else if (zero_angle - zero_angle_prev < -_PI) {
      zero_angle += _2PI;
    }
    zero_angle_prev = zero_angle;
    avg_elec_angle += zero_angle / n_ticks;
  }
  _delay(2000);

  if (dbg) dbg->println("CAL: reverse");
  for (int i = n_ticks - 1; i >= 0; i--) {
    for (int j = 0; j < n2_ticks; j++) {
      wrapped.update();
      elec_angle -= deltaElectricalAngle;
      motor.setPhaseVoltage(motor.voltage_sensor_align, 0, elec_angle);
    }
    _delay(settle_time_ms);
    wrapped.update();
    float theta_actual = static_cast<int>(motor.sensor_direction) * (wrapped.getAngle() - theta_init);
    error[i] += 0.5f * (theta_actual - elec_angle / n_pp);

    float zero_angle = (static_cast<int>(motor.sensor_direction) * wrapped.getMechanicalAngle() * n_pp) - (elec_angle + _PI_2);
    zero_angle = _normalizeAngle(zero_angle);
    if (zero_angle - zero_angle_prev > _PI) {
      zero_angle -= _2PI;
    } else if (zero_angle - zero_angle_prev < -_PI) {
      zero_angle += _2PI;
    }
    zero_angle_prev = zero_angle;
    avg_elec_angle += zero_angle / n_ticks;
  }

  wrapped.update();
  float theta_absolute_post = wrapped.getMechanicalAngle();
  motor.setPhaseVoltage(0, 0, 0);

  float raw_offset = (theta_absolute_init + theta_absolute_post) / 2.0f;
  motor.zero_electric_angle = _normalizeAngle(avg_elec_angle / 2.0f);

  float error_mean = 0.0f;
  filter_error(error, error_mean, n_ticks, window);

  int index_offset = static_cast<int>(floorf(static_cast<float>(out.lut_size) * raw_offset / _2PI));
  float dn = n_ticks / static_cast<float>(out.lut_size);

  for (size_t i = 0; i < out.lut_size; i++) {
    int ind = index_offset + static_cast<int>(i) * static_cast<int>(motor.sensor_direction);
    if (ind > (static_cast<int>(out.lut_size) - 1)) ind -= out.lut_size;
    if (ind < 0) ind += out.lut_size;
    float val = error[static_cast<int>(i * dn)] - error_mean;
    out.lut[ind] = static_cast<int>(motor.sensor_direction) * val;
  }

  out.zero_electric_angle = motor.zero_electric_angle;
  out.direction = static_cast<int32_t>(motor.sensor_direction);
  out.valid = true;

  if (dbg) {
    dbg->print("CAL: done, zero=");
    dbg->println(out.zero_electric_angle, 6);
  }
  return true;
}
