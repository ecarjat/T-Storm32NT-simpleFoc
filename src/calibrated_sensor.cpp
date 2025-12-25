#include "calibrated_sensor.h"

#include <Arduino.h>
#include <cmath>
#include <cstring>

// ============================================================================
// Robust statistics helpers
// ============================================================================

// Insertion sort for small arrays
static void insertion_sort_int32(int32_t* arr, int n) {
  for (int i = 1; i < n; i++) {
    int32_t key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

int32_t median_int32(int32_t* arr, int n) {
  if (n <= 0) return 0;
  insertion_sort_int32(arr, n);
  if (n % 2 == 1) {
    return arr[n / 2];
  } else {
    return (arr[n / 2 - 1] + arr[n / 2]) / 2;
  }
}

int32_t trimmed_mean_int32(int32_t* arr, int n, int k) {
  if (n <= 0) return 0;
  if (n <= 2 * k) {
    // Not enough samples to trim, just return median
    return median_int32(arr, n);
  }
  insertion_sort_int32(arr, n);
  int64_t sum = 0;
  int count = n - 2 * k;
  for (int i = k; i < n - k; i++) {
    sum += arr[i];
  }
  return static_cast<int32_t>(sum / count);
}

// ============================================================================
// StoredCalibratedSensor implementation
// ============================================================================

StoredCalibratedSensor::StoredCalibratedSensor(Sensor& wrapped)
    : wrapped_(wrapped), calibration_(nullptr) {}

void StoredCalibratedSensor::setCalibrationData(SensorCalibrationData* data) {
  calibration_ = data;
}

void StoredCalibratedSensor::update() {
  wrapped_.update();
  this->Sensor::update();
}

void StoredCalibratedSensor::init() {
  // Assume wrapped sensor has already been initialized
  this->Sensor::init();
}

int32_t StoredCalibratedSensor::applyCorrectionCounts(int32_t raw_counts) const {
  if (!calibration_ || !calibration_->valid || calibration_->lut_size == 0) {
    return raw_counts;
  }

  constexpr int32_t lut_n = motor_config::CAL_LUT_SIZE;
  constexpr int32_t bin_counts = motor_config::CAL_BIN_COUNTS;
  const int16_t* lut = calibration_->lut_counts;

  // Ensure raw_counts is in [0, CPR-1]
  raw_counts = wrapCounts(raw_counts);

  // Calculate LUT index and fractional position
  int32_t idx = raw_counts / bin_counts;
  int32_t frac = raw_counts - idx * bin_counts;  // 0 to bin_counts-1
  int32_t next_idx = (idx + 1) & (lut_n - 1);    // Power-of-two wrap

  // Get LUT offsets
  int32_t o0 = lut[idx];
  int32_t o1 = lut[next_idx];

  // Linear interpolation: offset = o0 + (o1 - o0) * frac / bin_counts
  int32_t offset = o0 + ((o1 - o0) * frac) / bin_counts;

  // Apply correction
  int32_t corrected = wrapCounts(raw_counts - offset);

  return corrected;
}

float StoredCalibratedSensor::getSensorAngle() {
  constexpr int32_t cpr = motor_config::CPR;

  // Get raw mechanical angle from wrapped sensor
  float raw_angle = fmodf(wrapped_.getMechanicalAngle(), _2PI);
  if (raw_angle < 0) raw_angle += _2PI;

  // Convert to counts
  int32_t raw_counts = static_cast<int32_t>(raw_angle * (cpr / _2PI));
  raw_counts = wrapCounts(raw_counts);

  // Store as last valid counts (for SPI error fallback)
  last_valid_counts_ = raw_counts;

  // Apply LUT correction
  int32_t corrected_counts = applyCorrectionCounts(raw_counts);

  // Apply optional precision reduction (drop LSBs)
  if (drop_bits_ > 0) {
    int32_t step = 1 << drop_bits_;
    corrected_counts = (corrected_counts / step) * step;
    corrected_counts = wrapCounts(corrected_counts);
  }

  // Convert back to radians
  return countsToRad(corrected_counts);
}

// ============================================================================
// Calibration implementation
// ============================================================================

// Filter error array with moving average (FIR filter at electrical frequency)
static void filter_error(
    float* error,
    float* filtered,
    float& error_mean,
    int n_ticks,
    int window,
    float* window_buffer
) {
  memset(window_buffer, 0, window * sizeof(float));
  float window_sum = 0.0f;
  int buffer_index = 0;

  // Fill initial window buffer (wrap around from end of array)
  for (int i = 0; i < window; i++) {
    int ind = n_ticks - window / 2 - 1 + i;
    if (ind < 0) ind += n_ticks;
    ind = ind % n_ticks;
    window_buffer[i] = error[ind];
    window_sum += window_buffer[i];
  }

  // Calculate moving average
  error_mean = 0.0f;
  for (int i = 0; i < n_ticks; i++) {
    // Update buffer
    window_sum -= window_buffer[buffer_index];
    int source_idx = (i + window / 2) % n_ticks;
    window_buffer[buffer_index] = error[source_idx];
    window_sum += window_buffer[buffer_index];
    buffer_index = (buffer_index + 1) % window;

    // Store filtered value
    filtered[i] = window_sum / static_cast<float>(window);
    error_mean += filtered[i] / static_cast<float>(n_ticks);
  }
}

// Build LUT from filtered error array
static void build_lut_counts(
    const float* filtered_error,
    float error_mean,
    int n_ticks,
    int lut_size,
    float raw_offset_rad,
    int sensor_direction,
    int16_t* lut_out
) {
  // Calculate index offset from raw position
  int index_offset = static_cast<int>(floorf(
      static_cast<float>(lut_size) * raw_offset_rad / _2PI
  ));

  float dn = static_cast<float>(n_ticks) / static_cast<float>(lut_size);

  for (int i = 0; i < lut_size; i++) {
    // Calculate destination index with direction
    int ind = index_offset + i * sensor_direction;
    // Wrap to LUT range
    ind = ind % lut_size;
    if (ind < 0) ind += lut_size;

    // Get source error value
    int source_idx = static_cast<int>(i * dn);
    if (source_idx >= n_ticks) source_idx = n_ticks - 1;

    // Convert error (radians) to offset (counts)
    float error_rad = (filtered_error[source_idx] - error_mean) * sensor_direction;
    int32_t offset_counts = radToCounts(error_rad);

    // Clamp to int16_t range (should be well within range)
    if (offset_counts > 32767) offset_counts = 32767;
    if (offset_counts < -32768) offset_counts = -32768;

    lut_out[ind] = static_cast<int16_t>(offset_counts);
  }
}

// Compute RMS and max difference between two LUTs
static void compute_lut_diff(
    const int16_t* lut_a,
    const int16_t* lut_b,
    int lut_size,
    float& rms_out,
    float& max_abs_out
) {
  float sum_sq = 0.0f;
  float max_abs = 0.0f;

  for (int i = 0; i < lut_size; i++) {
    float diff = static_cast<float>(lut_a[i] - lut_b[i]);
    sum_sq += diff * diff;
    float abs_diff = fabsf(diff);
    if (abs_diff > max_abs) max_abs = abs_diff;
  }

  rms_out = sqrtf(sum_sq / lut_size);
  max_abs_out = max_abs;
}

// Single pass (forward or backward) of calibration
static void run_calibration_pass(
    Sensor& wrapped,
    BLDCMotor& motor,
    float* tick_error,
    float& avg_elec_angle,
    float& elec_angle,
    float& zero_angle_prev,
    float theta_init,
    int n_ticks,
    int n2_ticks,
    float delta_electrical_angle,
    float voltage,
    int settle_time_ms,
    int n_samples_per_tick,
    int sample_delay_ms,
    int32_t* sample_buffer,
    bool forward,
    Print* dbg
) {
  int n_pp = motor.pole_pairs;
  int dir_mult = forward ? 1 : -1;

  int start = forward ? 0 : (n_ticks - 1);
  int end = forward ? n_ticks : -1;
  int step = forward ? 1 : -1;

  for (int i = start; i != end; i += step) {
    // Move to next position with microsteps
    for (int j = 0; j < n2_ticks; j++) {
      wrapped.update();
      elec_angle += dir_mult * delta_electrical_angle;
      motor.setPhaseVoltage(voltage, 0, elec_angle);
    }

    // Settle delay
    _delay(settle_time_ms);

    // Collect multiple samples for robust aggregation
    int valid_samples = 0;
    int attempts = 0;
    int max_attempts = n_samples_per_tick * 3;

    while (valid_samples < n_samples_per_tick && attempts < max_attempts) {
      wrapped.update();

      // Get current angle reading
      float theta_actual = static_cast<int>(motor.sensor_direction) *
                           (wrapped.getAngle() - theta_init);
      float error_rad = theta_actual - elec_angle / n_pp;

      // Convert to signed counts for storage
      int32_t error_counts = radToCounts(error_rad);
      error_counts = wrapCountsSigned(error_counts);

      // TODO: Add SPI CRC/safety check here when available
      // For now, accept all samples
      sample_buffer[valid_samples] = error_counts;
      valid_samples++;

      if (valid_samples < n_samples_per_tick) {
        _delay(sample_delay_ms);
      }
      attempts++;
    }

    // Robust aggregation: use median of samples
    int32_t median_error_counts = 0;
    if (valid_samples > 0) {
      median_error_counts = median_int32(sample_buffer, valid_samples);
    }

    // Accumulate error (will be averaged across passes later)
    // Each pass contributes 0.5 weight (forward + backward = 1.0)
    tick_error[i] += 0.5f * countsToRad(median_error_counts);

    // Calculate zero angle for averaging
    wrapped.update();
    float zero_angle = (static_cast<int>(motor.sensor_direction) *
                        wrapped.getMechanicalAngle() * n_pp) -
                       (elec_angle + _PI_2);
    zero_angle = _normalizeAngle(zero_angle);

    // Remove 2PI jumps
    if (zero_angle - zero_angle_prev > _PI) {
      zero_angle -= _2PI;
    } else if (zero_angle - zero_angle_prev < -_PI) {
      zero_angle += _2PI;
    }
    zero_angle_prev = zero_angle;
    avg_elec_angle += zero_angle / n_ticks;
  }
}

bool calibrate_sensor(
    Sensor& wrapped,
    BLDCMotor& motor,
    SensorCalibrationData& out,
    CalibWorkspace& ws,
    const CalibrationConfig& config
) {
  out.valid = false;
  out.lut_size = motor_config::CAL_LUT_SIZE;

  Print* dbg = motor.monitor_port;
  if (dbg) dbg->println("CAL: start (multi-pass robust calibration)");

  const int n_pp = motor.pole_pairs;
  const int n_ticks = config.n_pos * n_pp;

  if (n_ticks > CAL_MAX_TICKS) {
    if (dbg) dbg->println("CAL: ERROR - too many ticks for workspace");
    return false;
  }

  const float delta_electrical_angle = _2PI * n_pp /
                                       (static_cast<float>(n_ticks) * config.n2_ticks);

  // Initialize workspace arrays
  memset(ws.tick_error_rad, 0, sizeof(ws.tick_error_rad));
  memset(ws.lut_prev, 0, sizeof(ws.lut_prev));
  memset(ws.lut_curr, 0, sizeof(ws.lut_curr));

  // Temporarily unlink current sense for alignment
  CurrentSense* current_sense = motor.current_sense;
  motor.current_sense = nullptr;
  motor.linkSensor(&wrapped);

  if (!motor.initFOC()) {
    if (dbg) dbg->println("CAL: initFOC failed");
    motor.current_sense = current_sense;
    return false;
  }
  motor.current_sense = current_sense;

  // Set initial position and record initial angles
  motor.setPhaseVoltage(1, 0, 0);
  _delay(1000);
  wrapped.update();
  float theta_init = wrapped.getAngle();
  float theta_absolute_init = wrapped.getMechanicalAngle();

  int consecutive_good = 0;
  float final_avg_elec_angle = 0.0f;
  float theta_absolute_post = theta_absolute_init;

  // Multi-pass calibration with convergence checking
  for (int pass_pair = 0; pass_pair < config.max_pass_pairs; pass_pair++) {
    if (dbg) {
      dbg->print("CAL: pass pair ");
      dbg->print(pass_pair + 1);
      dbg->print("/");
      dbg->println(config.max_pass_pairs);
    }

    // Reset tick error for this pass pair
    memset(ws.pass_tick_error, 0, sizeof(ws.pass_tick_error));

    float elec_angle = 0.0f;
    float avg_elec_angle = 0.0f;
    float zero_angle_prev = 0.0f;

    // Forward pass
    if (dbg) dbg->println("CAL: forward");
    run_calibration_pass(
        wrapped, motor, ws.pass_tick_error, avg_elec_angle,
        elec_angle, zero_angle_prev, theta_init, n_ticks,
        config.n2_ticks, delta_electrical_angle, config.voltage,
        config.settle_time_ms, config.n_samples_per_tick,
        config.sample_delay_ms, ws.sample_counts, true, dbg
    );

    _delay(2000);

    // Backward pass
    if (dbg) dbg->println("CAL: backward");
    run_calibration_pass(
        wrapped, motor, ws.pass_tick_error, avg_elec_angle,
        elec_angle, zero_angle_prev, theta_init, n_ticks,
        config.n2_ticks, delta_electrical_angle, config.voltage,
        config.settle_time_ms, config.n_samples_per_tick,
        config.sample_delay_ms, ws.sample_counts, false, dbg
    );

    // Accumulate pass results into main tick_error array
    // (averaging across passes)
    float pass_weight = 1.0f / (pass_pair + 1);
    for (int i = 0; i < n_ticks; i++) {
      // Running average: new_avg = old_avg + (new_val - old_avg) / n
      ws.tick_error_rad[i] += (ws.pass_tick_error[i] - ws.tick_error_rad[i]) * pass_weight;
    }

    // Store final angles
    final_avg_elec_angle = avg_elec_angle;
    wrapped.update();
    theta_absolute_post = wrapped.getMechanicalAngle();

    // Filter and build LUT
    float error_mean = 0.0f;
    filter_error(ws.tick_error_rad, ws.filtered_error_rad, error_mean,
                 n_ticks, config.n_pos, ws.window_buffer);

    float raw_offset = (theta_absolute_init + theta_absolute_post) / 2.0f;

    build_lut_counts(
        ws.filtered_error_rad, error_mean, n_ticks,
        motor_config::CAL_LUT_SIZE, raw_offset,
        static_cast<int>(motor.sensor_direction), ws.lut_curr
    );

    // Check convergence (skip first pass)
    if (pass_pair > 0) {
      float rms_diff, max_diff;
      compute_lut_diff(ws.lut_curr, ws.lut_prev, motor_config::CAL_LUT_SIZE,
                       rms_diff, max_diff);

      if (dbg) {
        dbg->print("CAL: LUT diff - RMS=");
        dbg->print(rms_diff, 2);
        dbg->print(" MAX=");
        dbg->println(max_diff, 2);
      }

      if (rms_diff <= config.stop_rms_counts && max_diff <= config.stop_max_counts) {
        consecutive_good++;
        if (consecutive_good >= config.stop_consecutive) {
          if (dbg) dbg->println("CAL: converged - stopping early");
          break;
        }
      } else {
        consecutive_good = 0;
      }
    }

    // Copy current LUT to previous for next comparison
    memcpy(ws.lut_prev, ws.lut_curr, sizeof(ws.lut_prev));
  }

  // Finalize
  motor.setPhaseVoltage(0, 0, 0);

  motor.zero_electric_angle = _normalizeAngle(final_avg_elec_angle / 2.0f);

  // Copy final LUT to output
  memcpy(out.lut_counts, ws.lut_curr, sizeof(out.lut_counts));
  out.zero_electric_angle = motor.zero_electric_angle;
  out.direction = static_cast<int32_t>(motor.sensor_direction);
  out.valid = true;

  // Print results
  if (dbg) {
    dbg->print("CAL: done, zero=");
    dbg->println(out.zero_electric_angle, 6);

    // Print LUT as int16 array
    dbg->print("int16_t calibrationLutCounts[");
    dbg->print(motor_config::CAL_LUT_SIZE);
    dbg->println("] = {");
    _delay(100);
    for (size_t i = 0; i < motor_config::CAL_LUT_SIZE; i++) {
      dbg->print(out.lut_counts[i]);
      if (i < motor_config::CAL_LUT_SIZE - 1) dbg->print(", ");
      if ((i + 1) % 16 == 0) dbg->println();
      _delay(1);
    }
    dbg->println("};");

    dbg->print("float zero_electric_angle = ");
    dbg->print(out.zero_electric_angle, 6);
    dbg->println(";");

    dbg->print("Direction sensor_direction = ");
    dbg->println(motor.sensor_direction == Direction::CCW ? "Direction::CCW;" : "Direction::CW;");
  }

  return true;
}

// Convenience overload that allocates workspace internally
bool calibrate_sensor(
    Sensor& wrapped,
    BLDCMotor& motor,
    SensorCalibrationData& out,
    const CalibrationConfig& config
) {
  // Allocate workspace on heap to avoid stack overflow
  CalibWorkspace* ws = new CalibWorkspace();
  if (!ws) {
    if (motor.monitor_port) {
      motor.monitor_port->println("CAL: failed to allocate workspace");
    }
    return false;
  }

  bool result = calibrate_sensor(wrapped, motor, out, *ws, config);

  delete ws;
  return result;
}
