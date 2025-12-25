#pragma once

#include <SimpleFOC.h>
#include <cstdint>
#include "motor_config.h"
#include "runtime_settings.h"

// ============================================================================
// Count-space utilities
// ============================================================================

// Wrap counts to [0, CPR-1]
inline int32_t wrapCounts(int32_t x) {
  constexpr int32_t cpr = motor_config::CPR;
  x = x % cpr;
  return (x < 0) ? x + cpr : x;
}

// Wrap counts to signed range [-CPR/2, +CPR/2)
inline int32_t wrapCountsSigned(int32_t x) {
  constexpr int32_t cpr = motor_config::CPR;
  constexpr int32_t half = cpr / 2;
  x = wrapCounts(x);
  return (x >= half) ? x - cpr : x;
}

// Convert radians to counts
inline int32_t radToCounts(float rad) {
  constexpr float scale = motor_config::CPR / _2PI;
  return static_cast<int32_t>(rad * scale);
}

// Convert counts to radians
inline float countsToRad(int32_t counts) {
  constexpr float scale = _2PI / motor_config::CPR;
  return counts * scale;
}

// ============================================================================
// Calibration workspace (heap-allocated, eliminates VLAs)
// ============================================================================

// Maximum tick count: n_pos * max_pole_pairs
// For 14 pole pairs and n_pos=5: 70 ticks max. We'll use 128 for safety.
constexpr int CAL_MAX_TICKS = 128;

struct CalibWorkspace {
  // Per-tick error accumulation (radians, across all passes)
  float tick_error_rad[CAL_MAX_TICKS];

  // Filtered error (after moving average)
  float filtered_error_rad[CAL_MAX_TICKS];

  // Per-tick sample buffer for robust aggregation
  int32_t sample_counts[motor_config::CAL_N_SAMPLES_PER_TICK];

  // Moving average window buffer
  float window_buffer[motor_config::CAL_N_POS];

  // LUT buffers for convergence tracking
  int16_t lut_prev[motor_config::CAL_LUT_SIZE];
  int16_t lut_curr[motor_config::CAL_LUT_SIZE];

  // Pass accumulation: tick_error per pass for averaging
  float pass_tick_error[CAL_MAX_TICKS];
};

// ============================================================================
// StoredCalibratedSensor: Runtime sensor with persisted calibration
// ============================================================================

class StoredCalibratedSensor : public Sensor {
public:
  explicit StoredCalibratedSensor(Sensor& wrapped);

  void setCalibrationData(SensorCalibrationData* data);

  // Optional: set number of LSBs to drop from corrected counts (0/1/2)
  void setDropBits(int bits) { drop_bits_ = bits; }

  void update() override;
  void init() override;
  float getSensorAngle() override;

  // Get the last valid raw counts (for SPI error fallback)
  int32_t getLastValidCounts() const { return last_valid_counts_; }

private:
  Sensor& wrapped_;
  SensorCalibrationData* calibration_;
  int drop_bits_ = motor_config::CAL_DROP_BITS;
  int32_t last_valid_counts_ = 0;

  // Apply LUT correction in count space with linear interpolation
  int32_t applyCorrectionCounts(int32_t raw_counts) const;
};

// ============================================================================
// Calibration function
// ============================================================================

// Calibration configuration (can be customized per-call)
struct CalibrationConfig {
  int settle_time_ms = motor_config::CAL_SETTLE_TIME_MS;
  float voltage = motor_config::CAL_VOLTAGE;
  int n_samples_per_tick = motor_config::CAL_N_SAMPLES_PER_TICK;
  int sample_delay_ms = motor_config::CAL_SAMPLE_DELAY_MS;
  int max_pass_pairs = motor_config::CAL_MAX_PASS_PAIRS;
  float stop_rms_counts = motor_config::CAL_STOP_RMS_COUNTS;
  float stop_max_counts = motor_config::CAL_STOP_MAX_COUNTS;
  int stop_consecutive = motor_config::CAL_STOP_CONSECUTIVE;
  int n_pos = motor_config::CAL_N_POS;
  int n2_ticks = motor_config::CAL_N2_TICKS;
};

// Run calibration against the wrapped sensor and fill SensorCalibrationData.
// Returns true on success, false on failure (e.g., FOC alignment failed).
// The workspace must be provided (heap-allocated by caller to avoid stack overflow).
bool calibrate_sensor(
    Sensor& wrapped,
    BLDCMotor& motor,
    SensorCalibrationData& out,
    CalibWorkspace& workspace,
    const CalibrationConfig& config = CalibrationConfig{}
);

// Convenience overload that allocates workspace internally
bool calibrate_sensor(
    Sensor& wrapped,
    BLDCMotor& motor,
    SensorCalibrationData& out,
    const CalibrationConfig& config = CalibrationConfig{}
);

// ============================================================================
// Robust statistics helpers
// ============================================================================

// Compute median of int32_t array (sorts in place)
int32_t median_int32(int32_t* arr, int n);

// Compute trimmed mean of int32_t array (sorts in place)
// Discards k lowest and k highest values, averages the rest
int32_t trimmed_mean_int32(int32_t* arr, int n, int k = 1);
