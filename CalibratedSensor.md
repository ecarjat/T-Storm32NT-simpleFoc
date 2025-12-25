# SPEC: CalibratedSensor overhaul for robust LUT calibration + counts-based LUT (SimpleFOC / STM32F103 / TLE5012B)

## 0) Goals
- Improve velocity-mode stability by eliminating LUT noise, SPI glitches, and quantization artifacts.
- Keep commutation/mechanical angle precision as high as possible.
- Store LUT as OFFSETS in COUNTS (not radians), with power-of-two LUT size.
- Make calibration robust via multiple passes, multiple samples per step, robust aggregation (median/trimmed mean), and convergence-based early stop.
- Remove variable-length arrays (VLAs) and large stack allocations to avoid stack corruption on STM32F103.

---

## 1) Runtime sensor angle handling
### 1.1 Preserve high-resolution mechanical angle for commutation
- Do NOT add heavy low-pass filtering to the angle used by FOC commutation.
- Keep wrapped sensor’s mechanical angle path unchanged (or only minimal non-phase-lag operations).

### 1.2 Provide calibrated sensor angle via LUT correction in COUNT space
- The calibrated sensor angle returned to SimpleFOC is computed as:
  corrected_counts = wrapCounts(raw_counts - interpolated_offset_counts)
  corrected_angle_rad = corrected_counts * (2π / CPR)

- CPR = 32768 for TLE5012B in this project.

### 1.3 Optional precision reduction ONLY for getSensorAngle output (NOT for commutation)
- Implement optional LSB dropping on the corrected output counts (post-LUT):
  drop 1 bit: corrected_counts &= 0xFFFE
  drop 2 bits: corrected_counts &= 0xFFFC
- Provide configuration flag(s) or parameter to enable/disable and choose 0/1/2 bits.
- Default: drop_bits = 0 (disabled). Only enable if needed for velocity jitter.

---

## 2) LUT size and structure
### 2.1 LUT size
- Set LUT size to a power of two.
- Target: LUT_N = 1024 (default).
- Optionally support LUT_N = 2048 as an advanced mode (overkill).

### 2.2 LUT contents
- Store LUT offsets as COUNTS (signed), centered around 0:
  int16_t calibrationLutCounts[LUT_N]
- Offset meaning: how many counts to subtract from raw to obtain corrected.
- Offsets should be in signed count space, typically within a small range (int16 is ample).

### 2.3 Bin width and interpolation
- BIN_COUNTS = CPR / LUT_N
  For LUT_N=1024: BIN_COUNTS=32
- LUT index:
  idx = raw_counts / BIN_COUNTS
  frac = raw_counts - idx*BIN_COUNTS  (0..BIN_COUNTS-1)
  next = (idx+1) & (LUT_N-1)
- Linear interpolation (counts):
  offset = o0 + ((o1 - o0) * frac) / BIN_COUNTS
- corrected = wrapCounts(raw_counts - offset)

### 2.4 Wrap helpers
- Implement wrapCounts(x) to return x in [0, CPR-1].
- Implement signed-wrap helper for error in [-CPR/2, +CPR/2] where needed.

---

## 3) Calibration process improvements
### 3.1 Number of passes
- Perform multiple calibration passes:
  default: 3 forward + 3 backward (6 total passes).
  advanced: 5 forward + 5 backward (10 total passes).
- Each pass traverses the same electrical angle grid and records error at each tick i.

### 3.2 Multiple samples per tick (after settle)
- At each tick i, after the settle delay, sample the sensor multiple times and aggregate:
  N_SAMPLES_PER_TICK default: 10
  sampling interval: ~1ms (or as feasible) between samples
  sampling window: ~10ms total
- For each tick and each sample:
  - call _wrapped.update()
  - read wrapped angle(s) needed for error computation
  - optionally discard sample if SPI safety/CRC indicates invalid data (see Section 5)
- Aggregate samples per tick using a robust method (Section 3.4).

### 3.3 Settle time and calibration voltage
- Keep settle_time_ms default = 30ms (current).
- Provide configuration to increase settle_time_ms to 50ms if rotor continues creeping.
- Keep voltage_calibration default = 3V (current).
- Provide option to raise voltage_calibration to 4V if forward/backward disagreement indicates slipping or insufficient hold torque.
- Maintain n2_ticks smoothing microsteps; allow increasing n2_ticks from 5 to 8–10 if step-to-step snapping observed.

### 3.4 Error calculation and robust aggregation
- Continue using the existing conceptual error definition:
  err_rad = 0.5*(theta_actual - elec_angle / NPP) for forward pass, plus 0.5*(...) for backward pass (averaging direction effects).
- Convert error to COUNTS for storage and aggregation:
  err_counts = radToSignedCounts(err_rad)
- For each tick i:
  - collect err_counts samples for the tick (N_SAMPLES_PER_TICK samples)
  - compute per-tick robust statistic:
    - Primary: median of samples (robust to spikes)
    - Optional: trimmed mean (drop top/bottom 10–20%) then average
- Across passes:
  - For each tick i, accumulate per-pass per-tick value (median or trimmed mean of that pass/tick)
  - Final tick error is the average (mean) of per-pass tick values:
    tick_error[i] = mean_over_passes(pass_tick_value[i])
  - Optional: also compute variance to detect unstable ticks.

### 3.5 “Stop when good” convergence criterion
- Implement early stop to avoid unnecessary extra passes.
- After completing each full forward+backward pair (2 passes) or after each full set (e.g., 1F+1B), update an intermediate LUT and compare to previous LUT:
  - Compute RMS difference in counts:
    rms = sqrt(mean_i( (lut_new[i] - lut_prev[i])^2 ))
  - Also track max_abs difference.
- Stop conditions (defaults):
  - rms <= 1 count AND max_abs <= 3 counts for two consecutive comparisons
  - OR maximum configured passes reached
- Provide parameters:
  - STOP_RMS_COUNTS default 1
  - STOP_MAX_COUNTS default 3
  - STOP_CONSECUTIVE default 2
  - MAX_PASS_PAIRS default 3 (for 3F+3B), configurable up to 5 pairs

---

## 4) Filtering and LUT construction
### 4.1 Preserve existing cogging/electrical-frequency rejection filter logic
- Current approach uses a moving average filter window = n_pos to remove components at electrical frequency and multiples.
- Keep the filter concept but apply it consistently to the aggregated tick error array.

### 4.2 Filtering implementation
- Input: tick_error[i] array (in counts or radians; choose one and be consistent).
- Recommended: run filter in FLOAT radians for minimal behavior change, then convert to counts for LUT storage.
  - Alternatively run filter directly in counts (int32 intermediate) for speed; ensure no overflow.

### 4.3 Mean removal
- Continue computing error_mean and subtract it from filtered error before LUT storage.
- Ensure mean removal is computed on filtered array.
- Apply sensor_direction sign consistently.

### 4.4 Offset/index alignment (raw_offset) handling
- Preserve raw_offset logic but ensure it is applied correctly in count domain:
  - raw_offset_rad = (theta_absolute_init + theta_absolute_post)/2
  - index_offset = floor(LUT_N * raw_offset_rad / 2π)
- Ensure index_offset is wrapped to [0, LUT_N-1].
- Ensure correct direction mapping:
  ind = index_offset + i * sensor_direction
  wrap ind to LUT_N range
- Map from tick grid (n_ticks) to LUT grid (LUT_N):
  dn = n_ticks / float(LUT_N)
  source_index = int(i * dn)
- Store:
  lut_counts[ind] = sensor_direction * radToSignedCounts(filtered_error[source_index] - error_mean)

---

## 5) SPI safety / CRC robustness (must-have)
### 5.1 Do not inject invalid samples into control or calibration
- If safety bits indicate error or CRC mismatch:
  - do NOT return the new value
  - return last_good value (count or angle), and mark sample invalid for calibration sampling
- Avoid vendor behavior of returning 0 on error (creates discontinuity).

### 5.2 Calibration sampling behavior under errors
- When collecting N_SAMPLES_PER_TICK:
  - if a sample is invalid, discard it and continue sampling until either:
    - valid_samples == N_SAMPLES_PER_TICK
    - OR timeout / max_attempts reached (e.g., N_SAMPLES_PER_TICK * 3 attempts)
  - If insufficient valid samples:
    - fall back to using whatever valid samples exist if above a minimum threshold (e.g., >= 50%)
    - otherwise mark tick as suspect and optionally reuse last pass value.

---

## 6) Remove VLAs and large stack allocations (STM32F103 safety)
### 6.1 VLAs to eliminate
- error[n_ticks] and window_buffer[window] are VLAs and risky on embedded stacks.

### 6.2 Required refactor
- Replace VLAs with one of:
  - statically allocated buffers with maximum sizes (compile-time constants), OR
  - heap allocations (new/delete) with explicit lifetime management, OR
  - a small custom allocator / pre-allocated workspace buffer

### 6.3 Preferred approach
- Introduce a CalibWorkspace struct owned by CalibratedSensor:
  - tick_error_rad or tick_error_counts arrays sized to max n_ticks
  - filtered_error array
  - per-tick sample scratch buffer sized to N_SAMPLES_PER_TICK (or dynamic)
  - window buffer sized to max window (n_pos)
- Allocate workspace once and reuse across calibrations; avoid repeated heap churn.

---

## 7) API and code structure changes (agent tasks)
### 7.1 Class members
- Replace float* calibrationLut with:
  - int16_t* calibrationLutCounts (or fixed array if compile-time)
- Add configuration fields:
  - int lut_size (LUT_N)
  - int cpr (CPR)
  - int drop_bits (0/1/2)
  - int n_pass_pairs_max (default 3)
  - int n_samples_per_tick (default 10)
  - int sample_delay_ms (default 1)
  - float stop_rms_counts, stop_max_counts
  - int stop_consecutive

### 7.2 getSensorAngle implementation
- Implement count-based LUT lookup + interpolation.
- Convert to radians at the end.
- Apply optional drop_bits to corrected counts post-LUT.

### 7.3 Calibration implementation
- Refactor calibrate() to:
  - perform repeated forward/backward passes
  - per tick: after settle, gather N samples and compute per-tick robust statistic (median or trimmed mean)
  - aggregate per-pass tick values into tick_error array
  - after each pass pair, compute intermediate LUT and compare to previous LUT (stop when good)
  - after completion, finalize LUT, print it, and print motor.zero_electric_angle + sensor_direction as before

### 7.4 Robust statistics requirements
- Provide a median function for small arrays of int32/int16 samples:
  - sorting N<=25 via insertion sort is acceptable
  - return middle value (or average of two middle for even N)
- Provide trimmed mean function:
  - sort samples
  - discard k lowest and k highest (k configurable, default 10% of N, at least 1 if N>=10)
  - average remaining
- Choose default per-tick estimator: median.

### 7.5 Convergence tracking
- Maintain lut_prev_counts[LUT_N] and lut_curr_counts[LUT_N] buffers.
- After each pair of passes:
  - rebuild lut_curr_counts
  - compute RMS and max_abs diffs
  - if below thresholds for STOP_CONSECUTIVE, stop early
  - else swap/copy curr->prev and continue

---

## 8) Defaults (recommended)
- CPR = 32768
- LUT_N = 1024
- n_pos = 5 (existing)
- n2_ticks = 5 (existing), allow 8–10 if needed
- settle_time_ms = 30
- voltage_calibration = 3
- N_SAMPLES_PER_TICK = 10
- sample_delay_ms = 1
- per-tick estimator = median
- MAX_PASS_PAIRS = 3 (3F+3B)
- STOP_RMS_COUNTS = 1
- STOP_MAX_COUNTS = 3
- STOP_CONSECUTIVE = 2
- drop_bits = 0 (disabled)

---

## 9) Acceptance criteria / validation
- Calibration completes without stack overflow (no VLAs).
- LUT prints as counts (int16) array; offsets are continuous and not stair-steppy.
- Re-running calibration yields similar LUT (RMS difference low).
- Velocity at rest shows significantly reduced spikes compared to baseline.
- In velocity mode, target 2 rad/s shows reduced dips/limit-cycle behavior.
- Safety/CRC errors do not inject discontinuities (control remains stable on occasional SPI errors).

---