#include "runtime_settings.h"

#include <Arduino.h>
#include <stm32f1xx_hal.h>

// CRC32 (polynomial 0xEDB88320) for integrity.
static uint32_t crc32_calc(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      uint32_t mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
  }
  return ~crc;
}

RuntimeSettings& runtime_settings() {
  static RuntimeSettings settings;
  return settings;
}

struct PersistedSettings {
  uint32_t magic;
  uint32_t version;
  RuntimeSettings data;
  uint32_t crc;
};

// Compile-time size check so we know the flash pages still fit the struct.
static constexpr size_t PERSISTED_SETTINGS_SIZE = sizeof(PersistedSettings);
static_assert(PERSISTED_SETTINGS_SIZE <= 3072, "Persisted settings exceed reserved 3KB flash pages");

static PersistedSettings* flash_ptr() {
  return reinterpret_cast<PersistedSettings*>(SETTINGS_ADDR);
}

bool load_settings_from_flash() {
  PersistedSettings* p = flash_ptr();
  if (p->magic != SETTINGS_MAGIC || p->version != SETTINGS_VERSION) {
    return false;
  }
  uint32_t calc = crc32_calc(reinterpret_cast<const uint8_t*>(&p->data), sizeof(RuntimeSettings));
  if (calc != p->crc) {
    return false;
  }
  runtime_settings() = p->data;
  return true;
}

static bool flash_erase_pages(uint32_t address, uint32_t num_pages) {
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef erase = {};
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = address;
  erase.NbPages = num_pages;
  uint32_t page_error = 0;
  HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &page_error);
  HAL_FLASH_Lock();
  return st == HAL_OK;
}

static bool flash_write_bytes(uint32_t address, const uint8_t* data, size_t len) {
  HAL_FLASH_Unlock();
  for (size_t i = 0; i < len; i += 2) {
    uint16_t half = data[i];
    if (i + 1 < len) {
      half |= static_cast<uint16_t>(data[i + 1]) << 8;
    } else {
      half |= 0xFF00;  // pad
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + i, half) != HAL_OK) {
      HAL_FLASH_Lock();
      return false;
    }
  }
  HAL_FLASH_Lock();
  return true;
}

bool save_settings_to_flash(const BLDCMotor& motor, const BLDCDriver3PWM& driver) {
  // Populate runtime_settings() from current motor/driver state.
  RuntimeSettings& s = runtime_settings();
  // Core motor/driver parameters
  s.motor_voltage_limit = motor.voltage_limit;
  s.motor_current_limit = motor.current_limit;
  s.velocity_limit = motor.velocity_limit;
  s.driver_voltage_limit = driver.voltage_limit;
  s.pole_pairs = motor.pole_pairs;
  s.phase_resistance = motor.phase_resistance;
  s.kv_rating = motor.KV_rating;
  s.supply_voltage = driver.voltage_power_supply;
  s.motion_downsample = motor.motion_downsample;

  // Velocity PID settings
  s.v_pid_p = motor.PID_velocity.P;
  s.v_pid_i = motor.PID_velocity.I;
  s.v_pid_d = motor.PID_velocity.D;
  s.v_pid_velocity_limit = motor.PID_velocity.limit;
  s.v_lpf_tf = motor.LPF_velocity.Tf;
  s.v_output_ramp = motor.PID_velocity.output_ramp;

  // Angle PID settings
  s.a_pid_p = motor.P_angle.P;
  s.a_pid_i = motor.P_angle.I;
  s.a_pid_d = motor.P_angle.D;
  s.a_pid_output_limit = motor.P_angle.limit;
  s.a_lpf_tf = motor.LPF_angle.Tf;
  s.a_output_ramp = motor.P_angle.output_ramp;

  // also persist the current zero angle/direction in case calibration ran earlier
  s.calibration.zero_electric_angle = motor.zero_electric_angle;
  s.calibration.direction = static_cast<int32_t>(motor.sensor_direction);

  PersistedSettings blob{};
  blob.magic = SETTINGS_MAGIC;
  blob.version = SETTINGS_VERSION;
  blob.data = s;
  blob.crc = crc32_calc(reinterpret_cast<const uint8_t*>(&blob.data), sizeof(RuntimeSettings));

  if (!flash_erase_pages(SETTINGS_ADDR, 3)) {
    return false;
  }
  return flash_write_bytes(SETTINGS_ADDR, reinterpret_cast<const uint8_t*>(&blob), sizeof(blob));
}

void apply_settings_to_objects(BLDCMotor& motor, BLDCDriver3PWM& driver) {
  RuntimeSettings& s = runtime_settings();
  // Core motor/driver parameters
  motor.voltage_limit = s.motor_voltage_limit;
  motor.current_limit = s.motor_current_limit;
  motor.velocity_limit = s.velocity_limit;
  driver.voltage_limit = s.driver_voltage_limit;
  motor.pole_pairs = s.pole_pairs;
  motor.phase_resistance = s.phase_resistance;
  motor.KV_rating = s.kv_rating;
  driver.voltage_power_supply = s.supply_voltage;
  motor.motion_downsample = s.motion_downsample;

  // Velocity PID settings
  motor.PID_velocity.P = s.v_pid_p;
  motor.PID_velocity.I = s.v_pid_i;
  motor.PID_velocity.D = s.v_pid_d;
  motor.PID_velocity.limit = s.v_pid_velocity_limit;
  motor.LPF_velocity.Tf = s.v_lpf_tf;
  motor.PID_velocity.output_ramp = s.v_output_ramp;

  // Angle PID settings
  motor.P_angle.P = s.a_pid_p;
  motor.P_angle.I = s.a_pid_i;
  motor.P_angle.D = s.a_pid_d;
  motor.P_angle.limit = s.a_pid_output_limit;
  motor.LPF_angle.Tf = s.a_lpf_tf;
  motor.P_angle.output_ramp = s.a_output_ramp;

  if (s.calibration.valid && s.calibration.lut_size == motor_config::CAL_LUT_SIZE) {
    motor.zero_electric_angle = s.calibration.zero_electric_angle;
    motor.sensor_direction = static_cast<Direction>(s.calibration.direction);
  }
}

void set_calibration_data(const SensorCalibrationData& data) {
  runtime_settings().calibration = data;
}

void clear_calibration_data() {
  SensorCalibrationData empty{};
  empty.valid = false;
  runtime_settings().calibration = empty;
}
