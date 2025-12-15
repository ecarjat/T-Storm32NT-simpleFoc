#include "comms_streams.h"

#include <Arduino.h>
#include <comms/streams/BinaryIO.h>
#include <comms/streams/PacketCommander.h>
#include <comms/telemetry/Telemetry.h>

#include "board_pins.h"
#include "calibrated_sensor.h"
#include "runtime_settings.h"

// Forward pointers/state for settings and calibration handling.
static BLDCDriver3PWM *g_driver = nullptr;
static BLDCMotor *g_motor = nullptr;
static Sensor *g_raw_sensor = nullptr;
static StoredCalibratedSensor *g_calibrated_sensor = nullptr;

// Telemetry register set: target, angle, velocity, enable, status.
static uint8_t telemetry_registers[] = {
    REG_TARGET,
    REG_ANGLE,
    REG_POSITION,
    REG_VELOCITY,
    REG_STATUS,
};

static Telemetry telemetry;
static BinaryIO *stream_io = nullptr;

static bool perform_sensor_calibration() {
  if (!g_motor || !g_driver || !g_raw_sensor || !g_calibrated_sensor) {
    Serial.println("CAL_ERR");
    return false;
  }
  SensorCalibrationData data{};
  data.lut_size = motor_config::CAL_LUT_SIZE;

  // Ensure driver is enabled for calibration moves.
  g_driver->enable();
  bool ok = calibrate_sensor(*g_raw_sensor, *g_motor, data);
  if (!ok) {
    Serial.println("CAL_ERR");
    return false;
  }

  set_calibration_data(data);
  bool saved = save_settings_to_flash(*g_motor, *g_driver);

  g_calibrated_sensor->setCalibrationData(&runtime_settings().calibration);
  g_motor->sensor_direction = static_cast<Direction>(runtime_settings().calibration.direction);
  g_motor->zero_electric_angle = runtime_settings().calibration.zero_electric_angle;
  g_motor->linkSensor(g_calibrated_sensor);
  g_motor->initFOC();

  // Dump LUT for logging/verification before reporting status.
  Serial.print("CAL_LUT[");
  Serial.print(data.lut_size);
  Serial.print("]=");
  Serial.print("{");
  for (uint16_t i = 0; i < data.lut_size; i++) {
    if (i > 0) Serial.print(", ");
    Serial.print(data.lut[i], 6);
  }
  Serial.println("}");
  Serial.print("CAL_ZERO=");
  Serial.println(data.zero_electric_angle, 6);
  Serial.print("CAL_DIR=");
  Serial.println(data.direction);

  Serial.println(saved ? "CAL_OK" : "CAL_SAVE_ERR");
  return saved;
}

// Custom PacketCommander that catches a 'B' packet to trigger bootloader reset.
class BootPacketCommander : public PacketCommander {
public:
  using PacketCommander::PacketCommander;

protected:
  bool handlePacket(Packet &packet) override {
    if (packet.type == 'B') {
      uint8_t code = 0;
      *_io >> code;
      // require "B6" (at least two chars before newline) to avoid accidental resets
      if (code == 6) {
        request_bootloader_reset();
      }
      return true;
    }
    if (packet.type == 'S') {
      uint8_t code = 0;
      *_io >> code;
      if (code == 1 && motors[curMotor] && g_driver) {
        // PacketCommander stores FOCMotor*, but we add a BLDCMotor so this cast is safe here.
        auto *m = static_cast<BLDCMotor *>(motors[curMotor]);
        bool ok = save_settings_to_flash(*m, *g_driver);
        if (ok) {
          Serial.print("SAVE_OK\n");
        } else {
          Serial.print("SAVE_ERR\n");
        }
      }
      return true;
    }
    if (packet.type == 'C') {
      uint8_t code = 0;
      *_io >> code;
      // "C2" runs sensor calibration and stores LUT to flash.
      if (code == 2) {
        perform_sensor_calibration();
      }
      return true;
    }
    return PacketCommander::handlePacket(packet);
  }
};

// Binary stream over UART1 (PA9/PA10) for PacketCommander + Telemetry.
static BootPacketCommander packet_commander(/*echo=*/true);


// BKP_DR1 magic to request bootloader (handled in bootloader on reset)
constexpr uint16_t BOOT_MAGIC = 0xB007;

void init_streams(BLDCMotor &motor, BLDCDriver3PWM &driver, Sensor &raw_sensor, StoredCalibratedSensor &calibrated) {
  static BinaryIO serial_io(Serial);
  stream_io = &serial_io;
  g_driver = &driver;
  g_motor = &motor;
  g_raw_sensor = &raw_sensor;
  g_calibrated_sensor = &calibrated;

  packet_commander.addMotor(&motor);
  packet_commander.init(*stream_io);

  telemetry.addMotor(&motor);
  telemetry.setTelemetryRegisters(sizeof(telemetry_registers), telemetry_registers);
  telemetry.downsample = 100;      // send every 100 loop iterations by default
  telemetry.min_elapsed_time = 0;  // no additional rate limit
  telemetry.init(*stream_io);
}

void handle_streams(BLDCMotor &motor) {
  (void)motor;
  if (!stream_io) {
    return;
  }
  packet_commander.run();
  telemetry.run();

}

// Write magic to backup register and reset into bootloader
void request_bootloader_reset() {
  // Enable backup domain access and write magic to BKP_DR1
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_BKP_CLK_ENABLE();
  WRITE_REG(BKP->DR1, BOOT_MAGIC);

  NVIC_SystemReset();
}
