#include "comms_streams.h"

#include <Arduino.h>
#include <cstdio>
#include <comms/streams/PacketCommander.h>
#include <comms/telemetry/Telemetry.h>

#include "board_pins.h"
#include "calibrated_sensor.h"
#include "flushing_binary_io.h"
#include "log_packet.h"
#include "runtime_settings.h"
#include "status_led.h"
#include "uart_dma_stream.h"

extern volatile bool control_isr_active;
extern volatile bool control_loop_enabled;

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
    log_packet(LOG_ERROR, "CAL", "ERR");
    return false;
  }
  control_loop_enabled = false;
  uint32_t wait_us = 0;
  while (control_isr_active) {
    delayMicroseconds(100);
    wait_us += 100;
    if (wait_us >= 1000000) {
      log_packet(LOG_ERROR, "CAL", "ISR_BUSY");
      control_loop_enabled = true;
      return false;
    }
  }
  SensorCalibrationData data{};
  data.lut_size = motor_config::CAL_LUT_SIZE;

  //store previous motion control mode
  MotionControlType prev_control = g_motor->controller;
  // Ensure driver is enabled for calibration moves and set to torque mode
  g_motor->controller = MotionControlType::torque;
  g_driver->enable();
  bool ok = calibrate_sensor(*g_raw_sensor, *g_motor, data);
  if (!ok) {
    log_packet(LOG_ERROR, "CAL", "ERR");
    control_loop_enabled = true;
    return false;
  }
  //restore previous motion control mode
  g_motor->controller = prev_control;

  set_calibration_data(data);
  bool saved = save_settings_to_flash(*g_motor, *g_driver);

  g_calibrated_sensor->setCalibrationData(&runtime_settings().calibration);
  g_motor->sensor_direction = static_cast<Direction>(runtime_settings().calibration.direction);
  g_motor->zero_electric_angle = runtime_settings().calibration.zero_electric_angle;
  g_motor->linkSensor(g_calibrated_sensor);
  g_motor->initFOC();

  // Dump LUT entries via log packets for host capture.
  // LUT is now stored as int16_t counts, not float radians.
  char msg[60] = {0};
  for (uint16_t i = 0; i < data.lut_size; i++) {
    snprintf(msg, sizeof(msg), "LUT[%u]=%d", static_cast<unsigned>(i), static_cast<int>(data.lut_counts[i]));
    log_packet(LOG_INFO, "CAL", msg);
  }
  snprintf(msg, sizeof(msg), "ZERO=%.6f", data.zero_electric_angle);
  log_packet(LOG_INFO, "CAL", msg);
  snprintf(msg, sizeof(msg), "DIR=%u", static_cast<unsigned>(data.direction));
  log_packet(LOG_INFO, "CAL", msg);
  log_packet(saved ? LOG_INFO : LOG_ERROR, "CAL", saved ? "OK" : "SAVE_ERR");
  control_loop_enabled = true;
  return saved;
}

// Custom PacketCommander that catches a 'B' packet to trigger bootloader reset.
class BootPacketCommander : public PacketCommander {
public:
  using PacketCommander::PacketCommander;

  void run() override {
    *_io >> curr_packet;
    if (curr_packet.type != 0x00) {
      bool handled = false;
      commanderror = false;
      handled = handlePacket(curr_packet);
      lastcommanderror = commanderror;
      lastcommandregister = curRegister;
      bool bad_packet = false;
      if (!handled) {
        bad_packet = true;
      }
      if (commanderror) {
        _io->in_sync = false;
        bad_packet = true;
      }
      if (!_io->is_complete()) {
        _io->in_sync = false;
        bad_packet = true;
      }
      if (bad_packet) {
        status_led_pulse();
      }
    }
  }

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
    if (packet.type == 'W') {
      uint8_t code = 0;
      *_io >> code;
      if (code == 1 && motors[curMotor] && g_driver) {
        // PacketCommander stores FOCMotor*, but we add a BLDCMotor so this cast is safe here.
        auto *m = static_cast<BLDCMotor *>(motors[curMotor]);
        bool ok = save_settings_to_flash(*m, *g_driver);
        // Respond with w1=1 (ok) or w1=0 (fail) over BinaryIO.
        *_io << START_PACKET('w', 2) << (uint8_t)1 << Separator('=') << (uint8_t)(ok ? 1 : 0) << END_PACKET;
      }
      return true;
    }
    if (packet.type == 'C') {
      uint8_t code = 0;
      *_io >> code;
      // "C2" runs sensor calibration and stores LUT to flash.
      if (code == 2) {
        bool ok = perform_sensor_calibration();
        // Respond with c2=1 (ok) or c2=0 (fail) similar to register responses.
        *_io << START_PACKET('c', 2) << (uint8_t)2 << Separator('=') << (uint8_t)(ok ? 1 : 0) << END_PACKET;
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
  static FlushingBinaryIO dma_io(uart_dma_stream());
  stream_io = &dma_io;
  g_driver = &driver;
  g_motor = &motor;
  g_raw_sensor = &raw_sensor;
  g_calibrated_sensor = &calibrated;

  init_log_stream(*stream_io);
  packet_commander.addMotor(&motor);
  packet_commander.init(*stream_io);

  telemetry.addMotor(&motor);
  telemetry.setTelemetryRegisters(sizeof(telemetry_registers), telemetry_registers);
  telemetry.downsample = 1;      // allow every loop; rate-limited by min_elapsed_time
  telemetry.min_elapsed_time = 2000;  // 500 Hz default
  telemetry.init(*stream_io);
}

void handle_streams() {
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
