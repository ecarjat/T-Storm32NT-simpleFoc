#include "comms_streams.h"

#include <Arduino.h>
#include <comms/streams/PacketCommander.h>
#include <comms/streams/TextIO.h>
#include <comms/telemetry/Telemetry.h>

#include "board_pins.h"
#include "runtime_settings.h"

// Forward pointers/state for settings and calibration handling.
static BLDCDriver3PWM *g_driver = nullptr;
static BLDCMotor *g_motor = nullptr;
static bool calibration_mode = false;
static uint8_t calibration_stage = 0;
static float calibration_target_angle = 0.0f;
static float calibration_remaining = 0.0f;
static int calibration_dir = 1;
static unsigned long calibration_last_us = 0;

// Telemetry register set: target, angle, velocity, enable, status.
static uint8_t telemetry_registers[] = {
    REG_TARGET,
    REG_ANGLE,
    REG_POSITION,
    REG_VELOCITY,
    REG_STATUS,
};
static uint8_t telemetry_calib_registers[] = {
    REG_SENSOR_TIMESTAMP,
    REG_ANGLE,
    REG_POSITION,
    REG_SENSOR_MECHANICAL_ANGLE,
};

static Telemetry telemetry;
static TextIO *stream_io = nullptr;

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
    /*
    if (packet.type == 'C') {
      uint8_t code = 0;
      *_io >> code;
      // "C1" enters calibration mode
      if (code == 1 && g_motor) {
        calibration_mode = true;
        calibration_stage = 1;
        calibration_dir = 1;
        calibration_remaining = 3.1415926f; // 180 deg
        calibration_target_angle = g_motor->shaft_angle;
        calibration_last_us = micros();
        // Force open-loop angle mode with voltage torque and PWM modulation
        g_motor->controller = MotionControlType::angle_openloop;
        g_motor->torque_controller = TorqueControlType::voltage;
        g_motor->foc_modulation = FOCModulationType::SinePWM;
        g_motor->target = calibration_target_angle;
        telemetry.setTelemetryRegisters(sizeof(telemetry_calib_registers), telemetry_calib_registers);
        telemetry.downsample = 10;
        Serial.print("CAL_MODE\n");
      }
      return true;
    }
      */
    return PacketCommander::handlePacket(packet);
  }
};

// Text-based stream over UART1 (PA9/PA10). BinaryIO is available if higher throughput is needed.
static BootPacketCommander packet_commander(/*echo=*/true);


// BKP_DR1 magic to request bootloader (handled in bootloader on reset)
constexpr uint16_t BOOT_MAGIC = 0xB007;

void init_streams(BLDCMotor &motor, BLDCDriver3PWM &driver) {
  static TextIO serial_io(Serial);
  serial_io.precision = 4;
  stream_io = &serial_io;
  g_driver = &driver;
  g_motor = &motor;

  packet_commander.addMotor(&motor);
  packet_commander.init(*stream_io);

  telemetry.addMotor(&motor);
  telemetry.setTelemetryRegisters(sizeof(telemetry_registers), telemetry_registers);
  telemetry.downsample = 1000;      // send every 100 loop iterations by default
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

  /*
  if (calibration_mode && g_motor) {
    const float speed = 5.0f; // rad/s
    unsigned long now = micros();
    float dt = (now - calibration_last_us) / 1000000.0f;
    if (dt < 0) {
      dt = 0;
    }
    calibration_last_us = now;

    float delta = speed * dt * calibration_dir;
    calibration_target_angle += delta;
    calibration_remaining -= fabs(delta);
    g_motor->target = calibration_target_angle;

    if (calibration_remaining <= 0.0f) {
      if (calibration_stage == 1) {
        // Reverse direction for second 180 deg
        calibration_stage = 2;
        calibration_dir = -1;
        calibration_remaining = 3.1415926f;
      } else {
        // Done: restore defaults
        calibration_mode = false;
        calibration_stage = 0;
        calibration_dir = 1;
        calibration_remaining = 0.0f;
        g_motor->controller = MotionControlType::velocity;
        g_motor->target = 0.0f;
        telemetry.setTelemetryRegisters(sizeof(telemetry_registers), telemetry_registers);
        telemetry.downsample = 0;
      }
    }
  }
    */
}

// Write magic to backup register and reset into bootloader
void request_bootloader_reset() {
  // Enable backup domain access and write magic to BKP_DR1
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_BKP_CLK_ENABLE();
  WRITE_REG(BKP->DR1, BOOT_MAGIC);

  NVIC_SystemReset();
}
