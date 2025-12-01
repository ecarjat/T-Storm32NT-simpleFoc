#include "comms_streams.h"

#include <Arduino.h>
#include <comms/streams/PacketCommander.h>
#include <comms/streams/TextIO.h>
#include <comms/telemetry/Telemetry.h>

#include "board_pins.h"
#include "runtime_settings.h"

// Forward pointer to driver for save-settings handling.
static BLDCDriver3PWM *g_driver = nullptr;

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
    return PacketCommander::handlePacket(packet);
  }
};

// Text-based stream over UART1 (PA9/PA10). BinaryIO is available if higher throughput is needed.
static BootPacketCommander packet_commander(/*echo=*/true);
static Telemetry telemetry;
static TextIO *stream_io = nullptr;

// BKP_DR1 magic to request bootloader (handled in bootloader on reset)
constexpr uint16_t BOOT_MAGIC = 0xB007;

// Telemetry register set: target, angle, velocity, enable, status.
static uint8_t telemetry_registers[] = {
    REG_TARGET,
    REG_ANGLE,
    REG_VELOCITY,
    REG_ENABLE,
    REG_STATUS,
};

void init_streams(BLDCMotor &motor, BLDCDriver3PWM &driver) {
  static TextIO serial_io(Serial);
  serial_io.precision = 4;
  stream_io = &serial_io;
  g_driver = &driver;

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
