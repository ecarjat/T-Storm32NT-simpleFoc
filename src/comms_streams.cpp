#include "comms_streams.h"

#include <Arduino.h>
#include <comms/streams/PacketCommander.h>
#include <comms/streams/TextIO.h>
#include <comms/telemetry/Telemetry.h>

#include "board_pins.h"

// Text-based stream over UART1 (PA9/PA10). BinaryIO is available if higher throughput is needed.
static PacketCommander packet_commander(/*echo=*/true);
static Telemetry telemetry;
static TextIO *stream_io = nullptr;

// Telemetry register set: target, angle, velocity, enable, status.
static uint8_t telemetry_registers[] = {
    REG_TARGET,
    REG_ANGLE,
    REG_VELOCITY,
    REG_ENABLE,
    REG_STATUS,
};

void init_streams(BLDCMotor &motor) {
  static TextIO serial_io(Serial);
  serial_io.precision = 4;
  stream_io = &serial_io;

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
