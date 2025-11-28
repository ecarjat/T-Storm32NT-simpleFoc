#pragma once

#include <SimpleFOC.h>

// UART streams/telemetry bridge using Arduino-FOC-drivers comms API.
// PacketCommander handles register R/W, Telemetry periodically publishes key registers.
// Transport is Arduino Stream-compatible (UART1 / Serial on PA9/PA10).
void init_streams(BLDCMotor &motor);
void handle_streams(BLDCMotor &motor);

// Trigger a reset into the bootloader update mode.
void request_bootloader_reset();
