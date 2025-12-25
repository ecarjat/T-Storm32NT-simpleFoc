#pragma once

#include <SimpleFOC.h>
#include "calibrated_sensor.h"

// UART streams/telemetry bridge using Arduino-FOC-drivers comms API.
// PacketCommander handles register R/W, Telemetry periodically publishes key registers.
// Transport is Arduino Stream-compatible (UART1 DMA stream on PA9/PA10).
void init_streams(BLDCMotor &motor, BLDCDriver3PWM &driver, Sensor &raw_sensor, StoredCalibratedSensor &calibrated);
void handle_streams();

// Trigger a reset into the bootloader update mode.
void request_bootloader_reset();
