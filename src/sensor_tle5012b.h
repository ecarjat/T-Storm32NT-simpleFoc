#pragma once

#include <SimpleFOC.h>
#include <encoders/tle5012b/MagneticSensorTLE5012B.h>
#include "board_pins.h"

// MagneticSensorTLE5012B uses STM32 half-duplex SPI on the MOSI pin.
// Hardware bridges PA7 (MOSI) to the encoder DATA line via R5; PA6 reads DATA.
extern MagneticSensorTLE5012B encoder_sensor;

// Convenience setup hook to configure SPI and the encoder driver.
void setup_tle5012b_sensor();
