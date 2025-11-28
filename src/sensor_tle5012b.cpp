#include "sensor_tle5012b.h"

#include <Arduino.h>

// MagneticSensorTLE5012B drives SPI in 1-line (half-duplex) mode using MOSI.
// Hardware: PA7 (MOSI) → DATA via R5, PA6 (MISO) also tied to DATA for reads, PA5 = SCK, PA8 = CS.

MagneticSensorTLE5012B encoder_sensor(ENC_MOSI_PIN, ENC_SCK_PIN, ENC_CS_PIN);

void setup_tle5012b_sensor() {
  // Ensure the DATA line on PA6 is not driving the bus before the driver reconfigures pins.
  pinMode(ENC_MISO_PIN, INPUT);
  encoder_sensor.init();
}
