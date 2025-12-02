#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>

#if defined(_STM32_DEF_)

class TLE5012BFullDuplex : public Sensor {
public:
  TLE5012BFullDuplex(int mosi, int miso, int sck, int ncs, uint32_t freq_hz = 1000000);
  ~TLE5012BFullDuplex() = default;

  void init() override;
  uint16_t readRawAngle();
  float getCurrentAngle(); // radians

protected:
  float getSensorAngle() override; // wrapped 0..2PI

private:
  void readBytes(uint16_t reg, uint8_t *data, uint8_t len);

  int _mosi;
  int _miso;
  int _sck;
  int _ncs;
  uint32_t _freq;
  SPI_HandleTypeDef _spi;
};

#endif
