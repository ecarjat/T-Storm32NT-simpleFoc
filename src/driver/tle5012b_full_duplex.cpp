#include "tle5012b_full_duplex.h"

#if defined(_STM32_DEF_)

#include <cmath>
#include "utility/spi_com.h"
#include "log_packet.h"
#include "status_led.h"
extern "C" uint32_t spi_getClkFreqInst(SPI_TypeDef *spi_inst);

#define TLE5012B_CPR 32768.0f
#define _2PI 6.28318530718f
#define TLE5012B_READ_REGISTER 0x8000
#define TLE5012B_ANGLE_REG 0x0020

TLE5012BFullDuplex::TLE5012BFullDuplex(int mosi, int miso, int sck, int ncs, uint32_t freq_hz)
    : _mosi(mosi), _miso(miso), _sck(sck), _ncs(ncs), _freq(freq_hz) {}

void TLE5012BFullDuplex::init() {
  pinMode(_ncs, OUTPUT);
  digitalWrite(_ncs, HIGH);

  GPIO_InitTypeDef gpio;
  gpio.Pin = digitalPinToBitMask(_mosi);
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(digitalPinToPort(_mosi), &gpio);

  gpio.Pin = digitalPinToBitMask(_miso);
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(digitalPinToPort(_miso), &gpio);

  gpio.Pin = digitalPinToBitMask(_sck);
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(digitalPinToPort(_sck), &gpio);

  SPI_TypeDef *spi_mosi = (SPI_TypeDef *)pinmap_peripheral(digitalPinToPinName(_mosi), PinMap_SPI_MOSI);
  SPI_TypeDef *spi_miso = (SPI_TypeDef *)pinmap_peripheral(digitalPinToPinName(_miso), PinMap_SPI_MISO);
  SPI_TypeDef *spi_sclk = (SPI_TypeDef *)pinmap_peripheral(digitalPinToPinName(_sck), PinMap_SPI_SCLK);
  SPI_TypeDef *spi_inst = (SPI_TypeDef *)pinmap_merge_peripheral(spi_mosi, spi_sclk);
  spi_inst = (SPI_TypeDef *)pinmap_merge_peripheral(spi_inst, spi_miso);

  pinmap_pinout(digitalPinToPinName(_mosi), PinMap_SPI_MOSI);
  pinmap_pinout(digitalPinToPinName(_miso), PinMap_SPI_MISO);
  pinmap_pinout(digitalPinToPinName(_sck), PinMap_SPI_SCLK);

  if (spi_inst == SPI1) {
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
  }
#if defined(SPI2_BASE)
  else if (spi_inst == SPI2) {
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();
  }
#endif
#if defined(SPI3_BASE)
  else if (spi_inst == SPI3) {
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_SPI3_FORCE_RESET();
    __HAL_RCC_SPI3_RELEASE_RESET();
  }
#endif

  _spi.Instance = spi_inst;
  _spi.Init.Direction = SPI_DIRECTION_2LINES;
  _spi.Init.Mode = SPI_MODE_MASTER;
  _spi.Init.DataSize = SPI_DATASIZE_8BIT;
  _spi.Init.CLKPolarity = SPI_POLARITY_LOW;
  _spi.Init.CLKPhase = SPI_PHASE_2EDGE;
  _spi.Init.NSS = SPI_NSS_SOFT;
  _spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
  _spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  _spi.Init.CRCPolynomial = 7;
  _spi.Init.TIMode = SPI_TIMODE_DISABLE;
#if defined(SPI_NSS_PULSE_DISABLE)
  _spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
#endif

  uint32_t spi_freq = spi_getClkFreqInst(spi_inst);
  if (_freq >= (spi_freq / SPI_SPEED_CLOCK_DIV2_MHZ)) {
    _spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  } else if (_freq >= (spi_freq / SPI_SPEED_CLOCK_DIV4_MHZ)) {
    _spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  } else if (_freq >= (spi_freq / SPI_SPEED_CLOCK_DIV8_MHZ)) {
    _spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  } else if (_freq >= (spi_freq / SPI_SPEED_CLOCK_DIV16_MHZ)) {
    _spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  } else if (_freq >= (spi_freq / SPI_SPEED_CLOCK_DIV32_MHZ)) {
    _spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  } else if (_freq >= (spi_freq / SPI_SPEED_CLOCK_DIV64_MHZ)) {
    _spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  } else if (_freq >= (spi_freq / SPI_SPEED_CLOCK_DIV128_MHZ)) {
    _spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  } else {
    _spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  }

  if (HAL_SPI_Init(&_spi) != HAL_OK) {
    log_packet(LOG_ERROR, "TLE5012B", "SPI_INIT_ERR");
    status_led_set_critical_fault(true);
  }
}

uint16_t TLE5012BFullDuplex::readRawAngle() {
  uint8_t data[4];
  readBytes(TLE5012B_ANGLE_REG, data, 2);
  return (((uint16_t)data[0] << 8) | data[1]) & 0x7FFF;
}

float TLE5012BFullDuplex::getCurrentAngle() { return ((float)readRawAngle()) / TLE5012B_CPR * _2PI; }

float TLE5012BFullDuplex::getSensorAngle() { return fmodf(getCurrentAngle(), _2PI); }

void TLE5012BFullDuplex::readBytes(uint16_t reg, uint8_t *data, uint8_t len) {
  digitalWrite(_ncs, LOW);

  reg |= TLE5012B_READ_REGISTER + (len >> 1);
  uint8_t txbuffer[2] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0x00FF)};
  uint8_t dummy[2] = {0, 0};
  HAL_SPI_TransmitReceive(&_spi, txbuffer, dummy, 2, 100);
  // Datasheet requires a tiny delay after changing data direction (twr_delay ~130 ns).
  for (volatile int i = 0; i < 15; i++) {
    __NOP();
  }
  HAL_SPI_TransmitReceive(&_spi, dummy, data, len + 2, 100);

  digitalWrite(_ncs, HIGH);
}

#endif
