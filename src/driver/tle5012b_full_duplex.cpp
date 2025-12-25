#include "tle5012b_full_duplex.h"

#if defined(_STM32_DEF_)

#include <cmath>

#include "log_packet.h"
#include "status_led.h"
#include "tle5012b_regs.h"
#include "utility/spi_com.h"
extern "C" uint32_t spi_getClkFreqInst(SPI_TypeDef* spi_inst);

// SPI protocol constants
constexpr uint16_t TLE5012B_READ_REGISTER = 0x8000;
constexpr uint16_t TLE5012B_SAFE_BIT = 0x0001;  // Request safety word

// CRC constants (from Infineon TLx5012 docs)
constexpr uint8_t CRC_POLYNOMIAL = 0x1D;
constexpr uint8_t CRC_SEED = 0xFF;

// Safety word bit masks
constexpr uint16_t SYSTEM_ERROR_MASK = 0x4000;
constexpr uint16_t INTERFACE_ERROR_MASK = 0x2000;
constexpr uint16_t INV_ANGLE_ERROR_MASK = 0x1000;

// Angle data masks
constexpr uint16_t DELETE_BIT_15 = 0x7FFF;

// STAT bit masks (from vendor bitFields table)
constexpr uint16_t STAT_SRST = (1u << 0);
constexpr uint16_t STAT_SWD = (1u << 1);
constexpr uint16_t STAT_SVR = (1u << 2);
constexpr uint16_t STAT_SFUSE = (1u << 3);
constexpr uint16_t STAT_SDSPU = (1u << 4);
constexpr uint16_t STAT_SOV = (1u << 5);
constexpr uint16_t STAT_SXYOL = (1u << 6);
constexpr uint16_t STAT_SMAGOL = (1u << 7);
constexpr uint16_t STAT_SADCT = (1u << 9);
constexpr uint16_t STAT_SROM = (1u << 10);
constexpr uint16_t STAT_NOGMRXY = (1u << 11);
constexpr uint16_t STAT_NOGMRA = (1u << 12);
constexpr uint16_t STAT_RDST = (1u << 15);

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

  SPI_TypeDef* spi_mosi =
      (SPI_TypeDef*)pinmap_peripheral(digitalPinToPinName(_mosi), PinMap_SPI_MOSI);
  SPI_TypeDef* spi_miso =
      (SPI_TypeDef*)pinmap_peripheral(digitalPinToPinName(_miso), PinMap_SPI_MISO);
  SPI_TypeDef* spi_sclk =
      (SPI_TypeDef*)pinmap_peripheral(digitalPinToPinName(_sck), PinMap_SPI_SCLK);
  SPI_TypeDef* spi_inst = (SPI_TypeDef*)pinmap_merge_peripheral(spi_mosi, spi_sclk);
  spi_inst = (SPI_TypeDef*)pinmap_merge_peripheral(spi_inst, spi_miso);

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

AngleReadResult TLE5012BFullDuplex::readRawAngleChecked() {
  uint8_t data[4] = {0};
  // Request 1 data word + safety word using REG_AVAL register
  readBytes(REG_AVAL | TLE5012B_SAFE_BIT, data, 2);

  uint16_t angle = (((uint16_t)data[0] << 8) | data[1]) & DELETE_BIT_15;
  const uint16_t safety = ((uint16_t)data[2] << 8) | data[3];

  if (checkSafety(safety) != NO_ERROR) {
    status_led_pulse();
    return {_lastValidAngle, false};
  }

  _lastValidAngle = angle;
  return {angle, true};
}

uint16_t TLE5012BFullDuplex::readRawAngle() {
  AngleReadResult result = readRawAngleChecked();
  return result.angle;  // Returns last valid on error (via readRawAngleChecked)
}

float TLE5012BFullDuplex::getCurrentAngle() {
  return static_cast<float>(readRawAngle()) / tle5012b::CPR * _2PI;
}

float TLE5012BFullDuplex::getSensorAngle() {
  return fmodf(getCurrentAngle(), _2PI);
}

bool TLE5012BFullDuplex::readStatus(uint16_t& stat_out) {
  uint8_t data[4] = {0};
  readBytes(REG_STAT | TLE5012B_SAFE_BIT, data, 2);
  uint16_t stat = ((uint16_t)data[0] << 8) | data[1];
  const uint16_t safety = ((uint16_t)data[2] << 8) | data[3];
  if (checkSafety(safety) != NO_ERROR) {
    return false;
  }
  _stat = stat;
  stat_out = stat;
  return true;
}

void TLE5012BFullDuplex::logStatusBits() {
  if (_stat == tle5012b::STAT_UNREAD) {
    return;  // Status has never been read
  }
  char msg[96] = {0};
  // Collect set flags.
  const uint16_t snr = (_stat >> 13) & 0x3;
  snprintf(msg, sizeof(msg), "STAT=0x%04X SNR=%u%s%s%s%s%s%s%s%s%s%s%s%s",
           static_cast<unsigned>(_stat), static_cast<unsigned>(snr),
           (_stat & STAT_SRST) ? " SRST" : "", (_stat & STAT_SWD) ? " SWD" : "",
           (_stat & STAT_SVR) ? " SVR" : "", (_stat & STAT_SFUSE) ? " SFUSE" : "",
           (_stat & STAT_SDSPU) ? " SDSPU" : "", (_stat & STAT_SOV) ? " SOV" : "",
           (_stat & STAT_SXYOL) ? " SXYOL" : "", (_stat & STAT_SMAGOL) ? " SMAGOL" : "",
           (_stat & STAT_SADCT) ? " SADCT" : "", (_stat & STAT_SROM) ? " SROM" : "",
           (_stat & STAT_NOGMRXY) ? " NOGMRXY" : "", (_stat & STAT_NOGMRA) ? " NOGMRA" : "",
           (_stat & STAT_RDST) ? " RDST" : "");
  log_packet(LOG_INFO, "TLE5012B", msg);
  snprintf(msg, sizeof(msg), "SAFETY=0x%04X, S:%u, I: %u, A: %u", static_cast<unsigned>(_safetyWord), 
           static_cast<unsigned>((_safetyWord & SYSTEM_ERROR_MASK) ? 1 : 0),
           static_cast<unsigned>((_safetyWord & INTERFACE_ERROR_MASK) ? 1 : 0),
           static_cast<unsigned>((_safetyWord & INV_ANGLE_ERROR_MASK) ? 1 : 0));
  log_packet(LOG_INFO, "TLE5012B", msg);
}

void TLE5012BFullDuplex::readBytes(uint16_t reg, uint8_t* data, uint8_t len) {
  digitalWrite(_ncs, LOW);

  reg |= TLE5012B_READ_REGISTER + (len >> 1);
  uint8_t txbuffer[2] = {static_cast<uint8_t>(reg >> 8), static_cast<uint8_t>(reg & 0x00FF)};
  uint8_t dummy[2] = {0, 0};
  HAL_SPI_TransmitReceive(&_spi, txbuffer, dummy, 2, tle5012b::SPI_TIMEOUT_MS);

  // Datasheet requires a tiny delay after changing data direction (twr_delay ~130 ns at 72MHz)
  for (volatile int i = 0; i < tle5012b::TWR_DELAY_NOPS; i++) {
    __NOP();
  }
  HAL_SPI_TransmitReceive(&_spi, dummy, data, len + 2, tle5012b::SPI_TIMEOUT_MS);

  digitalWrite(_ncs, HIGH);
}



errorTypes TLE5012BFullDuplex::checkSafety(uint16_t safety)
{
    errorTypes errorCheck = NO_ERROR;
    _safetyWord = safety;
    if (!((safety) & SYSTEM_ERROR_MASK))
    {
        errorCheck = SYSTEM_ERROR;
        // resetSafety();
    } else if (!((safety) & INTERFACE_ERROR_MASK))
    {
        errorCheck = INTERFACE_ACCESS_ERROR;
        // resetSafety();
    } else if (!((safety) & INV_ANGLE_ERROR_MASK))
    {
        errorCheck = INVALID_ANGLE_ERROR;
        // resetSafety();
    }else{
        resetSafety();
    }
    return (errorCheck);
}

void TLE5012BFullDuplex::resetSafety() {
  uint16_t reg = TLE5012B_READ_REGISTER | TLE5012B_SAFE_BIT;
  uint8_t rx[6] = {0};
  readBytes(reg, rx, 4);  // discard
}

uint8_t TLE5012BFullDuplex::crc8_calc(const uint8_t* data, uint8_t length) {
  uint32_t crc = CRC_SEED;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ CRC_POLYNOMIAL;
      } else {
        crc <<= 1;
      }
    }
  }
  return static_cast<uint8_t>((~crc) & CRC_SEED);
}

#endif
