#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>

#if defined(_STM32_DEF_)

// TLE5012B timing and SPI constants
namespace tle5012b {
  constexpr float CPR = 32768.0f;                 // Counts per revolution (15-bit)
  constexpr int TWR_DELAY_NOPS = 15;              // ~130ns delay at 72MHz after direction change
  constexpr uint32_t SPI_TIMEOUT_MS = 10;         // SPI transaction timeout
}

enum errorTypes {
  NO_ERROR = 0x00,  //!< \brief NO_ERROR = Safety word was OK
  SYSTEM_ERROR =
      0x01,  //!< \brief SYSTEM_ERROR = over/under voltage, VDD negative, GND off, ROM defect
  INTERFACE_ACCESS_ERROR = 0x02,  //!< \brief INTERFACE_ACCESS_ERROR = wrong address or wrong lock
  INVALID_ANGLE_ERROR = 0x03,     //!< \brief INVALID_ANGLE_ERROR = NO_GMR_A = 1 or NO_GMR_XY = 1
  ANGLE_SPEED_ERROR =
      0x04,         //!< \brief ANGLE_SPEED_ERROR = combined error, angular speed calculation wrong
  CRC_ERROR = 0xFF  //!< \brief CRC_ERROR = Cyclic Redundancy Check (CRC), which includes the STAT
                    //!< and RESP bits wrong
};

// Result struct for angle reads with validity flag
struct AngleReadResult {
  uint16_t angle;
  bool valid;
};

class TLE5012BFullDuplex : public Sensor {
 public:
  TLE5012BFullDuplex(int mosi, int miso, int sck, int ncs, uint32_t freq_hz = 1000000);
  ~TLE5012BFullDuplex() = default;

  void init() override;

  // Read raw angle with validity check
  AngleReadResult readRawAngleChecked();

  // Legacy interface (returns last good value on error)
  uint16_t readRawAngle();
  float getCurrentAngle();  // radians


 protected:
  float getSensorAngle() override;  // wrapped 0..2PI

 private:
  /**
   * @brief Error types from safety word
   */
  uint16_t readBytes(uint16_t reg, uint8_t* data, uint8_t len);
  bool validateCrc(uint16_t cmdWord, const uint8_t* data, uint8_t dataLen, uint16_t safety);
  errorTypes checkSafety(uint16_t safety, uint16_t cmdWord, const uint8_t* data, uint8_t len);
  void resetSafety();
  uint8_t crc8_calc(const uint8_t* data, uint8_t length);

  int _mosi;
  int _miso;
  int _sck;
  int _ncs;
  uint32_t _freq;
  SPI_HandleTypeDef _spi;
  GPIO_TypeDef* _ncs_port = nullptr;  // Cached for fast CS toggle
  uint16_t _ncs_pin = 0;              // Cached pin mask
  uint16_t _safetyWord = 0;
  uint16_t _lastValidAngle = 0;  // Last successfully read angle (for fallback)
  bool _initFailed = true;       // True until init() succeeds
};

#endif
