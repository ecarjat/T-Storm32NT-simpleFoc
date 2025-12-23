#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>

#if defined(_STM32_DEF_)

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

class TLE5012BFullDuplex : public Sensor {
 public:
  TLE5012BFullDuplex(int mosi, int miso, int sck, int ncs, uint32_t freq_hz = 1000000);
  ~TLE5012BFullDuplex() = default;

  void init() override;
  uint16_t readRawAngle();
  float getCurrentAngle();  // radians

  // Read STAT register with safety/CRC; returns true on success.
  bool readStatus(uint16_t& stat_out);
  // Log set bits of STAT via log_packet.
  void logStatusBits();

 protected:
  float getSensorAngle() override;  // wrapped 0..2PI

 private:
  /**
   * @brief Error types from safety word
   */
  void readBytes(uint16_t reg, uint8_t* data, uint8_t len);
  errorTypes checkSafety(uint16_t safety);
  void resetSafety();
  uint8_t crc8_calc(const uint8_t* data, uint8_t length);

  /*!
   * Gets the first byte of a 2 byte word
   * @param twoByteWord insert word of two bytes long
   * @return returns the first byte
   */
  uint8_t getFirstByte(uint16_t twoByteWord) { return ((uint8_t)(twoByteWord >> 8)); }

  /*!
   * Gets the second byte of the 2 byte word
   * @param twoByteWord insert word of two bytes long
   * @return returns the second byte
   */
  uint8_t getSecondByte(uint16_t twoByteWord) { return ((uint8_t)twoByteWord); }

  int _mosi;
  int _miso;
  int _sck;
  int _ncs;
  uint32_t _freq;
  SPI_HandleTypeDef _spi;
  uint16_t _stat = -1;
  uint16_t _safetyWord = 0;
};

#endif
