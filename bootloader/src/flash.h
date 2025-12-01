#pragma once

#include "stm32f1xx_hal.h"

#define FLASH_APP_START_ADDRESS ((uint32_t)0x08002000U)
#define FLASH_APP_END_ADDRESS ((uint32_t)0x08010000U)

typedef enum {
  FLASH_OK = 0x00u,
  FLASH_ERROR_SIZE = 0x01u,
  FLASH_ERROR_WRITE = 0x02u,
  FLASH_ERROR_READBACK = 0x04u,
  FLASH_ERROR = 0xFFu
} flash_status;

flash_status flash_erase(uint32_t address);
flash_status flash_write(uint32_t address, const uint8_t *data, uint32_t length_bytes);
