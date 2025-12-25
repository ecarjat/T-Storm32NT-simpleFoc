#pragma once

#include "stm32f1xx_hal.h"

// APP_START_ADDR is defined in platformio.ini build_flags (-D APP_START_ADDR=0x08001800)
// This is the single source of truth for the application start address.
#ifndef APP_START_ADDR
#error "APP_START_ADDR must be defined in build_flags"
#endif

// Settings region starts at FLASH_APP_END_ADDRESS (3KB reserved at end of 64KB flash)
#define FLASH_APP_START_ADDRESS ((uint32_t)APP_START_ADDR)
#define FLASH_APP_END_ADDRESS   ((uint32_t)0x0800F400U)  // 64KB - 3KB settings = 0x10000 - 0xC00

typedef enum {
  FLASH_OK = 0x00u,
  FLASH_ERROR_SIZE = 0x01u,
  FLASH_ERROR_WRITE = 0x02u,
  FLASH_ERROR_READBACK = 0x04u,
  FLASH_ERROR = 0xFFu
} flash_status;

flash_status flash_erase(uint32_t address);
flash_status flash_write(uint32_t address, const uint8_t *data, uint32_t length_bytes);
