#include "flash.h"

flash_status flash_erase(uint32_t address) {
  HAL_FLASH_Unlock();
  flash_status status = FLASH_ERROR;
  FLASH_EraseInitTypeDef erase_init;
  uint32_t error = 0u;

  erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
  erase_init.PageAddress = address;
  erase_init.Banks = FLASH_BANK_1;
  erase_init.NbPages = (FLASH_APP_END_ADDRESS - address) / FLASH_PAGE_SIZE;

  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
  if (HAL_OK == HAL_FLASHEx_Erase(&erase_init, &error)) {
    status = FLASH_OK;
  }

  HAL_FLASH_Lock();
  return status;
}

flash_status flash_write(uint32_t address, const uint8_t *data, uint32_t length_bytes) {
  flash_status status = FLASH_OK;

  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);

  for (uint32_t i = 0u; (i < length_bytes) && (FLASH_OK == status); i += 2u) {
    if (FLASH_APP_END_ADDRESS <= address) {
      status |= FLASH_ERROR_SIZE;
      break;
    }
    uint16_t half = 0xFFFF;
    if (i + 1 < length_bytes) {
      half = (uint16_t)data[i] | ((uint16_t)data[i + 1] << 8);
    } else {
      half = (uint16_t)data[i] | 0xFF00;
    }
    if (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, half)) {
      status |= FLASH_ERROR_WRITE;
      break;
    }
    if ((uint16_t)half != *(volatile uint16_t *)address) {
      status |= FLASH_ERROR_READBACK;
      break;
    }
    address += 2u;
  }

  HAL_FLASH_Lock();
  return status;
}
