/* Minimal UART bootloader for STM32F103C8/T8 without Arduino core.
 * Bootloader size target: <=8 KB.
 * Layout: bootloader at 0x08000000 (8 KB), app at 0x08002000 (VECT_TAB_OFFSET=0x2000).
 * Protocol: "UPD0" + <uint32 length> + <uint32 crc32> little-endian, reply "OK", stream payload, reply "OK" on success.
 * Reset-to-bootloader: app writes BOOT_MAGIC to BKP->DR1 and triggers NVIC_SystemReset().
 */

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define STRINGIFY_(x) #x
#define STRINGIFY(x) STRINGIFY_(x)

#ifndef APP_START_ADDR
#define APP_START_ADDR 0x08002000UL
#endif
#define FLASH_END_ADDR 0x08010000UL
#define APP_FLASH_PAGE_SIZE 1024UL

#define BOOT_MAGIC 0xB007U
#define HEADER_MAGIC 0x30445055UL /* "UPD0" */
#define HANDSHAKE_TIMEOUT_MS 500U
#define RX_BUF_SIZE 256U

#define LED_PORT GPIOA
#define LED_PIN GPIO_PIN_1

UART_HandleTypeDef huart1;

static uint8_t rx_buf[RX_BUF_SIZE];

/* CRC32 (poly 0xEDB88320) */
static uint32_t crc32_update(uint32_t crc, const uint8_t *data, uint32_t len) {
  crc = ~crc;
  for (uint32_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint32_t j = 0; j < 8; j++) {
      uint32_t mask = -(crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

static bool app_is_valid(void) {
  uint32_t sp = *(__IO uint32_t *)APP_START_ADDR;
  uint32_t rv = *(__IO uint32_t *)(APP_START_ADDR + 4U);
  bool sp_ok = (sp >= 0x20000000UL) && (sp <= 0x20005000UL);
  bool rv_ok = (rv >= APP_START_ADDR) && (rv < FLASH_END_ADDR);
  return sp_ok && rv_ok;
}

__attribute__((naked, noreturn)) static void jump_to_app(void) {
  __asm volatile(
      "ldr r0, =" STRINGIFY(APP_START_ADDR) "\n"  // r0 = app base
      "ldr r1, [r0]\n"                             // r1 = app SP
      "ldr r2, [r0, #4]\n"                         // r2 = app entry
      "ldr r3, =0xE000ED08\n"                      // VTOR
      "msr msp, r1\n"                              // set MSP
      "str r0, [r3]\n"                             // VTOR = APP_START_ADDR
      "cpsie i\n"
      "bx r2\n");
}

static void SystemClock_Config(void) {
  /* Use HSI (8 MHz) default */
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

static void GPIO_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* LED */
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
  /* USART1 TX/RX */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void USART1_Init(void) {
  __HAL_RCC_USART1_CLK_ENABLE();
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);
}

static void send_str(const char *s) {
  HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

static bool read_exact(uint8_t *dst, uint32_t len, uint32_t timeout_ms) {
  return HAL_UART_Receive(&huart1, dst, len, timeout_ms) == HAL_OK;
}

static bool erase_app_flash(void) {
  HAL_FLASH_Unlock();
  uint32_t page_error = 0;
  FLASH_EraseInitTypeDef erase = {0};
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = APP_START_ADDR;
  erase.NbPages = (FLASH_END_ADDR - APP_START_ADDR) / APP_FLASH_PAGE_SIZE;
  HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &page_error);
  HAL_FLASH_Lock();
  return (st == HAL_OK) && (page_error == 0xFFFFFFFFUL);
}

static bool program_chunk(uint32_t addr, const uint8_t *data, uint32_t len) {
  HAL_FLASH_Unlock();
  for (uint32_t i = 0; i < len; i += 2) {
    uint16_t half = data[i];
    if (i + 1 < len) {
      half |= (uint16_t)data[i + 1] << 8;
    } else {
      half |= 0xFF00;
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, half) != HAL_OK) {
      HAL_FLASH_Lock();
      return false;
    }
  }
  HAL_FLASH_Lock();
  return true;
}

int main(void) {
  HAL_Init();
  SystemClock_Config();
  GPIO_Init();
  USART1_Init();

  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_BKP_CLK_ENABLE();

  uint8_t hdr_buf[12];
  uint32_t last_blink = 0;
  bool force_update = false;

  /* Check boot magic */
  if (READ_REG(BKP->DR1) == BOOT_MAGIC) {
    force_update = true;
    WRITE_REG(BKP->DR1, 0);
  }

  while (1) {
    /* Slow blink in boot mode (~1 Hz) */
    uint32_t now = HAL_GetTick();
    if ((now - last_blink) > 1000U) {
      HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
      last_blink = now;
    }

    /* Handshake timeout if not forced */
    if (!force_update) {
      if (HAL_UART_Receive(&huart1, hdr_buf, 4, HANDSHAKE_TIMEOUT_MS) != HAL_OK) {
        if (app_is_valid()) {
          jump_to_app();
        } else {
          continue;
        }
      }
    } else {
      if (HAL_UART_Receive(&huart1, hdr_buf, 4, HANDSHAKE_TIMEOUT_MS) != HAL_OK) {
        continue;
      }
    }

    uint32_t magic = hdr_buf[0] | (hdr_buf[1] << 8) | (hdr_buf[2] << 16) | (hdr_buf[3] << 24);
    if (magic != HEADER_MAGIC) {
      continue;
    }

    /* Read length and crc */
    if (!read_exact(&hdr_buf[4], 8, 200)) {
      continue;
    }
    uint32_t length = hdr_buf[4] | (hdr_buf[5] << 8) | (hdr_buf[6] << 16) | (hdr_buf[7] << 24);
    uint32_t crc_expected = hdr_buf[8] | (hdr_buf[9] << 8) | (hdr_buf[10] << 16) | (hdr_buf[11] << 24);

    if (length == 0 || (APP_START_ADDR + length) > FLASH_END_ADDR) {
      send_str("ERLEN\n");
      continue;
    }

    send_str("OK\n");

    if (!erase_app_flash()) {
      send_str("ERASE\n");
      continue;
    }

    uint32_t addr = APP_START_ADDR;
    uint32_t remaining = length;
    uint32_t crc = 0;
    while (remaining > 0) {
      uint32_t chunk = (remaining > RX_BUF_SIZE) ? RX_BUF_SIZE : remaining;
      if (!read_exact(rx_buf, chunk, 5000)) {
        send_str("TIMEOUT\n");
        break;
      }
      crc = crc32_update(crc, rx_buf, chunk);
      if (!program_chunk(addr, rx_buf, chunk)) {
        send_str("PFAIL\n");
        break;
      }
      addr += chunk;
      remaining -= chunk;
    }

    if (remaining > 0) {
      continue;
    }

    if (crc != crc_expected) {
      send_str("CRC\n");
      continue;
    }

    send_str("OK\n");
    jump_to_app();
  }
}

/* Minimal HAL callbacks */
void SysTick_Handler(void) { HAL_IncTick(); }
