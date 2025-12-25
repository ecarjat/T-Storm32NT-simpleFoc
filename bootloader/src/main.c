#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "flash.h"

#ifndef APP_START_ADDR
#define APP_START_ADDR FLASH_APP_START_ADDRESS
#endif
#define FLASH_END_ADDR FLASH_APP_END_ADDRESS
#define BOOT_MAGIC 0xB007U
#define HEADER_MAGIC 0x30445055UL /* "UPD0" */
#define RX_BUF_SIZE 256U

UART_HandleTypeDef huart1;

static uint8_t rx_buf[RX_BUF_SIZE];

/* No longer used
static void log_wrpr(void) {
  FLASH_OBProgramInitTypeDef ob = {0};
  HAL_FLASHEx_OBGetConfig(&ob);
  char buf[16];
  int n = 0;
  buf[n++] = 'W';
  buf[n++] = 'R';
  buf[n++] = 'P';
  buf[n++] = '=';
  uint16_t val = (uint16_t)ob.WRPPage;
  uint8_t hi = (val >> 12) & 0xF;
  uint8_t midh = (val >> 8) & 0xF;
  uint8_t midl = (val >> 4) & 0xF;
  uint8_t lo = val & 0xF;
  buf[n++] = (hi < 10) ? ('0' + hi) : ('A' + hi - 10);
  buf[n++] = (midh < 10) ? ('0' + midh) : ('A' + midh - 10);
  buf[n++] = (midl < 10) ? ('0' + midl) : ('A' + midl - 10);
  buf[n++] = (lo < 10) ? ('0' + lo) : ('A' + lo - 10);
  buf[n++] = '\n';
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)n, 100);
}
*/

static void send_hex8(uint8_t v) {
  char buf[5];
  buf[0] = '0';
  buf[1] = 'x';
  uint8_t hi = (v >> 4) & 0xF;
  uint8_t lo = v & 0xF;
  buf[2] = (hi < 10) ? ('0' + hi) : ('A' + hi - 10);
  buf[3] = (lo < 10) ? ('0' + lo) : ('A' + lo - 10);
  buf[4] = '\n';
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, sizeof(buf), 100);
}

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

static bool read_exact(uint8_t *dst, uint32_t len, uint32_t timeout_ms) {
  return HAL_UART_Receive(&huart1, dst, len, timeout_ms) == HAL_OK;
}

static void send_str(const char *s) { HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 100); }

static bool app_is_valid(void) {
  uint32_t sp = *(__IO uint32_t *)APP_START_ADDR;
  uint32_t rv = *(__IO uint32_t *)(APP_START_ADDR + 4U);
  bool sp_ok = (sp >= 0x20000000UL) && (sp <= 0x20005000UL);
  bool rv_ok = (rv >= APP_START_ADDR) && (rv < FLASH_END_ADDR);
  return sp_ok && rv_ok;
}

__attribute__((noreturn)) static void jump_to_app(void) {
  uint32_t app_addr = APP_START_ADDR;
  uint32_t sp = *(__IO uint32_t *)app_addr;
  uint32_t rv = *(__IO uint32_t *)(app_addr + 4U);

  // Set vector table offset register
  SCB->VTOR = app_addr;

  // Set stack pointer and jump
  __asm volatile(
      "msr msp, %0\n"
      "cpsie i\n"
      "bx %1\n"
      :
      : "r"(sp), "r"(rv)
      : "memory");

  // Should never reach here
  while (1) {}
}

static void SystemClock_Config(void) {
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
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void GPIO_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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

int main(void) {
  HAL_Init();
  SystemClock_Config();
  GPIO_Init();
  USART1_Init();

  // bootloader indicator
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // solid on at start
  // log_wrpr();

  // BOOT magic: if not set and app valid, jump
  if (READ_REG(BKP->DR1) != BOOT_MAGIC) {
    if (app_is_valid()) {
      jump_to_app();
    }
  } else {
    WRITE_REG(BKP->DR1, 0);
  }

  // Wait for header
  uint8_t hdr_buf[12];
  send_str("BOOT\n");
  while (1) {
    // slow blink while waiting
    static uint32_t last = 0;
    if (HAL_GetTick() - last > 1000) {
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
      last = HAL_GetTick();
    }
    if (HAL_UART_Receive(&huart1, hdr_buf, 4, 2000) != HAL_OK) {
      continue;
    }
    uint32_t magic = hdr_buf[0] | (hdr_buf[1] << 8) | (hdr_buf[2] << 16) | (hdr_buf[3] << 24);
    if (magic != HEADER_MAGIC) {
      continue;
    }
    if (!read_exact(&hdr_buf[4], 8, 500)) {
      continue;
    }
    uint32_t length = hdr_buf[4] | (hdr_buf[5] << 8) | (hdr_buf[6] << 16) | (hdr_buf[7] << 24);
    uint32_t crc_expected = hdr_buf[8] | (hdr_buf[9] << 8) | (hdr_buf[10] << 16) | (hdr_buf[11] << 24);
    if (length == 0 || (APP_START_ADDR + length) > FLASH_END_ADDR) {
      send_str("ERLEN\n");
      continue;
    }
    if (flash_erase(APP_START_ADDR) != FLASH_OK) {
      send_str("ERASE\n");
      continue;
    }
    // verify first word is erased
    if (*(__IO uint32_t *)APP_START_ADDR != 0xFFFFFFFFU) {
      send_str("ERCHK\n");
      continue;
    }
    send_str("OK\n");
    uint32_t addr = APP_START_ADDR;
    uint32_t remaining = length;
    uint32_t crc = 0;
    bool ok = true;
    while (remaining > 0 && ok) {
      uint32_t chunk = (remaining > RX_BUF_SIZE) ? RX_BUF_SIZE : remaining;
      if (!read_exact(rx_buf, chunk, 5000)) {
        send_str("TIMEOUT\n");
        ok = false;
        break;
      }
      send_str("CHK RCV\n ");
      crc = crc32_update(crc, rx_buf, chunk);
      uint8_t buf[RX_BUF_SIZE + 4];
      memset(buf, 0xFF, sizeof(buf));
      memcpy(buf, rx_buf, chunk);
      flash_status st = flash_write(addr, buf, chunk);
      if (st != FLASH_OK) {
        send_str("PFAIL ");
        send_hex8((uint8_t)st);
        ok = false;
        break;
      }
      send_str("ACK\n");
      addr += chunk;
      remaining -= chunk;
    }
    if (!ok) {
      continue;
    }
    if (crc != crc_expected) {
      send_str("CRC\n");
      continue;
    }
    send_str("OK\n");
    HAL_Delay(100);
    jump_to_app();
  }
}

void SysTick_Handler(void) { HAL_IncTick(); }
void Error_Handler(void) { while (1) {} }
