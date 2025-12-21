#include "uart_dma_stream.h"

#include <Arduino.h>

extern "C" {
#include "stm32f1xx_hal.h"
}

namespace {
constexpr uint16_t UART_RX_BUFFER_SIZE = 256;
constexpr uint16_t UART_TX_BUFFER_SIZE = 256;

UART_HandleTypeDef g_uart1;
DMA_HandleTypeDef g_dma_uart1_rx;
DMA_HandleTypeDef g_dma_uart1_tx;

uint8_t g_rx_buffer[UART_RX_BUFFER_SIZE];
uint8_t g_tx_buffer[UART_TX_BUFFER_SIZE];

volatile uint16_t g_rx_tail = 0;
volatile uint16_t g_rx_last_head = 0;

volatile uint16_t g_tx_head = 0;
volatile uint16_t g_tx_tail = 0;
volatile uint16_t g_tx_dma_len = 0;
volatile bool g_tx_dma_active = false;

volatile uint32_t g_rx_overruns = 0;
volatile uint32_t g_tx_drops = 0;

bool g_uart_initialized = false;

inline uint16_t rx_dma_head() {
  uint16_t remaining = __HAL_DMA_GET_COUNTER(&g_dma_uart1_rx);
  uint16_t head = (UART_RX_BUFFER_SIZE - remaining);
  if (head >= UART_RX_BUFFER_SIZE) {
    head = 0;
  }
  return head;
}

inline uint16_t ring_available(uint16_t head, uint16_t tail, uint16_t size) {
  if (head >= tail) {
    return head - tail;
  }
  return size - tail + head;
}

inline uint16_t ring_next(uint16_t value, uint16_t size) {
  value++;
  if (value >= size) {
    value = 0;
  }
  return value;
}

void kick_tx_dma() {
  if (!g_uart_initialized) {
    return;
  }
  if (g_tx_dma_active) {
    return;
  }
  uint16_t tail = g_tx_tail;
  uint16_t head = g_tx_head;
  if (tail == head) {
    return;
  }

  uint16_t len = 0;
  if (head > tail) {
    len = head - tail;
  } else {
    len = UART_TX_BUFFER_SIZE - tail;
  }
  if (len == 0) {
    return;
  }

  g_tx_dma_len = len;
  g_tx_dma_active = true;

  DMA1_Channel4->CCR &= ~DMA_CCR_EN;
  DMA1->IFCR = DMA_IFCR_CGIF4 | DMA_IFCR_CTCIF4 | DMA_IFCR_CHTIF4 | DMA_IFCR_CTEIF4;
  DMA1_Channel4->CMAR = reinterpret_cast<uint32_t>(&g_tx_buffer[tail]);
  DMA1_Channel4->CNDTR = len;
  DMA1_Channel4->CCR |= DMA_CCR_EN;
}

void init_gpio_usart1() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  GPIO_InitTypeDef gpio{};
  gpio.Pin = GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);
}

void init_dma_usart1() {
  __HAL_RCC_DMA1_CLK_ENABLE();

  g_dma_uart1_rx.Instance = DMA1_Channel5;
  g_dma_uart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  g_dma_uart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  g_dma_uart1_rx.Init.MemInc = DMA_MINC_ENABLE;
  g_dma_uart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  g_dma_uart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  g_dma_uart1_rx.Init.Mode = DMA_CIRCULAR;
  g_dma_uart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
  HAL_DMA_Init(&g_dma_uart1_rx);

  g_dma_uart1_tx.Instance = DMA1_Channel4;
  g_dma_uart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  g_dma_uart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  g_dma_uart1_tx.Init.MemInc = DMA_MINC_ENABLE;
  g_dma_uart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  g_dma_uart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  g_dma_uart1_tx.Init.Mode = DMA_NORMAL;
  g_dma_uart1_tx.Init.Priority = DMA_PRIORITY_LOW;
  HAL_DMA_Init(&g_dma_uart1_tx);

  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

  DMA1_Channel5->CPAR = reinterpret_cast<uint32_t>(&USART1->DR);
  DMA1_Channel5->CMAR = reinterpret_cast<uint32_t>(g_rx_buffer);
  DMA1_Channel5->CNDTR = UART_RX_BUFFER_SIZE;
  DMA1_Channel5->CCR |= DMA_CCR_EN;

  DMA1_Channel4->CPAR = reinterpret_cast<uint32_t>(&USART1->DR);
  DMA1_Channel4->CCR |= DMA_CCR_TCIE;
}
}  // namespace

UartDmaStream::UartDmaStream() = default;

void UartDmaStream::begin(uint32_t baud) {
  init_uart_dma(baud);
}

int UartDmaStream::available() {
  if (!g_uart_initialized) {
    return 0;
  }
  uint16_t head = rx_dma_head();
  uint16_t tail = g_rx_tail;
  uint16_t avail = ring_available(head, tail, UART_RX_BUFFER_SIZE);
  if (avail == 0 && head != g_rx_last_head) {
    g_rx_overruns++;
    g_rx_tail = head;
  }
  g_rx_last_head = head;
  return avail;
}

int UartDmaStream::read() {
  if (!g_uart_initialized || available() <= 0) {
    return -1;
  }
  uint8_t value = g_rx_buffer[g_rx_tail];
  g_rx_tail = ring_next(g_rx_tail, UART_RX_BUFFER_SIZE);
  return value;
}

int UartDmaStream::peek() {
  if (!g_uart_initialized || available() <= 0) {
    return -1;
  }
  return g_rx_buffer[g_rx_tail];
}

size_t UartDmaStream::write(uint8_t value) {
  return write(&value, 1);
}

size_t UartDmaStream::write(const uint8_t *buffer, size_t size) {
  if (!buffer || size == 0) {
    return 0;
  }
  size_t written = 0;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  for (; written < size; ++written) {
    uint16_t next = ring_next(g_tx_head, UART_TX_BUFFER_SIZE);
    if (next == g_tx_tail) {
      g_tx_drops += (size - written);
      break;
    }
    g_tx_buffer[g_tx_head] = buffer[written];
    g_tx_head = next;
  }
  if (!primask) {
    __enable_irq();
  }
  kick_tx_dma();
  return written;
}

void UartDmaStream::flush() {
  kick_tx_dma();
}

uint32_t UartDmaStream::rx_overruns() const {
  return g_rx_overruns;
}

uint32_t UartDmaStream::tx_drops() const {
  return g_tx_drops;
}

UartDmaStream &uart_dma_stream() {
  static UartDmaStream instance;
  return instance;
}

void init_uart_dma(uint32_t baud) {
  if (g_uart_initialized) {
    return;
  }

  __HAL_RCC_USART1_CLK_ENABLE();
  init_gpio_usart1();

  g_uart1.Instance = USART1;
  g_uart1.Init.BaudRate = baud;
  g_uart1.Init.WordLength = UART_WORDLENGTH_8B;
  g_uart1.Init.StopBits = UART_STOPBITS_1;
  g_uart1.Init.Parity = UART_PARITY_NONE;
  g_uart1.Init.Mode = UART_MODE_TX_RX;
  g_uart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  g_uart1.Init.OverSampling = UART_OVERSAMPLING_16;

  HAL_UART_DeInit(&g_uart1);
  HAL_UART_Init(&g_uart1);

  init_dma_usart1();

  USART1->CR3 |= USART_CR3_DMAR | USART_CR3_DMAT;

  g_tx_head = 0;
  g_tx_tail = 0;
  g_tx_dma_len = 0;
  g_tx_dma_active = false;
  g_rx_tail = 0;
  g_rx_last_head = 0;

  g_uart_initialized = true;
}

extern "C" void DMA1_Channel4_IRQHandler() {
  uint32_t isr = DMA1->ISR;
  if (isr & DMA_ISR_TEIF4) {
    DMA1->IFCR = DMA_IFCR_CTEIF4 | DMA_IFCR_CGIF4;
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    g_tx_dma_active = false;
    g_tx_drops++;
  }
  if (isr & DMA_ISR_TCIF4) {
    DMA1->IFCR = DMA_IFCR_CTCIF4 | DMA_IFCR_CGIF4;
    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    uint16_t advance = g_tx_dma_len;
    g_tx_dma_len = 0;
    g_tx_tail = (g_tx_tail + advance) % UART_TX_BUFFER_SIZE;
    g_tx_dma_active = false;
    kick_tx_dma();
  }
}
