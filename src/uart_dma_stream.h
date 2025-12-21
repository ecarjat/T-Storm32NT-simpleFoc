#pragma once

#include <Arduino.h>

// Stream-compatible UART1 DMA transport (HAL/LL backed).
class UartDmaStream : public Stream {
public:
  UartDmaStream();

  void begin(uint32_t baud);

  int available() override;
  int read() override;
  int peek() override;
  size_t write(uint8_t value) override;
  size_t write(const uint8_t *buffer, size_t size) override;
  void flush() override;

  using Print::write;

  uint32_t rx_overruns() const;
  uint32_t tx_drops() const;
};

// Singleton access used by comms/streams.
UartDmaStream &uart_dma_stream();

// Initialize USART1 + DMA (PA9/PA10). Non-blocking TX/RX.
void init_uart_dma(uint32_t baud);
