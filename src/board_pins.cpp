#include "board_pins.h"

#include <Arduino.h>

#include "uart_dma_stream.h"

void init_debug_led() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
}

void init_uart_comms(unsigned long baud) {
  init_uart_dma(baud);
#ifdef DEBUG_SERIAL
  // Serial debug output shares USART1 with DMA; enable only for debug builds.
  Serial.setRx(UART_RX_PIN);
  Serial.setTx(UART_TX_PIN);
  Serial.begin(baud);
#endif
}

void read_strap_pins(bool &strap0_low, bool &strap1_low) {
  pinMode(STRAP0_PIN, INPUT_PULLUP);
  pinMode(STRAP1_PIN, INPUT_PULLUP);
  strap0_low = (digitalRead(STRAP0_PIN) == LOW);
  strap1_low = (digitalRead(STRAP1_PIN) == LOW);
}
