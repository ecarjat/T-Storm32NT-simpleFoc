#include "board_pins.h"

#include <Arduino.h>

void init_debug_led() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
}

void init_uart_comms(unsigned long baud) {
  // UART1 (PA9/PA10) is wired to the NT bus. Default Serial maps to a different UART,
  // so explicitly select pins to bind Serial to USART1.
  Serial.setRx(UART_RX_PIN);
  Serial.setTx(UART_TX_PIN);
  Serial.begin(baud);
}

void read_strap_pins(bool &strap0_low, bool &strap1_low) {
  pinMode(STRAP0_PIN, INPUT_PULLUP);
  pinMode(STRAP1_PIN, INPUT_PULLUP);
  strap0_low = (digitalRead(STRAP0_PIN) == LOW);
  strap1_low = (digitalRead(STRAP1_PIN) == LOW);
}
