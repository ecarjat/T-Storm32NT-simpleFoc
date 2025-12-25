#include "status_led.h"

#include <Arduino.h>

#include "board_pins.h"

namespace {
constexpr uint32_t PULSE_DURATION_MS = 50;
constexpr uint32_t CRITICAL_BLINK_MS = 200;  // Fast blink for critical fault

bool led_initialized = false;
bool critical_fault = false;
uint32_t pulse_start_ms = 0;
}  // namespace

void status_led_init() {
  if (led_initialized) {
    return;
  }
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  led_initialized = true;
}

void status_led_set_critical_fault(bool active) {
  critical_fault = active;
}

void status_led_pulse() {
  pulse_start_ms = millis();
  if (pulse_start_ms == 0) {
    pulse_start_ms = 1;  // Avoid 0 which means "no pulse"
  }
}

bool status_led_is_pulsing() {
  // Critical fault: fast continuous blink (LED OFF during half the cycle)
  if (critical_fault) {
    return ((millis() / CRITICAL_BLINK_MS) % 2) == 0;
  }

  // Normal pulse logic
  if (pulse_start_ms == 0) {
    return false;
  }
  uint32_t elapsed = millis() - pulse_start_ms;
  if (elapsed >= PULSE_DURATION_MS) {
    pulse_start_ms = 0;  // Pulse complete
    return false;
  }
  return true;
}

bool status_led_has_critical_fault() {
  return critical_fault;
}
