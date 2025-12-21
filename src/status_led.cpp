#include "status_led.h"

#include <Arduino.h>

#include "board_pins.h"

namespace {
constexpr uint16_t BOOT_BLINK_MS = 250;
constexpr uint16_t PULSE_MS = 100;
constexpr uint16_t CRITICAL_PULSE_MS = 100;

bool led_initialized = false;
bool is_running = false;
bool critical_fault = false;

uint32_t last_boot_toggle_ms = 0;
bool boot_state = false;

uint32_t pulse_until_ms = 0;
volatile bool pulse_pending = false;

uint32_t critical_toggle_ms = 0;
bool critical_state = false;

void led_write(bool on) {
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
}

void ensure_init() {
  if (led_initialized) {
    return;
  }
  pinMode(STATUS_LED_PIN, OUTPUT);
  led_write(false);
  led_initialized = true;
  last_boot_toggle_ms = millis();
  critical_toggle_ms = last_boot_toggle_ms;
}
}  // namespace

void status_led_init() {
  ensure_init();
}

void status_led_set_running(bool running) {
  is_running = running;
}

void status_led_set_critical_fault(bool active) {
  critical_fault = active;
  if (active) {
    critical_toggle_ms = millis();
  }
}

void status_led_pulse() {
  ensure_init();
  pulse_until_ms = millis() + PULSE_MS;
}

void status_led_pulse_isr() {
  pulse_pending = true;
}

void status_led_tick() {
  ensure_init();

  if (pulse_pending) {
    pulse_pending = false;
    pulse_until_ms = millis() + PULSE_MS;
  }

  const uint32_t now = millis();

  if (critical_fault) {
    if (now - critical_toggle_ms >= CRITICAL_PULSE_MS) {
      critical_state = !critical_state;
      critical_toggle_ms = now;
    }
    led_write(critical_state);
    return;
  }

  bool base_state = false;
  if (is_running) {
    base_state = true;
  } else {
    if (now - last_boot_toggle_ms >= BOOT_BLINK_MS) {
      boot_state = !boot_state;
      last_boot_toggle_ms = now;
    }
    base_state = boot_state;
  }

  if (pulse_until_ms && (int32_t)(pulse_until_ms - now) > 0) {
    led_write(!base_state);
  } else {
    pulse_until_ms = 0;
    led_write(base_state);
  }
}
