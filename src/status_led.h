#pragma once

#include <stdbool.h>

// Initialize status LED GPIO (PA1) and internal state.
void status_led_init();

// Update LED state; call frequently from the main loop.
void status_led_tick();

// Mark system running (solid ON when no pulses/boot).
void status_led_set_running(bool running);

// Mark critical init failure; keeps LED in continuous pulse mode.
void status_led_set_critical_fault(bool active);

// Trigger a short, non-blocking pulse.
void status_led_pulse();

// ISR-safe pulse request (handled on next tick).
void status_led_pulse_isr();
