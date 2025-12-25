#pragma once

#include <stdbool.h>

// Initialize status LED GPIO (PA1).
void status_led_init();

// Mark critical init failure (for future use).
void status_led_set_critical_fault(bool active);

// Trigger a pulse - LED will show OFF for PULSE_DURATION_MS.
// Non-blocking, just records timestamp.
void status_led_pulse();

// Returns true if currently in a pulse window (LED should be OFF).
// Call this from main loop to determine LED state.
bool status_led_is_pulsing();
