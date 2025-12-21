#pragma once

#include <Arduino.h>

// DRV8313 phase inputs (3-PWM, always enabled, no FAULT line)
constexpr uint8_t PHASE_U_PIN = PA3; // IN1 → phase U
constexpr uint8_t PHASE_V_PIN = PB0; // IN2 → phase V
constexpr uint8_t PHASE_W_PIN = PB1; // IN3 → phase W

// TLE5012B encoder SPI (3-wire DATA: MISO + MOSI share the same net via R5)
constexpr uint8_t ENC_CS_PIN = PA8;
constexpr uint8_t ENC_SCK_PIN = PA5;
constexpr uint8_t ENC_MISO_PIN = PA6; // DATA input to MCU
constexpr uint8_t ENC_MOSI_PIN = PA7; // DATA output via series R5

// UART1 for NT bus comms
constexpr uint8_t UART_TX_PIN = PA9;
constexpr uint8_t UART_RX_PIN = PA10;

// Status LED and strap pins
constexpr uint8_t STATUS_LED_PIN = PA1; // Active high LED2
constexpr uint8_t STRAP0_PIN = PA2;     // SJ2 optional strap to GND
constexpr uint8_t STRAP1_PIN = PA4;     // SJ3 optional strap to GND

// Firmware configuration
constexpr bool BOARD_USE_ENCODER = true; // Set false to force sensorless (open-loop) mode

// Reserved / not available pins (documented for clarity)
// PB2 → GND, PA11/PA12 → GND, RESET/SLEEP/FAULT on DRV8313 not connected

// Basic board/peripheral init helpers (implemented by Board & HAL agent)
void init_debug_led();
void init_uart_dma(unsigned long baud);
void read_strap_pins(bool &strap0_low, bool &strap1_low);
