#pragma once

#include <stdint.h>

#include <comms/RegisterIO.h>

enum LogLevel : uint8_t {
  LOG_DEBUG = 0,
  LOG_INFO = 1,
  LOG_WARN = 2,
  LOG_ERROR = 3,
};

// Initialize the log stream to use BinaryIO framing on the shared UART.
void init_log_stream(PacketIO &io);

// Send a BinaryIO log packet (type 'L') over the shared UART.
void log_packet(uint8_t level, const char *tag, const char *msg);
