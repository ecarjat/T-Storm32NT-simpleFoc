#include "log_packet.h"

#include <Arduino.h>

namespace {
constexpr uint8_t LOG_MAX_FIELD_LEN = 60;

PacketIO *g_log_io = nullptr;

uint8_t bounded_len(const char *text, uint8_t max_len) {
  if (!text) {
    return 0;
  }
  uint8_t len = 0;
  while (len < max_len && text[len] != '\0') {
    len++;
  }
  return len;
}
}  // namespace

void init_log_stream(PacketIO &io) {
  g_log_io = &io;
}

void log_packet(uint8_t level, const char *tag, const char *msg) {
#ifdef DEBUG_SERIAL
  if (!g_log_io) {
    return;
  }
  const uint8_t tag_len = bounded_len(tag, LOG_MAX_FIELD_LEN);
  const uint8_t msg_len = bounded_len(msg, LOG_MAX_FIELD_LEN);
  const uint8_t payload_len = static_cast<uint8_t>(2 + tag_len + 1 + msg_len);

  *g_log_io << START_PACKET('L', payload_len) << level << tag_len;
  for (uint8_t i = 0; i < tag_len; i++) {
    *g_log_io << static_cast<uint8_t>(tag[i]);
  }
  *g_log_io << msg_len;
  for (uint8_t i = 0; i < msg_len; i++) {
    *g_log_io << static_cast<uint8_t>(msg[i]);
  }
  *g_log_io << END_PACKET;
#else
  (void)level;
  (void)tag;
  (void)msg;
#endif
}
