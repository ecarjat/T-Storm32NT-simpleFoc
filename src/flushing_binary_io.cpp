#include "flushing_binary_io.h"

BinaryIO &FlushingBinaryIO::operator<<(Packet value) {
  // Use base framing, then flush immediately on END_PACKET (type 0).
  BinaryIO::operator<<(value);
  if (value.type == 0x00) {
    _flush();
  }
  return *this;
}
