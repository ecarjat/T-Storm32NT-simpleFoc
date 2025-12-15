#pragma once

#include <comms/streams/BinaryIO.h>

// BinaryIO subclass that flushes immediately when END_PACKET (type 0) is sent,
// ensuring each packet is written out without waiting for the internal buffer to fill.
class FlushingBinaryIO : public BinaryIO {
public:
  using BinaryIO::BinaryIO;
  BinaryIO &operator<<(Packet value) override;
};
