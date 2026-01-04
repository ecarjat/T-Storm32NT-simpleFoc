#pragma once

#include <Stream.h>
#include <comms/RegisterIO.h>

/*
 * Robust binary I/O with byte stuffing and CRC-32.
 *
 * Drop-in replacement for BinaryIO with improved framing:
 * - SLIP-style byte stuffing prevents false sync
 * - CRC-32 detects corrupted packets (uses STM32 hardware CRC)
 *
 * Frame format:
 *   [0xA5][LEN][TYPE][PAYLOAD...][CRC32]
 *
 * Byte stuffing:
 *   0xA5 -> 0xDB 0xDC
 *   0xDB -> 0xDB 0xDD
 *
 * CRC-32 polynomial: 0x04C11DB7 (STM32 hardware compatible)
 */

#define ROBUST_FRAME_MARKER      0xA5
#define ROBUST_FRAME_ESC         0xDB
#define ROBUST_FRAME_ESC_MARKER  0xDC
#define ROBUST_FRAME_ESC_ESC     0xDD
#define ROBUST_FRAME_CRC_SIZE    4

#define ROBUST_TX_BUFFER_SIZE    72
#define ROBUST_RX_BUFFER_SIZE    128

class RobustBinaryIO : public PacketIO {
public:
    explicit RobustBinaryIO(Stream& io);
    virtual ~RobustBinaryIO() = default;

    // PacketIO interface
    RobustBinaryIO& operator<<(float value) override;
    RobustBinaryIO& operator<<(uint32_t value) override;
    RobustBinaryIO& operator<<(uint8_t value) override;
    RobustBinaryIO& operator<<(char value) override;
    RobustBinaryIO& operator<<(Packet value) override;
    RobustBinaryIO& operator<<(Separator value) override;

    RobustBinaryIO& operator>>(float& value) override;
    RobustBinaryIO& operator>>(uint32_t& value) override;
    RobustBinaryIO& operator>>(uint8_t& value) override;
    PacketIO& operator>>(Packet& value) override;

    bool is_complete() override;

    // Statistics
    uint32_t sync_losses() const { return _sync_losses; }
    uint32_t crc_errors() const { return _crc_errors; }

protected:
    void _flush();

private:
    // CRC-32 (polynomial 0x04C11DB7, STM32 hardware compatible)
    static uint32_t crc32(const uint8_t* data, size_t len);
    static uint32_t crc32_hw(const uint8_t* data, size_t len);  // Hardware accelerated

    // TX helpers
    void tx_byte_escaped(uint8_t byte);
    void tx_flush_buffer();

    // RX helpers
    void rx_fill();
    int rx_peek();
    int rx_read();
    uint8_t rx_available() const;

    Stream& _io;

    // TX state
    uint8_t _tx_buffer[ROBUST_TX_BUFFER_SIZE];
    uint8_t _tx_pos = 0;
    uint8_t _tx_payload[64];  // Buffer for CRC calculation
    uint8_t _tx_payload_len = 0;
    bool _tx_in_packet = false;

    // RX state
    uint8_t _rx_buffer[ROBUST_RX_BUFFER_SIZE];
    uint8_t _rx_head = 0;
    uint8_t _rx_tail = 0;

    // Parser state
    enum class ParseState {
        IDLE,
        LEN,
        DATA,
    };
    ParseState _parse_state = ParseState::IDLE;
    uint8_t _parse_buf[ROBUST_RX_BUFFER_SIZE];
    uint8_t _parse_expected_len = 0;
    uint8_t _parse_pos = 0;
    bool _parse_esc_pending = false;
    uint8_t _remaining = 0;  // Bytes remaining in current received packet

    // Pending packet (waiting for full payload)
    bool _pending = false;
    uint8_t _pending_type = 0;
    uint8_t _pending_payload = 0;

    // Statistics
    uint32_t _sync_losses = 0;
    uint32_t _crc_errors = 0;
};

/*
 * Flushing variant - flushes on END_PACKET.
 */
class FlushingRobustBinaryIO : public RobustBinaryIO {
public:
    explicit FlushingRobustBinaryIO(Stream& io) : RobustBinaryIO(io) {}

    RobustBinaryIO& operator<<(Packet value) override {
        RobustBinaryIO::operator<<(value);
        if (value.type == 0x00) {
            _flush();
        }
        return *this;
    }
};
