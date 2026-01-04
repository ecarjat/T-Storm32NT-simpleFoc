#include "robust_binary_io.h"
#include <string.h>

extern "C" {
#include "stm32f1xx_hal.h"
}

/* Hardware CRC handle - must be initialized in main */
extern CRC_HandleTypeDef hcrc;

/*
 * CRC-32 using STM32F1 hardware CRC peripheral.
 * Polynomial: 0x04C11DB7
 */
uint32_t RobustBinaryIO::crc32_hw(const uint8_t* data, size_t len) {
    if (data == nullptr || len == 0) {
        return 0xFFFFFFFF;
    }

    __HAL_CRC_DR_RESET(&hcrc);

    /* Process full 32-bit words */
    size_t words = len / 4;
    const uint32_t* word_ptr = reinterpret_cast<const uint32_t*>(data);

    for (size_t i = 0; i < words; i++) {
        /* Reverse byte order for CRC calculation */
        uint32_t word = word_ptr[i];
        word = __REV(word);
        hcrc.Instance->DR = word;
    }

    /* Handle remaining bytes (F1 only supports 32-bit, pad with zeros) */
    size_t remaining = len % 4;
    if (remaining > 0) {
        uint32_t last_word = 0;
        const uint8_t* tail = data + (words * 4);
        for (size_t i = 0; i < remaining; i++) {
            last_word |= static_cast<uint32_t>(tail[i]) << (24 - (i * 8));
        }
        hcrc.Instance->DR = last_word;
    }

    return hcrc.Instance->DR;
}

/*
 * CRC-32 wrapper - uses hardware.
 */
uint32_t RobustBinaryIO::crc32(const uint8_t* data, size_t len) {
    return crc32_hw(data, len);
}

RobustBinaryIO::RobustBinaryIO(Stream& io) : _io(io) {
    memset(_tx_buffer, 0, sizeof(_tx_buffer));
    memset(_tx_payload, 0, sizeof(_tx_payload));
    memset(_rx_buffer, 0, sizeof(_rx_buffer));
    memset(_parse_buf, 0, sizeof(_parse_buf));
}

void RobustBinaryIO::tx_byte_escaped(uint8_t byte) {
    if (byte == ROBUST_FRAME_MARKER) {
        if (_tx_pos < sizeof(_tx_buffer)) _tx_buffer[_tx_pos++] = ROBUST_FRAME_ESC;
        if (_tx_pos < sizeof(_tx_buffer)) _tx_buffer[_tx_pos++] = ROBUST_FRAME_ESC_MARKER;
    } else if (byte == ROBUST_FRAME_ESC) {
        if (_tx_pos < sizeof(_tx_buffer)) _tx_buffer[_tx_pos++] = ROBUST_FRAME_ESC;
        if (_tx_pos < sizeof(_tx_buffer)) _tx_buffer[_tx_pos++] = ROBUST_FRAME_ESC_ESC;
    } else {
        if (_tx_pos < sizeof(_tx_buffer)) _tx_buffer[_tx_pos++] = byte;
    }
}

void RobustBinaryIO::tx_flush_buffer() {
    if (_tx_pos > 0) {
        _io.write(_tx_buffer, _tx_pos);
        _tx_pos = 0;
    }
}

void RobustBinaryIO::_flush() {
    tx_flush_buffer();
}

RobustBinaryIO& RobustBinaryIO::operator<<(Packet value) {
    if (value.type != 0x00) {
        // START_PACKET - begin new frame
        _tx_pos = 0;
        _tx_payload_len = 0;
        _tx_buffer[_tx_pos++] = ROBUST_FRAME_MARKER;

        // Length: TYPE(1) + PAYLOAD(N) + CRC32(4)
        uint8_t len = 1 + value.payload_size + ROBUST_FRAME_CRC_SIZE;
        tx_byte_escaped(len);

        // Store LEN for CRC calculation
        _tx_payload[_tx_payload_len++] = len;

        // Type
        tx_byte_escaped(value.type);
        _tx_payload[_tx_payload_len++] = value.type;

        _tx_in_packet = true;
    } else {
        // END_PACKET - finalize with CRC32
        if (_tx_in_packet) {
            // Calculate CRC over LEN + TYPE + PAYLOAD
            uint32_t crc = crc32(_tx_payload, _tx_payload_len);

            // Escape and send CRC32 (4 bytes, little-endian)
            for (int i = 0; i < 4; i++) {
                tx_byte_escaped(static_cast<uint8_t>(crc >> (i * 8)));
            }
            _tx_in_packet = false;
        }
    }
    return *this;
}

RobustBinaryIO& RobustBinaryIO::operator<<(float value) {
    uint8_t* bytes = reinterpret_cast<uint8_t*>(&value);
    for (int i = 0; i < 4; i++) {
        tx_byte_escaped(bytes[i]);
        if (_tx_payload_len < sizeof(_tx_payload)) {
            _tx_payload[_tx_payload_len++] = bytes[i];
        }
    }
    return *this;
}

RobustBinaryIO& RobustBinaryIO::operator<<(uint32_t value) {
    for (int i = 0; i < 4; i++) {
        uint8_t byte = (value >> (i * 8)) & 0xFF;
        tx_byte_escaped(byte);
        if (_tx_payload_len < sizeof(_tx_payload)) {
            _tx_payload[_tx_payload_len++] = byte;
        }
    }
    return *this;
}

RobustBinaryIO& RobustBinaryIO::operator<<(uint8_t value) {
    tx_byte_escaped(value);
    if (_tx_payload_len < sizeof(_tx_payload)) {
        _tx_payload[_tx_payload_len++] = value;
    }
    return *this;
}

RobustBinaryIO& RobustBinaryIO::operator<<(char value) {
    return operator<<(static_cast<uint8_t>(value));
}

RobustBinaryIO& RobustBinaryIO::operator<<(Separator value) {
    // Separators are ignored in binary protocol
    (void)value;
    return *this;
}

void RobustBinaryIO::rx_fill() {
    while (_io.available() > 0) {
        uint8_t next_head = (_rx_head + 1) % sizeof(_rx_buffer);
        if (next_head == _rx_tail) {
            break;  // Buffer full
        }
        int byte = _io.read();
        if (byte < 0) break;
        _rx_buffer[_rx_head] = static_cast<uint8_t>(byte);
        _rx_head = next_head;
    }
}

int RobustBinaryIO::rx_peek() {
    if (_rx_head == _rx_tail) return -1;
    return _rx_buffer[_rx_tail];
}

int RobustBinaryIO::rx_read() {
    if (_rx_head == _rx_tail) return -1;
    uint8_t byte = _rx_buffer[_rx_tail];
    _rx_tail = (_rx_tail + 1) % sizeof(_rx_buffer);
    return byte;
}

uint8_t RobustBinaryIO::rx_available() const {
    if (_rx_head >= _rx_tail) {
        return _rx_head - _rx_tail;
    }
    return sizeof(_rx_buffer) - _rx_tail + _rx_head;
}

PacketIO& RobustBinaryIO::operator>>(Packet& value) {
    rx_fill();

    // Return pending packet if we have one and enough data
    if (_pending) {
        if (rx_available() >= _pending_payload) {
            value.type = _pending_type;
            value.payload_size = _pending_payload;
            _remaining = value.payload_size;
            _pending = false;
            return *this;
        }
        value.type = 0x00;
        value.payload_size = 0;
        return *this;
    }

    // Parse incoming bytes
    while (rx_available() > 0) {
        int raw = rx_read();
        if (raw < 0) break;
        uint8_t byte = static_cast<uint8_t>(raw);

        // Handle marker - always starts new frame (unless escaped)
        if (byte == ROBUST_FRAME_MARKER && !_parse_esc_pending) {
            if (_parse_state != ParseState::IDLE && _parse_state != ParseState::LEN) {
                _sync_losses++;
            }
            _parse_state = ParseState::LEN;
            _parse_pos = 0;
            _parse_esc_pending = false;
            continue;
        }

        // Handle escape sequences
        if (_parse_esc_pending) {
            _parse_esc_pending = false;
            if (byte == ROBUST_FRAME_ESC_MARKER) {
                byte = ROBUST_FRAME_MARKER;
            } else if (byte == ROBUST_FRAME_ESC_ESC) {
                byte = ROBUST_FRAME_ESC;
            } else {
                // Invalid escape - sync loss
                _sync_losses++;
                _parse_state = ParseState::IDLE;
                continue;
            }
        } else if (byte == ROBUST_FRAME_ESC) {
            _parse_esc_pending = true;
            continue;
        }

        // State machine
        switch (_parse_state) {
        case ParseState::IDLE:
            break;

        case ParseState::LEN:
            _parse_expected_len = byte;
            // Minimum: TYPE(1) + CRC32(4) = 5
            if (_parse_expected_len < (1 + ROBUST_FRAME_CRC_SIZE) ||
                _parse_expected_len > sizeof(_parse_buf)) {
                _sync_losses++;
                _parse_state = ParseState::IDLE;
            } else {
                _parse_buf[0] = byte;
                _parse_pos = 1;
                _parse_state = ParseState::DATA;
            }
            break;

        case ParseState::DATA:
            if (_parse_pos < sizeof(_parse_buf)) {
                _parse_buf[_parse_pos++] = byte;
            }
            // Complete when we have LEN + (TYPE + PAYLOAD + CRC32) = _parse_expected_len + 1
            if (_parse_pos >= static_cast<size_t>(_parse_expected_len + 1)) {
                _parse_state = ParseState::IDLE;

                // Verify CRC32: computed over buf[0..pos-5], received in buf[pos-4..pos-1]
                uint32_t computed_crc = crc32(_parse_buf, _parse_pos - ROBUST_FRAME_CRC_SIZE);
                uint32_t received_crc = 0;
                for (int i = 0; i < 4; i++) {
                    received_crc |= static_cast<uint32_t>(
                        _parse_buf[_parse_pos - ROBUST_FRAME_CRC_SIZE + i]) << (i * 8);
                }

                if (computed_crc != received_crc) {
                    _crc_errors++;
                    _parse_pos = 0;
                    continue;
                }

                // Valid frame!
                value.type = _parse_buf[1];  // TYPE is after LEN
                // Payload length = LEN - TYPE(1) - CRC32(4)
                uint8_t payload_len = (_parse_expected_len > (1 + ROBUST_FRAME_CRC_SIZE))
                    ? (_parse_expected_len - 1 - ROBUST_FRAME_CRC_SIZE) : 0;
                value.payload_size = payload_len;
                _remaining = payload_len;

                // Copy payload to beginning of buffer for reading
                if (payload_len > 0) {
                    memmove(_parse_buf, &_parse_buf[2], payload_len);
                }
                _parse_pos = 0;
                return *this;
            }
            break;
        }
    }

    value.type = 0x00;
    value.payload_size = 0;
    return *this;
}

RobustBinaryIO& RobustBinaryIO::operator>>(float& value) {
    if (_remaining >= 4) {
        memcpy(&value, &_parse_buf[_parse_pos], 4);
        _parse_pos += 4;
        _remaining -= 4;
    } else {
        value = 0.0f;
    }
    return *this;
}

RobustBinaryIO& RobustBinaryIO::operator>>(uint32_t& value) {
    if (_remaining >= 4) {
        value = 0;
        for (int i = 0; i < 4; i++) {
            value |= static_cast<uint32_t>(_parse_buf[_parse_pos + i]) << (i * 8);
        }
        _parse_pos += 4;
        _remaining -= 4;
    } else {
        value = 0;
    }
    return *this;
}

RobustBinaryIO& RobustBinaryIO::operator>>(uint8_t& value) {
    if (_remaining >= 1) {
        value = _parse_buf[_parse_pos++];
        _remaining--;
    } else {
        value = 0;
    }
    return *this;
}

bool RobustBinaryIO::is_complete() {
    return _remaining == 0;
}
