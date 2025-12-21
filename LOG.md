# BinaryIO Log Spec (UART DMA)

This document defines how to send debug logs over the same UART used by BinaryIO
without corrupting packet framing.

## Goals

- Debug output shares USART1 with BinaryIO without breaking telemetry/commands.
- Logs are carried as BinaryIO packets (no raw text on the wire).
- Host tools can optionally display or ignore log packets.

## Packet Type

- Packet type: `L` (0x4C)
- Direction: MCU -> Host only
- Transport: BinaryIO framing
  - `[0xA5][size][type][payload...]`
  - `size = payload_len + 1` (type byte included, per BinaryIO)

## Payload Format

```
byte 0: level (uint8)
byte 1: tag_len (uint8)
byte 2..(2+tag_len-1): tag bytes (ASCII, no null)
byte N: msg_len (uint8)
byte N+1..(N+msg_len): msg bytes (ASCII, no null)
```

Levels (uint8):
- 0 = DEBUG
- 1 = INFO
- 2 = WARN
- 3 = ERROR

Notes:
- `tag_len` and `msg_len` must be <= 60 to keep packets short.
- If the message exceeds limits, it must be truncated.

## Sending Rules

- Logs must be sent using BinaryIO framing only.
- If the TX ring is full, drop the log and increment a drop counter.
- Logs must never block the FOC loop.
- When `DEBUG_SERIAL` is not defined, no logs are emitted.

## Host Behavior

- Host should ignore unknown packet types by default.
- If `L` packets are enabled, host renders:
  - `LEVEL/TAG: message`
- Logs are not part of telemetry registers.

## Host Implementation (Python)

Add a handler in the BinaryIO packet reader in `cli/pysfoc` that recognizes
type `L` and decodes the payload into printable text.

Example decoder (pseudo-code):

```python
def handle_packet(packet_type: int, payload: bytes):
    if packet_type != ord("L"):
        return False
    if len(payload) < 2:
        return True
    level = payload[0]
    tag_len = payload[1]
    idx = 2
    if idx + tag_len > len(payload):
        return True
    tag = payload[idx:idx+tag_len].decode("ascii", errors="replace")
    idx += tag_len
    if idx >= len(payload):
        msg_len = 0
        msg = ""
    else:
        msg_len = payload[idx]
        idx += 1
        msg = payload[idx:idx+msg_len].decode("ascii", errors="replace")
    print(f"{level_name(level)}/{tag}: {msg}")
    return True
```

Level mapping:

```
0 -> DEBUG
1 -> INFO
2 -> WARN
3 -> ERROR
```

Notes:
- If the payload is malformed or truncated, ignore the packet (do not resync).
- Host should keep parsing subsequent packets as usual.

## Implementation Notes

- Add a lightweight helper:
  - `void log_packet(uint8_t level, const char* tag, const char* msg);`
- Use the existing DMA-backed BinaryIO stream for writes.
- Do not use `Serial.print()` for logs on this UART.
