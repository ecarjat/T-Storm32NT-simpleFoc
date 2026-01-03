# Packet Debug Behavior (STM32 BinaryIO)

Packet parsing errors trigger LED pulses as part of the **standard** LED behavior
(see `StatusLed.md`). This is no longer gated behind `PACKET_DEBUG`.

## LED behavior
- **Boot blink:** LED blinks during boot, then turns solid ON when running.
- **Packet error pulse:** on any packet error (below), briefly pulse the LED
  (non-blocking) without changing the steady running state.

## Packet error conditions to pulse
- Unexpected packet type encountered in the BinaryIO / PacketCommander parser.
- Bad length or truncated frame: `_io->available()` indicates data, but `_io->read()` / frame assembly fails to complete a full packet (size mismatch or premature end).
- Optional: treat other parse failures (e.g., unknown register ID) as errors as needed.

## Touch points / implementation hints
- Wrap `PacketCommander::run()` or `BootPacketCommander::handlePacket` so you can detect “handled” vs. “ignored/invalid” frames and call a pulse helper on invalid cases.
- In the BinaryIO framing layer (if reachable), when bytes are present but no frame can be assembled, also pulse.
- Centralize LED control in shared helpers (non-blocking, use `millis()`/`micros()`, no `delay()`).

## Optional counters
- Track `debug_bad_packets` / `debug_rx_trunc` counters for errors.
- If useful, expose these counters via spare register IDs.

## Notes
- BinaryIO size field is 1 byte (payload+type), so max payload is 254 bytes; keep telemetry/register payloads well below this.
- TX and RX buffers are independent; telemetry flowing does not imply register writes were parsed. This mode gives a visual hint when the parser rejects frames.
