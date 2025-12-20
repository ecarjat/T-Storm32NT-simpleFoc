# Packet Debug Mode (STM32 BinaryIO)

Build-time diagnostic mode to help spot packet parsing issues on the STM32 BinaryIO path.

## Enable
- Define `PACKET_DEBUG` in the firmware build (e.g., add `-DPACKET_DEBUG` to `platformio.ini` build_flags or gate in `comms_streams.h`).
- When `PACKET_DEBUG` is **not** defined, all behavior below is compiled out and normal LED behavior is unchanged.

## LED behavior (PA1 / /LED)
- **Telemetry inactive (default):** keep existing idle/blink behavior.
- **Telemetry active:** after the first telemetry frame is sent (`telemetry.run()` emits a frame), stop idle blinking and hold LED **ON** steadily to indicate streaming.
- **Packet error pulse:** on any packet error (below), briefly pulse the LED (e.g., 50–100 ms toggle) without blocking. Use timestamp-based, non-blocking logic so the main loop is not delayed. Pulses should not permanently change the steady-on state during telemetry.

## Packet error conditions to pulse
- Unexpected packet type encountered in the BinaryIO / PacketCommander parser.
- Bad length or truncated frame: `_io->available()` indicates data, but `_io->read()` / frame assembly fails to complete a full packet (size mismatch or premature end).
- Optional: treat other parse failures (e.g., unknown register ID) as errors as needed.

## Touch points / implementation hints
- Wrap `PacketCommander::run()` or `BootPacketCommander::handlePacket` so you can detect “handled” vs. “ignored/invalid” frames and call `debug_led_pulse()` on invalid cases.
- In the BinaryIO framing layer (if reachable), when bytes are present but no frame can be assembled, also pulse.
- In `handle_streams()`, after `telemetry.run()` successfully sends a frame, set a `telemetry_active` flag so the LED is held ON while streaming.
- Centralize LED control under `#ifdef PACKET_DEBUG` helpers (e.g., `debug_led_set(bool)`, `debug_led_pulse()`), and keep timing non-blocking (use `millis()`/`micros()`, no `delay()`).

## Optional counters
- Track `debug_bad_packets` / `debug_rx_trunc` counters for errors.
- If useful, expose these counters via spare register IDs when `PACKET_DEBUG` is enabled.

## Notes
- BinaryIO size field is 1 byte (payload+type), so max payload is 254 bytes; keep telemetry/register payloads well below this.
- TX and RX buffers are independent; telemetry flowing does not imply register writes were parsed. This mode gives a visual hint when the parser rejects frames.
