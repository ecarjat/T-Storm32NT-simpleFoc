# Status LED Behavior (PA1)

This spec defines the **standard** LED behavior for the firmware. It replaces
the old PACKET_DEBUG-only LED logic and makes the pulse logic always active.

## States

- **Booting:** LED blinks continuously while initialization is in progress.
  - Blink is non-blocking (timer-based).
  - Starts at power-up/reset and ends once the system is marked running.
- **Running:** LED is **solid ON** as soon as the system is running.
- **Issue pulse:** Any detected issue triggers a brief LED pulse (toggle for
  ~50–100 ms) without blocking. Pulse overlays the current state.
- **Critical init failure:** if initialization fails, the LED stays in
    continuous pulse mode (do not switch to steady ON).

## Issue Conditions (Pulse Triggers)

- **BinaryIO packet errors:**
  - Unexpected packet type.
  - Bad length / truncated frame.
  - `_io->available()` reports data, but a full packet cannot be assembled.
- **Control loop overrun:**
  - Timer ISR detects that a previous loop is still running.
- **Critical init errors (optional):**
  - Sensor init failure.
  - Calibration failure.

## Implementation Notes

- Reuse the existing packet debug LED helpers (non-blocking pulse logic) but
  **do not** gate them behind `PACKET_DEBUG`.
- Boot blinking should stop as soon as `system_running == true`.
- Use a single LED update function (called from `loop()` or a periodic tick)
  that:
  1) Applies pulse overlay if active.
  2) Otherwise shows boot blink or steady ON, depending on state.

## Timing Guidelines

- Boot blink: ~2 Hz (toggle every ~250 ms).
- Pulse duration: 50–100 ms.
