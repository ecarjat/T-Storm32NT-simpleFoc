# UART DMA Spec (STM32F103T8, HAL/LL)

This document defines how to move the UART RX/TX path to DMA using STM32 HAL/LL
and how to keep Arduino `Serial` usage strictly gated behind `DEBUG_SERIAL`.

## Goals

- Use DMA for USART1 RX/TX (PA9/PA10) to reduce CPU load and avoid RX overruns.
- Keep BinaryIO as the only runtime protocol (no text path).
- Ensure `Serial` is only referenced when `DEBUG_SERIAL` is defined.
- Provide a Stream-compatible wrapper for BinaryIO without using Arduino `Serial`.

## Non-goals

- No changes to motor control, sensors, or SimpleFOC logic.
- No protocol changes to BinaryIO packet format.
- No reliance on Arduino `HardwareSerial` for the comms path.

## Hardware + Peripheral Map

- MCU: STM32F103T8 (8MHz HSE, 72MHz SYSCLK)
- UART: USART1
  - TX: PA9 (AF push-pull)
  - RX: PA10 (input floating)
- DMA:
  - DMA1 Channel 4: USART1_TX
  - DMA1 Channel 5: USART1_RX

## Architecture Overview

1. **UartDmaStream** (new) provides `Stream`-compatible API backed by HAL/LL.
2. **BinaryIO** uses `UartDmaStream` instead of `Serial`.
3. **PacketCommander** remains unchanged (runs over BinaryIO).
4. **DEBUG_SERIAL** only enables Arduino `Serial` for debug logging.

## UartDmaStream Requirements

Implement a new wrapper in `src/uart_dma_stream.h/.cpp`:

- Must inherit from `Stream` and implement:
  - `int available()`
  - `int read()`
  - `int peek()`
  - `size_t write(uint8_t)`
  - `size_t write(const uint8_t*, size_t)`
  - `void flush()`
- Must be non-blocking for RX and TX.
- Must tolerate bursts from telemetry packets without data loss.

### RX (DMA circular)

- Allocate a circular RX buffer (e.g., 256 or 512 bytes).
- Start `HAL_UART_Receive_DMA(&huart1, rx_buf, RX_LEN)`.
- Disable half-transfer IRQ if not needed:
  - `__HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT)`.
- Track DMA position via `RX_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx)`.
- Maintain a software `rx_head` (DMA write index) and `rx_tail` (read index).
- `available()` returns `(rx_head - rx_tail) mod RX_LEN`.
- `read()` returns next byte or `-1` if empty.
- `peek()` returns next byte without moving `rx_tail`.

Enable USART IDLE line interrupt to refresh `rx_head` quickly.

### TX (DMA normal)

Option A (simple, non-blocking):
- Maintain a TX ring buffer (e.g., 256 bytes).
- `write()` appends to ring; if DMA idle, kick a DMA transfer of a contiguous block.
- DMA TX complete IRQ:
  - Advance `tx_tail`.
  - If more data pending, start next DMA transfer.
- If TX ring full, drop bytes and increment a debug counter (no blocking).


## HAL/LL Initialization

Use HAL or LL directly (no Arduino `Serial`) for comms:

- Enable clocks:
  - `__HAL_RCC_GPIOA_CLK_ENABLE()`
  - `__HAL_RCC_USART1_CLK_ENABLE()`
  - `__HAL_RCC_DMA1_CLK_ENABLE()`
- Configure GPIO:
  - PA9: `GPIO_MODE_AF_PP`, `GPIO_SPEED_FREQ_HIGH`
  - PA10: `GPIO_MODE_INPUT`, `GPIO_NOPULL`
- UART config (HAL):
  - Baud: 460800 (or runtime setting)
  - 8N1, no flow control
- DMA config:
  - RX: circular, byte alignment
  - TX: normal, byte alignment
- IRQs:
  - Enable `DMA1_Channel4_IRQn`, `DMA1_Channel5_IRQn`
  - Optionally `USART1_IRQn` for IDLE line

LL alternative:
- Use `LL_USART_Init`, `LL_DMA_Init`, and `LL_GPIO_Init`.
- Same channel mapping as above.

## BinaryIO Integration

Replace Arduino `Serial` use for comms with UartDmaStream:

- `UartDmaStream uart1_stream;`
- `BinaryIO comms(uart1_stream);`
- `PacketCommander packet_commander(comms, register_io);`

The comms path must not reference `Serial` at all.

## DEBUG_SERIAL Gating Requirements

All `Serial` references must be inside `#ifdef DEBUG_SERIAL` blocks.

Expected updates (examples):

- `src/main.cpp`: wrap `Serial.begin` and status prints.
- `src/comms_streams.cpp`: wrap calibration logging prints.
- `src/driver/tle5012b_full_duplex.cpp`: wrap error print.
- `src/board_pins.cpp`: do not call `Serial.setRx/Tx` or `Serial.begin` unless
  `DEBUG_SERIAL` is defined.

Enforcement guideline:
- `rg -n "\\bSerial\\b" src | rg -v "DEBUG_SERIAL"` should be empty.

## Packet Error Visibility (Optional)

If `PACKET_DEBUG` is enabled, use LED pulses for unexpected packet type,
bad length, or incomplete read (already defined in PacketDebug.md).
Do not use `Serial` for packet diagnostics unless `DEBUG_SERIAL` is on.

## Expected Benefits

- Reduced CPU time spent on UART ISR or polling.
- More consistent BinaryIO packet handling under high telemetry rates.
- Lower risk of RX overruns at 460800 baud.

## Implementation Checklist

1. Add `UartDmaStream` class with RX/TX ring buffers.
2. Add HAL/LL init in a new `init_uart_dma()` function.
3. Wire BinaryIO to use `UartDmaStream`.
4. Gate all `Serial` usage behind `DEBUG_SERIAL`.
5. Add counters for RX overrun, TX drop, and framing errors (optional).
6. Verify telemetry and register commands at 460800 baud.

## Test Plan

- Build without `DEBUG_SERIAL`:
  - Ensure no `Serial` symbols are referenced.
  - Verify telemetry works and no LED error pulses.
- Build with `DEBUG_SERIAL`:
  - Verify debug prints appear and comms still work.
- Stress test:
  - Request 4+ telemetry registers at downsample 1.
  - Confirm no packet errors and stable throughput.
