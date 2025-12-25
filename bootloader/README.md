# SimpleFOC Driver Bootloader

Minimal UART bootloader for STM32F103 that enables firmware updates without a programmer.

## Memory Layout

| Region | Address Range | Size |
|--------|---------------|------|
| Bootloader | 0x08000000 - 0x080017FF | 6 KB |
| Application | 0x08001800 - 0x0800F3FF | 55 KB |
| Settings | 0x0800F400 - 0x0800FFFF | 3 KB |

The bootloader occupies the first 6KB of flash. The application starts at 0x08001800 with its vector table offset configured via `VECT_TAB_OFFSET=0x1800`.

## Features

- UART-based firmware upload (115200 baud, 8N1)
- CRC32 verification of uploaded firmware
- Automatic boot to application if valid
- Boot mode trigger via backup register (BKP->DR1)
- LED indicator on PA1 (solid = starting, blink = waiting for upload)

## Boot Process

1. On reset, bootloader initializes UART1 (PA9/PA10) and LED (PA1)
2. Checks if boot mode was requested via `BKP->DR1 == 0xB007`
3. If not in boot mode and application is valid, jumps to app
4. Otherwise, enters firmware upload mode

## Application Validity Check

The bootloader validates the application by checking:
- Stack pointer (first word) is within RAM: 0x20000000 - 0x20005000
- Reset vector (second word) is within application flash region

## Firmware Upload Protocol

### Header Format (12 bytes)
```
Offset  Size  Description
0       4     Magic: 0x30445055 ("UPD0", little-endian)
4       4     Firmware length in bytes
8       4     CRC32 of firmware data
```

### Upload Sequence

1. Host sends 12-byte header
2. Bootloader responds:
   - `ERLEN\n` - Invalid length
   - `ERASE\n` - Flash erase failed
   - `OK\n` - Ready to receive data
3. Host sends firmware in 256-byte chunks
4. For each chunk, bootloader responds:
   - `CHK RCV\n` - Chunk received
   - `ACK\n` - Chunk written successfully
   - `PFAIL 0xNN\n` - Flash write failed
   - `TIMEOUT\n` - Chunk receive timeout
5. After all data received:
   - `CRC\n` - CRC mismatch
   - `OK\n` - Success, jumping to app

### CRC32 Algorithm

Standard CRC32 with polynomial 0xEDB88320 (same as zlib/gzip):
```c
uint32_t crc32(const uint8_t *data, uint32_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (uint32_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      uint32_t mask = -(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320 & mask);
    }
  }
  return ~crc;
}
```

## Triggering Boot Mode from Application

To enter bootloader mode from the running application:

```c
#include "stm32f1xx_hal.h"

void enter_bootloader(void) {
  // Enable backup domain access
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_BKP_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  // Write boot magic
  WRITE_REG(BKP->DR1, 0xB007);

  // Reset to enter bootloader
  NVIC_SystemReset();
}
```

## Building

```bash
# Build bootloader
pio run -e tstorm32_bootloader

# Build with debug symbols
pio run -e tstorm32_bootloader_debug
```

## Flashing

The bootloader must be flashed via ST-Link or other SWD programmer:

```bash
pio run -e tstorm32_bootloader -t upload
```

## Hardware Configuration

| Pin | Function |
|-----|----------|
| PA9 | UART1 TX |
| PA10 | UART1 RX |
| PA1 | Status LED |

## Size Constraints

The bootloader must fit within 6KB (6144 bytes). Current size is approximately 4.9KB, leaving headroom for future enhancements.

## Configuration

The application start address is defined in a single location: `platformio.ini` via the `-D APP_START_ADDR=0x08001800` build flag. This value is used by:

- `bootloader/src/flash.h` - derives `FLASH_APP_START_ADDRESS` from `APP_START_ADDR`
- `ldscripts/app_offset.ld` - sets `FLASH ORIGIN` for the application
- Application build flags - sets `VECT_TAB_OFFSET` for vector table relocation

To change the memory layout, update `APP_START_ADDR` in platformio.ini and the corresponding values in the linker script.
