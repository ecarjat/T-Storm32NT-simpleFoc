# STM32F103 Memory Management

## Total Memory - STM32F103T8

| Memory Type | Size |
|-------------|------|
| **Flash** | 64 KB |
| **RAM** | 20 KB |

---

## Flash Memory Layout (64 KB)

```
┌─────────────────────────────────────┐ 0x08000000
│    BOOTLOADER (6 KB)                │
│    0x08000000 - 0x080017FF          │
├─────────────────────────────────────┤ 0x08001800
│                                     │
│    APPLICATION FIRMWARE (55 KB)     │
│    0x08001800 - 0x0800F3FF          │
│                                     │
│    - Vector table (VECT_TAB_OFFSET) │
│    - Code (.text)                   │
│    - Read-only data (.rodata)       │
│    - Initialized data (.data)       │
│                                     │
├─────────────────────────────────────┤ 0x0800F400
│    PERSISTED SETTINGS (3 KB)        │
│    0x0800F400 - 0x0800FFFF          │
│    - EEPROM emulation               │
│    - CRC32 protected                │
└─────────────────────────────────────┘ 0x08010000
```

---

## Configuration - Single Source of Truth

The application start address is defined in `platformio.ini`:

```ini
-D APP_START_ADDR=0x08001800
```

This value is used by:
- **Bootloader** (`bootloader/src/flash.h`) - derives flash addresses from `APP_START_ADDR`
- **Linker script** (`ldscripts/app_offset.ld`) - sets `FLASH ORIGIN` for application
- **Application** (`platformio.ini`) - sets `VECT_TAB_OFFSET=0x1800` for vector table relocation

To change the memory layout, update these locations:
1. `APP_START_ADDR` in platformio.ini (bootloader environments)
2. `VECT_TAB_OFFSET` in platformio.ini (app environments)
3. `FLASH ORIGIN` and `LENGTH` in `ldscripts/app_offset.ld`

---

## Settings Storage

**Persisted settings are in a SEPARATE location** - they are NOT part of the firmware binary.

This is enforced in two ways:

1. **Linker script** (`ldscripts/app_offset.ld`) explicitly limits flash to `0x0000DC00` (55 KB), stopping before `0x0800F400`

2. **Settings code** (`src/motor_config.h`) defines:
   ```cpp
   constexpr uint32_t SETTINGS_ADDR = 0x0800F400UL;
   ```

This design allows:
- Firmware updates without erasing calibration/settings
- Settings to persist across reflashes
- Independent wear-leveling of the settings sector

---

## Settings Structure

From `src/runtime_settings.h`:

```cpp
struct PersistedSettings {
    uint32_t magic;        // 0x53544631 ("STF1")
    uint32_t version;      // Currently 7
    RuntimeSettings data;  // Motor params, PID, calibration LUT
    uint32_t crc;          // CRC32 integrity check
};

struct SensorCalibrationData {
    bool valid;
    uint16_t lut_size;              // 1024 entries
    float zero_electric_angle;
    int32_t direction;
    int16_t lut_counts[1024];       // Offset in counts (int16_t)
};
```

The largest component is `SensorCalibrationData` with a 1024-entry int16_t LUT (~2048 bytes). A compile-time assertion ensures the total fits in 3 KB.

---

## RAM Layout (20 KB)

```
┌─────────────────────────────────────┐ 0x20000000
│    .data (initialized variables)    │
│    .bss  (zero-initialized)         │
├─────────────────────────────────────┤
│    Heap (min 512 bytes)             │
├─────────────────────────────────────┤
│              ↓ grows down           │
│    Stack (min 1 KB)                 │
└─────────────────────────────────────┘ 0x20005000 (_estack)
```

---

## Summary Table

| Region | Start | End | Size | Part of Firmware? |
|--------|-------|-----|------|-------------------|
| Bootloader | `0x08000000` | `0x080017FF` | 6 KB | No (separate binary) |
| Application | `0x08001800` | `0x0800F3FF` | 55 KB | Yes |
| Settings | `0x0800F400` | `0x0800FFFF` | 3 KB | **No (separate sector)** |
