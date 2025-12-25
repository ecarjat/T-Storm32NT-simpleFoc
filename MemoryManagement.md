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
│    BOOTLOADER (8 KB)                │
│    0x08000000 - 0x08001FFF          │
├─────────────────────────────────────┤ 0x08002000
│                                     │
│    APPLICATION FIRMWARE (54 KB)     │
│    0x08002000 - 0x0800F7FF          │
│                                     │
│    - Vector table (VECT_TAB_OFFSET) │
│    - Code (.text)                   │
│    - Read-only data (.rodata)       │
│    - Initialized data (.data)       │
│                                     │
├─────────────────────────────────────┤ 0x0800F800
│    PERSISTED SETTINGS (2 KB)        │
│    0x0800F800 - 0x0800FFFF          │
│    - EEPROM emulation               │
│    - CRC32 protected                │
└─────────────────────────────────────┘ 0x08010000
```

---

## Settings Storage

**Persisted settings are in a SEPARATE location** - they are NOT part of the firmware binary.

This is enforced in two ways:

1. **Linker script** (`ldscripts/app_offset.ld`) explicitly limits flash to `0x0000D800` (54 KB), stopping before `0x0800F800`

2. **Settings code** (`src/motor_config.h`) defines:
   ```cpp
   constexpr uint32_t SETTINGS_ADDR = 0x0800F800UL;
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
    uint32_t version;      // Currently 6
    RuntimeSettings data;  // Motor params, PID, calibration LUT
    uint32_t crc;          // CRC32 integrity check
};
```

The largest component is `SensorCalibrationData` with a 220-entry float LUT (~880 bytes). A compile-time assertion ensures the total fits in 2 KB.

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
| Bootloader | `0x08000000` | `0x08001FFF` | 8 KB | No (separate binary) |
| Application | `0x08002000` | `0x0800F7FF` | 54 KB | Yes |
| Settings | `0x0800F800` | `0x0800FFFF` | 2 KB | **No (separate sector)** |
