#pragma once

// Minimal register definitions from the TLx5012B vendor driver.
// This keeps the original full-duplex HAL sensor self-contained.

enum Tle5012bReg : uint16_t {
  REG_STAT = 0x0000,
  REG_ACSTAT = 0x0010,
  REG_AVAL = 0x0020,
  REG_ASPD = 0x0030,
  REG_AREV = 0x0040,
  REG_FSYNC = 0x0050,
  REG_MOD_1 = 0x0060,
  REG_SIL = 0x0070,
  REG_MOD_2 = 0x0080,
  REG_MOD_3 = 0x0090,
  REG_OFFX = 0x00A0,
  REG_OFFY = 0x00B0,
  REG_SYNCH = 0x00C0,
  REG_IFAB = 0x00D0,
  REG_MOD_4 = 0x00E0,
  REG_TCO_Y = 0x00F0,
  REG_ADC_RAW = 0x0100,
  REG_REVOL = 0x0110,
  REG_ADC_X = 0x0120,
  REG_ADC_Y = 0x0130,
  REG_D_MAG = 0x0140
};
