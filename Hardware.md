Board: T-STorM32 NT Motor Module v2.51E
MCU: STM32F103T8 (Cortex-M3, 3.3 V, 8 MHz ceramic resonator)
Motor driver: DRV8313PWPR (3-phase BLDC driver, VM = Vbat)
Encoder: TLE5012B (SPI, 3-wire DATA line)
Supplies:
- Vbat: motor supply, goes only to DRV8313 (VDD pins) + bulk caps
- 5V: input to LDO IC2 (TC1015-3.3V), not used directly by MCU
- 3.3V: output of IC2, powers MCU, encoder, LEDs, SWD, etc.

Clock & reset:
- External resonator Q1 (8 MHz) between PD0/OSC_IN and PD1/OSC_OUT + caps
- NRST: RC reset (C6), no external reset button
- BOOT0: strapped with R1 and solder jumper SJ1 (boot mode config)

Debug:
- SWD:
  - PA13 → SWDIO → JP3
  - PA14 → SWCLK → JP3
  - 3.3V and GND also on JP3

UART (NT bus logic side):
- PA9  → /TX  (UART1_TX)
- PA10 → /RX  (UART1_RX)
These nets go to the NT bus front-end / connectors. Logic is 3.3 V.

Motor driver interface (DRV8313):
- Phase control inputs:
  - PA3 → /PHASE1 → DRV8313 IN1 (Phase U control)
  - PB0 → /PHASE2 → DRV8313 IN2 (Phase V control)
  - PB1 → /PHASE3 → DRV8313 IN3 (Phase W control)

- Enable / reset / fault:
  - EN1, EN2, EN3 → all tied to /LDO3.3V (DRV8313 internal LDO output)
  - RESET and SLEEP pins are tied together and pulled up to /LDO3.3V via R4 with C10 to GND (power-on reset/enable). They are NOT connected to the MCU.
  - FAULT pin is not connected (no MCU fault feedback).

- Motor outputs:
  - OUT1 → motor connector JP4 pin 1 (phase U)
  - OUT2 → JP4 pin 2 (phase V)
  - OUT3 → JP4 pin 3 (phase W)

- Motor supply:
  - VDD/VDD@2 → Vbat (motor supply)
  - PGND pins → GND

Encoder interface (TLE5012B, 3-wire SPI on SPI1):
- PA5 → /SCK  → TLE5012B SCK
- PA6 → /MISO → TLE5012B DATA (bidirectional data line)
- PA7 → /MOSI → via R5 (≈1 kΩ) to the same DATA line
- PA8 → /CS   → TLE5012B CSQ
- VDD → 3.3V, GND → GND
- IF_A / IF_B / IF_C unconnected (mode is fixed by defaults; encoder used as SPI slave with 3-wire DATA)

Firmware note: configure SPI1 as normal full-duplex (PA5/PA6/PA7). Hardware bridges MOSI to DATA via R5, so commands go out on MOSI and responses come back on MISO over the same DATA line.

Status LEDs and jumpers:
- PA1 → /LED → R3 → LED2 → GND
  - Status LED, active high (drive PA1 high to turn LED2 on).
- LED1: 3.3V → R2 → LED1 → GND (power/“on” indicator, not MCU-controlled)

- PA2 → Net-(IC1C-PA2) → SJ2 → GND (optional strap)
  - If SJ2 is soldered, PA2 is tied to GND. Otherwise floating. Can be read at boot as an ID/strap bit.
- PA4 → Net-(IC1C-PA4) → SJ3 → GND (optional strap)
  - If SJ3 is soldered, PA4 is tied to GND. Otherwise floating. Can be read at boot as an ID/strap bit.

Power rails (MCU side):
- All VDDx and VDDA pins of MCU → 3.3V, decoupled with multiple capacitors
- All VSSx and VSSA pins → GND
- PB2 pin is tied directly to GND (not usable as GPIO)

Connectors (high-level):
- JP3: SWD header (SWDIO, SWCLK, 3.3V, GND)
- NT bus connectors (not fully detailed here): carry 5V (or module supply), TX, RX, GND, and connect to PA9/PA10 via front-end.
- JP4: 3-pin motor connector: OUT1, OUT2, OUT3

MCU pin usage summary (for firmware):
- PA0: not connected (can be used as free GPIO)
- PA1: LED (status output, active high)
- PA2: optional strap via SJ2 (input with pull-up recommended)
- PA3: PHASE1 → DRV8313 IN1 (PWM/commutation)
- PA4: optional strap via SJ3 (input with pull-up recommended)
- PA5: SPI1_SCK → encoder SCK
- PA6: SPI1_MISO → encoder DATA
- PA7: SPI1_MOSI → encoder DATA via R5
- PA8: encoder CS (chip select)
- PA9: UART1_TX → NT bus
- PA10: UART1_RX → NT bus
- PA11, PA12: tied to GND (not usable)
- PA13: SWDIO
- PA14: SWCLK
- PA15: unconnected (free GPIO if needed)

- PB0: PHASE2 → DRV8313 IN2 (PWM/commutation)
- PB1: PHASE3 → DRV8313 IN3 (PWM/commutation)
- PB2: tied to GND (not usable)
- PB3, PB4, PB5, PB6, PB7: unconnected (free GPIOs if broken out; not used on this board)

- PD0, PD1: oscillator pins with 8 MHz resonator

Assumptions for firmware:
- Use SPI1 on PA5/PA6/PA7 to talk to TLE5012B (3-wire DATA).
- Use UART1 on PA9/PA10 for NT bus (baud as required, e.g. up to 2 Mbaud).
- Use timers for PA3, PB0, PB1 as PWM outputs to drive DRV8313 phase inputs.
- Initialize PA1 as push-pull output (status LED).
- Initialize PA2 and PA4 as inputs with pull-ups and read them at boot as configuration straps.
