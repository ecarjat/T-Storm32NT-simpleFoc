# AGENTS.md

This file defines the AI agents and workflow for generating firmware to run
[SimpleFOC](https://github.com/simplefoc/Arduino-FOC) on the **T-STorM32 NT Motor Module v2.51E**
using **VSCode**, **SimpleFOC**, and **Arduino-FOC-drivers**.

The canonical hardware description, pin mapping and electrical constraints are in **`Hardware.md`**.
All agents MUST read and respect `Hardware.md` before writing any code.

Target MCU: `STM32F103T8` (3.3V, 8MHz external resonator)  
Motor driver: `DRV8313` (3-phase, IN1/2/3 only, always enabled, no FAULT line)  
Encoder: `TLE5012B` (3-wire SPI, shared DATA line via R5)  
Comms: UART (PA9/PA10) using Arduino-FOC-drivers **streams** API over Serial  
Tooling: VSCode, Arduino/PlatformIO-style C++ project

---

## Shared constraints for all agents

- **Source of truth**
  - Always consult `Hardware.md` for pin mappings and hardware details.
  - Do not invent extra signals (no FAULT pin, no DRV8313 enable pin, no current sense).
- **Frameworks & libraries**
  - Use **SimpleFOC** as the primary FOC library.
  - Use **Arduino-FOC-drivers** for:
    - The **TLE5012B** encoder driver.
    - The **comms/streams** UART interface described in  
      `https://github.com/simplefoc/Arduino-FOC-drivers/blob/master/src/comms/streams/README.md`.
- **MCU / board assumptions**
  - Treat the board as a custom **STM32F1 (STM32F103C8/T8)** with 3.3V IO and 8 MHz external clock.
  - No current sensing → configure SimpleFOC for **voltage (open-loop torque) FOC** with only angle feedback.
  - DRV8313’s EN/RESET/SLEEP are not MCU-controlled; driver is always on after power-up.
- **Code style & structure**
  - Language: **C++17** (Arduino/PlatformIO style).
  - Keep hardware-specific defines in one place (`board_pins.h` or similar).
  - Prefer small, focused modules over monolithic files.
  - Add clear comments where hardware quirks exist (3-wire SPI, missing FAULT, etc.).
- **Build system**
  - Assume usage from VSCode via **PlatformIO** or Arduino CLI.
  - All paths and includes must be consistent with a typical Arduino/PlatformIO project layout.

---

## Agent 1 – Firmware Architect

**Goal:** Design the high-level firmware structure and configuration for SimpleFOC on this board.

**Inputs:**

- `Hardware.md`
- This `AGENTS.md`
- SimpleFOC docs + examples
- Arduino-FOC-drivers docs (main README + TLE5012B driver + comms/streams README)

**Responsibilities:**

1. **Project structure**
   - Propose and create:
     - `platformio.ini` (or Arduino config notes) for STM32F103.
     - `src/main.cpp`
     - `src/board_pins.h`
     - `src/motor_config.h`
     - `src/sensor_tle5012b.h/.cpp` (or similar, if needed).
     - `src/comms_streams.h/.cpp`.
   - Ensure all pin numbers and peripherals match `Hardware.md`.

2. **SimpleFOC configuration**
   - Choose appropriate **SimpleFOC motor + driver classes** for:
     - 3-PWM control (DRV8313 IN1/2/3 on PA3/PB0/PB1).
     - No current sensing.
   - Define motor parameters placeholders (pole pairs, KV, phase resistance, etc.) as constants/config
     so they can be tuned later without refactoring.

3. **Sensor configuration**
   - Specify how the **TLE5012B** sensor is instantiated and wired:
     - SPI1 on PA5/PA6/PA7, CS on PA8.
     - 3-wire SPI DATA line (MISO+MOSI via R5).
   - Decide whether to use:
     - The **TLE5012B driver** from Arduino-FOC-drivers directly, or
     - A thin wrapper around it for SimpleFOC’s `Sensor` interface.

4. **Comms strategy**
   - Decide how SimpleFOC + Arduino-FOC-drivers **streams** will be used:
     - Which UART (Serial / Serial1) is used and at what baud rate.
     - Which motor and parameters are exposed over the streams/registers interface.

**Outputs:**

- A documented project layout and initial skeleton files with TODO comments.
- Pin and peripheral configuration fully aligned with `Hardware.md`.
- High-level comments in `main.cpp` describing initialization sequence:
  1. Init clocks / Serial / SPI.
  2. Init sensor.
  3. Init driver + motor.
  4. Init streams/telemetry over UART.
  5. Enter main FOC loop + command processing.

---

## Agent 2 – Board & HAL Agent

**Goal:** Implement the low-level board and peripheral layer for STM32F103T8 according to `Hardware.md`.

**Inputs:**

- `Hardware.md`
- `src/board_pins.h` skeleton from Firmware Architect
- MCU and toolchain constraints (Arduino/PlatformIO STM32F1 core)

**Responsibilities:**

1. **Board pin definitions (`board_pins.h`)**
   - Define all relevant pins as macros/constants, e.g.:
     - `PHASE_U_PIN`, `PHASE_V_PIN`, `PHASE_W_PIN`
     - `ENC_CS_PIN`, `ENC_SCK_PIN`, `ENC_MISO_PIN`, `ENC_MOSI_PIN`
     - `UART_TX_PIN`, `UART_RX_PIN`
     - `STATUS_LED_PIN`
     - Optional strap pins (`PA2`, `PA4`) for configuration.

2. **Peripheral init wrappers**
   - Provide small helper functions:
     - `void init_debug_led();`
     - `void init_spi_encoder();`
     - `void init_uart_comms();`
   - Ensure they use the **SPI/UART instances** expected by SimpleFOC (e.g. `SPI`, `Serial`).

3. **3-wire SPI handling notes**
   - Document (in comments) that:
     - TLE5012B uses **bidirectional DATA line**.
     - Hardware connects MOSI to DATA via R5 and MISO directly to DATA.
   - If any special SPI configuration is needed (e.g. half-duplex), document it or stub helper functions.

4. **Build/toolchain sanity**
   - Make sure headers include correct Arduino/STM32 core headers.
   - Avoid any direct STM32 HAL calls unless necessary; prefer Arduino-style APIs for compatibility.

**Outputs:**

- Completed `board_pins.h`.
- Small, well-commented board init utilities used by higher-level agents.

---

## Agent 3 – Motor & Sensor FOC Agent

**Goal:** Implement the SimpleFOC motor, driver, and sensor integration on this board.

**Inputs:**

- `Hardware.md`
- `board_pins.h`
- SimpleFOC library
- Arduino-FOC-drivers **TLE5012B** driver

**Responsibilities:**

1. **Sensor driver integration**
   - Include and configure the TLE5012B driver from Arduino-FOC-drivers.
   - Create a `Sensor`-compatible instance for SimpleFOC:
     - e.g. `TLE5012BEncoder` or adapter class wrapping the driver.
   - Implement `init()`, `getAngle()`, `getVelocity()` as needed.

2. **Motor driver and motor objects**
   - Configure a 3-PWM driver:
     - Using pin constants from `board_pins.h`.
     - Set `voltage_power_supply`, `voltage_limit`, etc.
   - Configure a `BLDCMotor` (or equivalent SimpleFOC motor class):
     - Link it to the driver and the TLE5012B sensor.
     - Configure motor parameters (pole pairs, PID gains, etc.) as constants or configurable values.

3. **Setup and loop**
   - Implement:
     - `void setup_motor_and_sensor();`
     - `void motor_loop_step();`
   - In `setup()`:
     - Initialize Serial, SPI, sensor, driver, motor, and FOC alignment/calibration.
   - In `loop()`:
     - Run `motor.loopFOC();`
     - Run `motor.move();`
     - Call any comms/streams processing callbacks.

4. **Diagnostics and safety**
   - Use the status LED for simple diagnostics:
     - Slow blink when idle / waiting for config.
     - Different pattern when in error / calibration / running.
   - Ensure defaults are conservative (low voltage, low velocity) until tuned.

**Outputs:**

- Fully working motor + sensor initialization and control, independent of UART streams.

---

## Agent 4 – UART Streams & Host Comms Agent

**Goal:** Wire up the UART interface using Arduino-FOC-drivers **streams** so the motor can be monitored and controlled from a host via Serial.

**Inputs:**

- `Hardware.md`
- `src/comms_streams.h/.cpp` skeleton
- SimpleFOC + Arduino-FOC-drivers (Telemetry, PacketCommander, Streams)
- Streams README:  
  `https://github.com/simplefoc/Arduino-FOC-drivers/blob/master/src/comms/streams/README.md`

**Responsibilities:**

1. **Streams initialization**
   - Wrap the selected UART (`Serial`/`Serial1`) in the streams API as described in the README.
   - Create the necessary **register abstraction** and **streams/telemetry** objects.

2. **Register mapping**
   - Expose at least the following over the register/streams interface:
     - Motor target (angle or velocity).
     - Motor actual angle and velocity.
     - Enable/disable motor flag.
     - Basic tuning parameters (e.g. PID gains) if practical.
   - Ensure register names/IDs are stable and documented in comments.

3. **Main loop integration**
   - Provide functions:
     - `void init_streams();`
     - `void handle_streams();`
   - Call `handle_streams()` from `loop()` after `motor.loopFOC()` / `motor.move()`.

4. **Host usage notes**
   - Add comments pointing to:
     - `Arduino-FOC-drivers/src/comms/streams/README.md`
     - Any expected host-side tools or scripts (VSCode terminal, Python client, etc.).

**Outputs:**

- Working UART “control plane” compatible with Arduino-FOC-drivers streams.
- Clear documentation in comments so a host program can talk to the board using the defined registers.

---

## Definition of Done

The system is considered “done” for this iteration when:

1. Code builds successfully in VSCode for STM32F103 using SimpleFOC + Arduino-FOC-drivers.
2. On power-up:
   - Board runs `setup()` and enters a stable `loop()`.
   - Status LED shows a predictable pattern (e.g. slow blink when idle).
3. TLE5012B angle is read correctly via SPI and used by SimpleFOC (verified via UART streams).
4. The motor can:
   - Hold position or run at a commanded velocity.
   - Be controlled and monitored via the UART streams interface as described in the streams README.
5. All pin mappings and hardware behavior strictly match `Hardware.md`.
