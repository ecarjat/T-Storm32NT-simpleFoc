#include <Arduino.h>
#include <SimpleFOC.h>

#include "board_pins.h"
#include "calibrated_sensor.h"
#include "comms_streams.h"
#include "driver/tle5012b_full_duplex.h"
#include "motor_config.h"
#include "runtime_settings.h"

// UART1 is the primary host interface (PA9/PA10).
constexpr unsigned long UART_BAUD = 460800;
constexpr const char* APP_VERSION = "app_v1.0.0";

// SimpleFOC objects
BLDCMotor motor(motor_config::POLE_PAIRS);
BLDCDriver3PWM driver(PHASE_U_PIN, PHASE_V_PIN, PHASE_W_PIN);
TLE5012BFullDuplex encoder_sensor(ENC_MOSI_PIN, ENC_MISO_PIN, ENC_SCK_PIN, ENC_CS_PIN);
StoredCalibratedSensor calibrated_sensor(encoder_sensor);
static bool system_running = false;

// No custom vector table; use Arduino's startup table

// Blink fast on HardFault using raw registers and stash PC in BKP_DR2/DR3.
extern "C" void HardFault_Handler() {
  // Capture stacked PC
  uint32_t* sp;
  __asm volatile("mrs %0, msp" : "=r"(sp));
  uint32_t pc = sp[6];
  uint32_t cfsr = SCB->CFSR;
  uint32_t hfsr = SCB->HFSR;
  uint32_t bfar = SCB->BFAR;

  // Enable backup domain and store PC (survives reset)
  RCC->APB1ENR |= RCC_APB1ENR_PWREN | RCC_APB1ENR_BKPEN;
  PWR->CR |= PWR_CR_DBP;
  BKP->DR1 = (uint16_t)(pc & 0xFFFF);
  BKP->DR2 = (uint16_t)(pc >> 16);
  BKP->DR3 = (uint16_t)(cfsr & 0xFFFF);
  BKP->DR4 = (uint16_t)(cfsr >> 16);
  BKP->DR5 = (uint16_t)(hfsr & 0xFFFF);
  BKP->DR6 = (uint16_t)(hfsr >> 16);
  BKP->DR7 = (uint16_t)(bfar & 0xFFFF);
  BKP->DR8 = (uint16_t)(bfar >> 16);

  // Enable GPIOA clock and drive PA1
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
  GPIOA->CRL |= (0x2 << GPIO_CRL_MODE1_Pos);  // MODE1 = 10 (2 MHz PP)

  while (1) {
    GPIOA->ODR ^= GPIO_ODR_ODR1;
    for (volatile uint32_t i = 0; i < 50000; i++) {
      __NOP();
    }
  }
}

// Forward declarations
static void setup_driver_and_motor(bool use_encoder, Sensor *sensor);
static void heartbeat_led(bool running);

void setup() {
  // Use the vector table provided by Arduino startup (VTOR should already be set)

  // Early LED sanity blink to verify app entry (uses Arduino)
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(200);
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(200);

  // 1) Init basic IO (LED + UART clocking) early.
  init_debug_led();
  digitalWrite(STATUS_LED_PIN, HIGH);  // solid on during setup
  init_uart_comms(UART_BAUD);
  // UART sanity: print serial marker
  Serial.print("APP_START ");
  Serial.println(APP_VERSION);
  const bool settings_loaded = load_settings_from_flash();
  if (settings_loaded) {
    Serial.println("SETTINGS_LOADED");
  } else {
    Serial.println("SETTINGS_DEFAULT");
  }

  // 2) Configuration: use encoder or force open-loop based on board setting.
  const bool use_encoder = BOARD_USE_ENCODER;

  // 3) Bring up SPI1 + encoder (3-wire DATA).
  Sensor *active_sensor = nullptr;
  if (use_encoder) {
    encoder_sensor.init();
    calibrated_sensor.setCalibrationData(&runtime_settings().calibration);
    if (runtime_settings().calibration.valid &&
        runtime_settings().calibration.lut_size == motor_config::CAL_LUT_SIZE) {
      // Restore calibration: use calibrated wrapper.
      motor.zero_electric_angle = runtime_settings().calibration.zero_electric_angle;
      motor.sensor_direction = static_cast<Direction>(runtime_settings().calibration.direction);
      active_sensor = &calibrated_sensor;
      Serial.println("CALIBRATION_LOADED");
    } else {
      active_sensor = &encoder_sensor;
    }
  }

  // 4) Configure driver + motor objects and run FOC alignment.
  setup_driver_and_motor(use_encoder, active_sensor);

  // 5) Initialize UART streams/telemetry.
  init_streams(motor, driver, encoder_sensor, calibrated_sensor);
  system_running = true;
}

void loop() {
  
  if (motor.controller == MotionControlType::angle) {
    const float c = _PI_3/motor.pole_pairs;
    motor.target = floor(motor.target / c + 0.5f) * c;
  }
  motor.loopFOC();  // Field-oriented control + sensor update
  motor.move();     // Target set via streams or defaults

  handle_streams(motor);
  heartbeat_led(system_running);
}

static void setup_driver_and_motor(bool use_encoder, Sensor *sensor) {
  // Apply persisted settings (or defaults) before init.
  apply_settings_to_objects(motor, driver);

  // Driver settings for DRV8313 (3-PWM, no enable/fault GPIO).
  driver.pwm_frequency = 20000;  // Hz, adjust per DRV8313/efficiency
  driver.init();

  // Motor configuration: with encoder = closed-loop velocity, without = open-loop velocity.
  motor.linkDriver(&driver);
  if (use_encoder && sensor) {
    motor.linkSensor(sensor);
  }

  motor.controller = MotionControlType::velocity;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.voltage_sensor_align = 3;
  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;

  // angle P controller -  default P=20
  motor.P_angle.P = 20;


  motor.init();
  if (use_encoder) {
    motor.initFOC();
  }
  motor.target = 0.0f;
  motor.disable();
}

static void heartbeat_led(bool running) {
  if (!running) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    return;
  }
  static unsigned long last_toggle = 0;
  static bool led_state = false;
  const unsigned long interval = running ? 500 : 250;  // ms
  const unsigned long now = millis();
  if (now - last_toggle >= interval) {
    led_state = !led_state;
    digitalWrite(STATUS_LED_PIN, led_state ? HIGH : LOW);
    last_toggle = now;
  }
}
