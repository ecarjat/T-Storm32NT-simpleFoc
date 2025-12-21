#include <Arduino.h>
#include <HardwareTimer.h>
#include <SimpleFOC.h>

#include <cstdio>

#include "board_pins.h"
#include "calibrated_sensor.h"
#include "comms_streams.h"
#include "driver/tle5012b_full_duplex.h"
#include "log_packet.h"
#include "motor_config.h"
#include "runtime_settings.h"
#include "status_led.h"

#if defined(_STM32_DEF_)
#include <drivers/hardware_specific/stm32/stm32_mcu.h>
#endif

uint32_t maxTime = 0;
uint32_t totalTime = 0;
uint32_t count = 0;

static uint32_t loop_total_us = 0;
static uint32_t loop_samples = 0;
static uint32_t loop_max_us = 0;
static uint32_t loop_last_ts = 0;

static volatile bool control_isr_active = false;

// SimppleFOC loop frequency
constexpr unsigned long CONTROL_FREQUENCY = 2000; // Hz

// UART1 is the primary host interface (PA9/PA10).
constexpr unsigned long UART_BAUD = 460800;
constexpr const char* APP_VERSION = "app_v1.1.0";

// SimpleFOC objects
BLDCMotor motor(motor_config::POLE_PAIRS);
BLDCDriver3PWM driver(PHASE_U_PIN, PHASE_V_PIN, PHASE_W_PIN);
TLE5012BFullDuplex encoder_sensor(ENC_MOSI_PIN, ENC_MISO_PIN, ENC_SCK_PIN, ENC_CS_PIN);
StoredCalibratedSensor calibrated_sensor(encoder_sensor);
static bool system_running = false;

// Hardware timer
HardwareTimer controlTimer(TIM4);

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
static void setup_driver_and_motor(bool use_encoder, Sensor* sensor);
static void log_simplefoc_timers();
static void control_timer_isr();

void setup() {
  // Use the vector table provided by Arduino startup (VTOR should already be set)

  // Early LED sanity blink to verify app entry (uses Arduino)
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(200);
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(200);
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(200);
  digitalWrite(STATUS_LED_PIN, LOW);
  delay(200);

  // 1) Init basic IO (LED + UART DMA) early.
  status_led_init();
  init_uart_dma(UART_BAUD);
  const bool settings_loaded = load_settings_from_flash();
  bool calibration_loaded = false;
  if (settings_loaded) {
  } else {
  }

  // 2) Configuration: use encoder or force open-loop based on board setting.
  const bool use_encoder = BOARD_USE_ENCODER;

  // 3) Bring up SPI1 + encoder (3-wire DATA).
  Sensor* active_sensor = nullptr;
  if (use_encoder) {
    encoder_sensor.init();
    calibrated_sensor.setCalibrationData(&runtime_settings().calibration);
    if (runtime_settings().calibration.valid &&
        runtime_settings().calibration.lut_size == motor_config::CAL_LUT_SIZE) {
      // Restore calibration: use calibrated wrapper.
      motor.zero_electric_angle = runtime_settings().calibration.zero_electric_angle;
      motor.sensor_direction = static_cast<Direction>(runtime_settings().calibration.direction);
      active_sensor = &calibrated_sensor;
      calibration_loaded = true;
    } else {
      active_sensor = &encoder_sensor;
    }
  }

  // 4) Configure driver + motor objects and run FOC alignment.
  setup_driver_and_motor(use_encoder, active_sensor);

  // 5) Initialize UART streams/telemetry.
  init_streams(motor, driver, encoder_sensor, calibrated_sensor);
  log_packet(LOG_INFO, "APP", APP_VERSION);
  log_packet(LOG_INFO, "SET", settings_loaded ? "LOADED" : "DEFAULT");
  if (calibration_loaded) {
    log_packet(LOG_INFO, "CAL", "LOADED");
  }
  // configure TIM4 for 2 kHz
  controlTimer.setOverflow(CONTROL_FREQUENCY, HERTZ_FORMAT);
  controlTimer.attachInterrupt(control_timer_isr);
  // start the timer
  controlTimer.resume();
  status_led_set_running(true);
  _delay(1000);

  system_running = true;
}

static void control_timer_isr() {
  if (control_isr_active) {
    status_led_pulse_isr();
    return;
  }
  control_isr_active = true;
  uint32_t start = micros();
  motor.loopFOC();
  motor.move();

  uint32_t elapsed = micros() - start;
  totalTime += elapsed;
  count++;
  if (elapsed > maxTime) {
    maxTime = elapsed;
  }
  control_isr_active = false;
}

static void log_timer() {
  char msg[128];

  // convert maxTime (already integer) directly
  unsigned long maxVal = maxTime;

  // convert average into fixed-point (two decimal places)
  // multiply by 100 to preserve two decimals, then split
  long avg100 = (long)((totalTime * 100) / (count ? count : 1));
  long avgInt = avg100 / 100;   // integer part
  long avgFrac = avg100 % 100;  // fractional part

  // make sure fractional is positive
  if (avgFrac < 0) avgFrac = -avgFrac;

  snprintf(msg, sizeof(msg), "MAX=%lu AVG=%ld.%02ld, C:%lu", maxVal, avgInt, avgFrac, count);

  log_packet(LOG_INFO, "TIMER", msg);
}

void loop() {
  const uint32_t now = micros();
  if (loop_last_ts != 0) {
    const uint32_t delta = now - loop_last_ts;
    loop_total_us += delta;
    loop_samples++;
    if (delta > loop_max_us) {
      loop_max_us = delta;
    }
    if (loop_total_us >= 1000000) {
      const uint32_t hz = loop_samples;
      char msg[48];
      snprintf(msg, sizeof(msg), "HZ=%lu MAX=%lu", static_cast<unsigned long>(hz),
               static_cast<unsigned long>(loop_max_us));
      log_packet(LOG_INFO, "LOOP", msg);
      loop_total_us = 0;
      loop_samples = 0;
      loop_max_us = 0;
    }
  }
  loop_last_ts = now;
  if (!(count % 10000)) {
    log_timer();
  }
  handle_streams(motor);
  status_led_tick();
}

static void setup_driver_and_motor(bool use_encoder, Sensor* sensor) {
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

static void log_simplefoc_timers() {
#if defined(_STM32_DEF_)
  const int motors_used = stm32_getNumMotorsUsed();
  char msg[60] = {0};

  for (int m = 0; m < motors_used; m++) {
    STM32DriverParams* params = stm32_getMotorUsed(m);
    if (!params) {
      continue;
    }
    for (int i = 0; i < 6; i++) {
      TIM_HandleTypeDef* ht = params->timers_handle[i];
      if (!ht) {
        break;
      }
      const char* name = "UNKNOWN";
#if defined(TIM1)
      if (ht->Instance == TIM1) name = "TIM1";
#endif
#if defined(TIM2)
      if (ht->Instance == TIM2) name = "TIM2";
#endif
#if defined(TIM3)
      if (ht->Instance == TIM3) name = "TIM3";
#endif
#if defined(TIM4)
      if (ht->Instance == TIM4) name = "TIM4";
#endif
#if defined(TIM5)
      if (ht->Instance == TIM5) name = "TIM5";
#endif
#if defined(TIM8)
      if (ht->Instance == TIM8) name = "TIM8";
#endif
      snprintf(msg, sizeof(msg), "M%d T%d=%s CH%lu", m, i, name,
               static_cast<unsigned long>(params->channels[i]));
      log_packet(LOG_INFO, "PWM", msg);
    }
  }

  snprintf(msg, sizeof(msg), "TIMERS_USED=%d", stm32_getNumTimersUsed());
  log_packet(LOG_INFO, "PWM", msg);
#endif
}
