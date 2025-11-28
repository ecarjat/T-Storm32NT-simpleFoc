#include <Arduino.h>
#include <SimpleFOC.h>

#include "board_pins.h"
#include "comms_streams.h"
#include "motor_config.h"
#include "sensor_tle5012b.h"

// UART1 is the primary host interface (PA9/PA10).
constexpr unsigned long UART_BAUD = 115200;

// SimpleFOC objects
BLDCMotor motor(motor_config::POLE_PAIRS);
BLDCDriver3PWM driver(PHASE_U_PIN, PHASE_V_PIN, PHASE_W_PIN);
static bool system_running = false;

// Forward declarations
static void setup_driver_and_motor(bool use_encoder);
static void heartbeat_led(bool running);

void setup() {
  // 1) Init basic IO (LED + UART clocking) early.
  init_debug_led();
  digitalWrite(STATUS_LED_PIN, HIGH); // solid on during setup
  init_uart_comms(UART_BAUD);

  // 2) Configuration: use encoder or force open-loop based on board setting.
  const bool use_encoder = BOARD_USE_ENCODER;

  // 3) Bring up SPI1 + encoder (3-wire DATA).
  if (use_encoder) {
    init_spi_encoder();
    setup_tle5012b_sensor();
  }

  // 4) Configure driver + motor objects and run FOC alignment.
  setup_driver_and_motor(use_encoder);

  // 5) Initialize UART streams/telemetry.
  init_streams(motor);
  system_running = true;
}

void loop() {
  motor.loopFOC(); // Field-oriented control + sensor update
  motor.move();    // Target set via streams or defaults

  handle_streams(motor);
  heartbeat_led(system_running);
}

static void setup_driver_and_motor(bool use_encoder) {
  // Driver settings for DRV8313 (3-PWM, no enable/fault GPIO).
  driver.voltage_power_supply = motor_config::SUPPLY_VOLTAGE;
  driver.voltage_limit = motor_config::DRIVER_VOLTAGE_LIMIT;
  driver.pwm_frequency = 20000; // Hz, adjust per DRV8313/efficiency
  driver.init();

  // Motor configuration: with encoder = closed-loop velocity, without = open-loop velocity.
  motor.linkDriver(&driver);
  if (use_encoder) {
    motor.linkSensor(&encoder_sensor);
    motor.controller = MotionControlType::velocity;
  } else {
    motor.controller = MotionControlType::velocity_openloop;
  }
  motor.voltage_limit = motor_config::DRIVER_VOLTAGE_LIMIT;
  motor.velocity_limit = motor_config::VELOCITY_LIMIT;
  motor.PID_velocity.P = motor_config::PID_P;
  motor.PID_velocity.I = motor_config::PID_I;
  motor.PID_velocity.D = motor_config::PID_D;
  motor.LPF_velocity.Tf = motor_config::LPF_TF;

  motor.init();
  if (use_encoder) {
    motor.initFOC();
  }
}

static void heartbeat_led(bool running) {
  if (!running) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    return;
  }
  static unsigned long last_toggle = 0;
  static bool led_state = false;
  const unsigned long interval = running ? 500 : 250; // ms
  const unsigned long now = millis();
  if (now - last_toggle >= interval) {
    led_state = !led_state;
    digitalWrite(STATUS_LED_PIN, led_state ? HIGH : LOW);
    last_toggle = now;
  }
}
