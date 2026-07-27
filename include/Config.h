#pragma once
#include <Arduino.h>

namespace Config {
  constexpr uint32_t SERIAL_BAUD = 115200;
  constexpr uint32_t TMC_UART_BAUD = 115200;
  constexpr uint16_t TMC_RUN_CURRENT_MA = 400;
  // A simple, low-resolution setting for initial motion bring-up. This can
  // later become a runtime TMC2209 setting rather than a fixed constant.
  constexpr uint16_t TMC_MICROSTEPS = 4;
  constexpr uint32_t STATUS_PERIOD_MS = 1000;
  constexpr uint32_t TEST_STEP_COUNT = 10;
  constexpr uint32_t TEST_STEP_INTERVAL_US = 2000;
  constexpr uint32_t MOTION_TEST_STEP_COUNT = 200;
  constexpr uint32_t MOTION_TEST_STEP_INTERVAL_US = 10000;
  constexpr int32_t MAX_SPEED_STEPS_PER_SECOND = 400;
  constexpr uint32_t MAX_ACCELERATION_STEPS_PER_SECOND_SQUARED = 200;

  constexpr float WHEEL_DIAMETER_MM = 61.0F;
  constexpr float WHEEL_TRACK_MM = 188.0F;
  constexpr uint16_t MOTOR_FULL_STEPS_PER_REVOLUTION = 200;
  constexpr float MAX_LINEAR_SPEED_MM_PER_SECOND = 100.0F;
  constexpr float MAX_TURN_SPEED_DEGREES_PER_SECOND = 180.0F;
  constexpr uint32_t CALIBRATION_MOVE_STEP_INTERVAL_US = 10000;
}
