#pragma once
#include <Arduino.h>

namespace Config {
  constexpr uint32_t SERIAL_BAUD = 115200;
  constexpr uint32_t TMC_UART_BAUD = 115200;
  constexpr uint16_t TMC_RUN_CURRENT_MA = 400;
  constexpr uint16_t TMC_MICROSTEPS = 16;
  constexpr uint32_t STATUS_PERIOD_MS = 1000;
  constexpr uint32_t TEST_STEP_COUNT = 10;
  constexpr uint32_t TEST_STEP_INTERVAL_US = 2000;
  constexpr uint32_t MOTION_TEST_STEP_COUNT = 200;
  constexpr uint32_t MOTION_TEST_STEP_INTERVAL_US = 10000;
}
