#pragma once
#include <Arduino.h>

namespace Config {
  constexpr uint32_t SERIAL_BAUD = 115200;
  constexpr uint32_t STATUS_PERIOD_MS = 1000;
  constexpr uint32_t TEST_STEP_COUNT = 10;
  constexpr uint32_t TEST_STEP_INTERVAL_US = 2000;
}
