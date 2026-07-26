#pragma once
#include <Arduino.h>

namespace HW {
  constexpr uint8_t LEFT_STEP  = 11;
  constexpr uint8_t LEFT_DIR   = 10;
  constexpr uint8_t LEFT_EN    = 12;

  constexpr uint8_t RIGHT_STEP = 6;
  constexpr uint8_t RIGHT_DIR  = 5;
  constexpr uint8_t RIGHT_EN   = 7;

  constexpr uint8_t UART_TX    = 8;
  constexpr uint8_t UART_RX    = 9;
}