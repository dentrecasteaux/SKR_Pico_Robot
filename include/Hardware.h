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

  // MPU6050 IMU (GY-521): I2C0 on the P5 header, data-ready interrupt on the
  // E0-STOP signal pin (E0-DIAG jumper removed). See docs/hardware-reference.md.
  constexpr uint8_t IMU_SDA    = 0;
  constexpr uint8_t IMU_SCL    = 1;
  constexpr uint8_t IMU_INT    = 16;
}
