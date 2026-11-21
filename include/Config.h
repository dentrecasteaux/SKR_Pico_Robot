#pragma once
#include <Arduino.h>

namespace Config {
  constexpr uint32_t SERIAL_BAUD = 115200;
  constexpr uint32_t TMC_UART_BAUD = 115200;
  constexpr uint32_t TMC_ACTIVE_POLL_INTERVAL_MS = 250;
  constexpr uint16_t TMC_RUN_CURRENT_MA = 400;
  constexpr uint16_t TMC_MAX_CURRENT_MA = 400;
  // A simple, low-resolution setting for initial motion bring-up. This can
  // later become a runtime TMC2209 setting rather than a fixed constant.
  constexpr uint16_t TMC_MICROSTEPS = 4;
  constexpr uint32_t STATUS_PERIOD_MS = 1000;
  constexpr uint32_t TEST_STEP_COUNT = 10;
  constexpr uint32_t TEST_STEP_INTERVAL_US = 2000;
  constexpr uint32_t MOTION_TEST_STEP_COUNT = 200;
  constexpr uint32_t MOTION_TEST_STEP_INTERVAL_US = 10000;
  constexpr int32_t MAX_SPEED_STEPS_PER_SECOND = 2000;
  constexpr int32_t MAX_PIO_STEP_FREQUENCY_HZ = 50000;
  constexpr uint32_t MAX_ACCELERATION_STEPS_PER_SECOND_SQUARED = 200;
  constexpr float DEFAULT_ACCELERATION_MM_PER_SECOND_SQUARED = 150.0F;
  constexpr float MIN_ACCELERATION_MM_PER_SECOND_SQUARED = 10.0F;
  constexpr float MAX_ACCELERATION_MM_PER_SECOND_SQUARED = 500.0F;

  constexpr float WHEEL_DIAMETER_MM = 61.0F;
  constexpr float WHEEL_TRACK_MM = 188.0F;
  constexpr uint16_t MOTOR_FULL_STEPS_PER_REVOLUTION = 200;
  constexpr float MAX_LINEAR_SPEED_MM_PER_SECOND = 1000.0F;
  constexpr float MAX_TURN_SPEED_DEGREES_PER_SECOND = 360.0F;
  constexpr float MAX_MOVE_DISTANCE_MM = 10000.0F;
  constexpr float MAX_TURN_ANGLE_DEGREES = 3600.0F;
  constexpr uint32_t MIN_VELOCITY_LEASE_MS = 100;
  constexpr uint32_t MAX_VELOCITY_LEASE_MS = 2000;
  constexpr uint32_t CALIBRATION_MOVE_STEP_INTERVAL_US = 10000;

  // IMU (MPU6050). Robot frame: X forward, Y left, Z up; positive yaw turns
  // left, matching the W (deg/s) convention of SET_VELOCITY and TURN.
  // Mounting: robot axis i = IMU_AXIS_SIGN[i] * sensor axis IMU_AXIS_FROM[i]
  // (0 = sensor X, 1 = Y, 2 = Z), applied to both gyro and accelerometer.
  // Identity until the board is mounted and its orientation measured.
  constexpr uint8_t IMU_AXIS_FROM[3] = {0, 1, 2};
  constexpr int8_t IMU_AXIS_SIGN[3] = {1, 1, 1};
  constexpr uint32_t IMU_I2C_HZ = 400000;
  constexpr float IMU_FILTER_TAU_S = 0.5F;
  // Auto-bias may only run after the motors have been stopped this long.
  constexpr uint32_t IMU_AUTO_BIAS_SETTLE_MS = 1000;
}
