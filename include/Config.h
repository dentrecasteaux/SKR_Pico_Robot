#pragma once
#include <Arduino.h>

namespace Config {
  constexpr uint32_t SERIAL_BAUD = 115200;
  constexpr uint32_t TMC_UART_BAUD = 115200;
  constexpr uint32_t TMC_ACTIVE_POLL_INTERVAL_MS = 250;
  // Balancing chassis (Nov 2026): Rtelligent 42A08 motors, 1.8 A/phase rated,
  // 2.0 ohm, 4.8 mH. Start at 1.0 A RMS; the SKR Pico's TMC2209s have no
  // heatsink, so the runtime CONFIGURE ceiling is 1.3 A RMS.
  constexpr uint16_t TMC_RUN_CURRENT_MA = 1000;
  constexpr uint16_t TMC_MAX_CURRENT_MA = 1300;
  // 1/16 with the driver's interpolation to 256 (TMC2209 default intpol=1).
  constexpr uint16_t TMC_MICROSTEPS = 16;
  constexpr uint32_t STATUS_PERIOD_MS = 1000;
  constexpr uint32_t TEST_STEP_COUNT = 10;
  constexpr uint32_t TEST_STEP_INTERVAL_US = 2000;
  constexpr uint32_t MOTION_TEST_STEP_COUNT = 200;
  constexpr uint32_t MOTION_TEST_STEP_INTERVAL_US = 10000;
  constexpr int32_t MAX_SPEED_STEPS_PER_SECOND = 20000;  // ~1.2 m/s at 1/16
  constexpr int32_t MAX_PIO_STEP_FREQUENCY_HZ = 50000;
  constexpr uint32_t MAX_ACCELERATION_STEPS_PER_SECOND_SQUARED = 200;
  constexpr float DEFAULT_ACCELERATION_MM_PER_SECOND_SQUARED = 150.0F;
  constexpr float MIN_ACCELERATION_MM_PER_SECOND_SQUARED = 10.0F;
  // High ceiling for the Phase 7 acceleration-limit tests; default stays gentle.
  constexpr float MAX_ACCELERATION_MM_PER_SECOND_SQUARED = 20000.0F;

  // Balancing chassis, 22 Nov 2026: 1958 mm per 10 revolutions (loaded roll
  // test). Track is the measured tyre-centreline distance; refine with turn
  // tests. (Three-wheeler values were 61.0 / 188.0.)
  constexpr float WHEEL_DIAMETER_MM = 62.33F;
  constexpr float WHEEL_TRACK_MM = 223.0F;
  constexpr uint16_t MOTOR_FULL_STEPS_PER_REVOLUTION = 200;
  constexpr float MAX_LINEAR_SPEED_MM_PER_SECOND = 1000.0F;
  constexpr float MAX_TURN_SPEED_DEGREES_PER_SECOND = 360.0F;
  constexpr float MAX_MOVE_DISTANCE_MM = 10000.0F;
  constexpr float MAX_TURN_ANGLE_DEGREES = 3600.0F;
  constexpr uint32_t MIN_VELOCITY_LEASE_MS = 100;
  constexpr uint32_t MAX_VELOCITY_LEASE_MS = 2000;
  // HOLD: drivers energised with the wheels stationary (locked), for the
  // free-fall and balance-angle tests. Always time-limited.
  constexpr uint32_t DEFAULT_HOLD_LEASE_MS = 30000;
  constexpr uint32_t MIN_HOLD_LEASE_MS = 500;
  constexpr uint32_t MAX_HOLD_LEASE_MS = 120000;
  // Finite MOVE / TURN run at a constant wheel speed (no ramp), so keep it
  // gentle. Independent of microstepping.
  constexpr float CALIBRATION_MOVE_SPEED_MM_PER_SECOND = 50.0F;

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
