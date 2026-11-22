#pragma once

#include <Arduino.h>

// MPU6050 IMU running on the RP2040's second core.
//
// Core 1 owns I2C0 and the data-ready interrupt. It reads every sample
// (500 Hz), calibrates the gyro bias, runs a complementary filter for roll and
// pitch, integrates yaw, and accumulates test-window statistics. It never
// prints and never touches the motors or the USB link.
//
// Core 0 uses only the functions in the "core 0 API" section. They exchange
// data with core 1 through a mutex-protected snapshot and request flags, and
// return quickly.
//
// Build with -DR2W_IMU=0 for a firmware without the IMU: core 1 is then never
// started and every query reports State::Off.

#ifndef R2W_IMU
#define R2W_IMU 1
#endif

namespace Imu
{
enum class State : uint8_t
{
  Off,          // compiled out
  Starting,     // core 1 has not finished start-up yet
  NoSensor,     // no reply at the I2C address; retried every second
  Calibrating,  // gyro bias calibration in progress (keep still)
  Ok,           // running
  Stale         // no new sample for more than 100 ms (wiring or INT fault)
};

enum class CalState : uint8_t
{
  None,
  Running,
  Done,    // calibrated while still
  Forced   // never still enough; last attempt used anyway
};

// Channel order for the window statistics.
enum Channel : uint8_t { GX, GY, GZ, AX, AY, AZ, CHANNELS };

struct Snapshot
{
  bool enabled = false;               // compiled in
  State state = State::Off;
  CalState cal = CalState::None;
  uint8_t calAttempts = 0;
  uint8_t whoAmI = 0;
  float roll = 0.0F;                  // degrees, robot frame
  float pitch = 0.0F;
  float yaw = 0.0F;
  float p = 0.0F;                     // bias-corrected body rates, deg/s
  float q = 0.0F;
  float r = 0.0F;
  float ax = 0.0F;                    // acceleration, g
  float ay = 0.0F;
  float az = 0.0F;
  float tempC = 0.0F;
  float bias[3] = {};                 // current gyro bias, deg/s
  float tau = 0.0F;                   // filter time constant, s
  bool autoBias = false;              // auto-bias enabled
  bool motorsStopped = true;          // gate for auto-bias, from core 0
  uint32_t autoUpdates = 0;
  uint32_t samples = 0;
  uint32_t errors = 0;
  uint32_t stalls = 0;
  uint32_t late = 0;                  // gaps longer than 1.5 sample periods
  uint32_t accRejects = 0;            // accel correction skipped (|a| not ~1 g)
  uint16_t sampleHz = 0;              // samples counted over the last second
  uint32_t ageMs = 0;                 // time since the latest sample
  uint32_t windowId = 0;
};

struct WindowStats
{
  uint32_t id = 0;                    // 0 = no window started yet
  uint32_t samples = 0;
  uint32_t durationMs = 0;
  // Gyro channels are bias-corrected deg/s; accel channels are g.
  float mean[CHANNELS] = {};
  float sd[CHANNELS] = {};
  float min[CHANNELS] = {};
  float max[CHANNELS] = {};
  float accelMagnitudeMin = 0.0F;
  float accelMagnitudeMax = 0.0F;
  float rollMin = 0.0F;
  float rollMax = 0.0F;
  float pitchMin = 0.0F;
  float pitchMax = 0.0F;
  float rollStart = 0.0F;
  float pitchStart = 0.0F;
  float yawChange = 0.0F;             // unwrapped, degrees
  uint32_t movingSamples = 0;         // samples taken while motors were running
  uint32_t late = 0;
  uint32_t errors = 0;
  uint32_t stalls = 0;
  uint32_t accRejects = 0;
  uint32_t gapMinUs = 0;
  uint32_t gapMaxUs = 0;
  uint32_t latencyMaxUs = 0;          // data-ready interrupt to data in hand
  uint32_t filterMaxUs = 0;           // attitude maths per sample
  uint32_t loopMaxUs = 0;             // longest core 1 loop pass
};

// One processed sample, for telemetry. Core 1 pushes every sample into a
// single-producer/single-consumer queue; core 0 pops them (Telemetry).
struct Sample
{
  uint32_t n = 0;                     // core 1 sample counter (gaps = drops)
  uint32_t tUs = 0;                   // micros() of the data-ready interrupt
  float roll = 0.0F;                  // degrees, robot frame
  float pitch = 0.0F;
  float yaw = 0.0F;
  float p = 0.0F;                     // bias-corrected rates, deg/s
  float q = 0.0F;
  float r = 0.0F;
  float ax = 0.0F;                    // acceleration, g
  float ay = 0.0F;
  float az = 0.0F;
  float uMmS2 = 0.0F;                 // balance: commanded wheel accel
  int32_t vSps = 0;                   // balance: commanded wheel speed
  uint8_t flags = 0;                  // SAMPLE_* bits below, plus
                                      // Balance::FLAG_ACTIVE / FLAG_DRY
};

constexpr uint8_t SAMPLE_LATE = 0x01;         // gap > 1.5 periods before it
constexpr uint8_t SAMPLE_STALL = 0x02;        // read by stall recovery
constexpr uint8_t SAMPLE_ACC_REJECT = 0x04;   // accel correction skipped
constexpr uint8_t SAMPLE_CALIBRATING = 0x08;  // gyro calibration running
constexpr uint8_t SAMPLE_NO_ATTITUDE = 0x10;  // attitude not yet valid

// ---- core 1 (called from setup1() / loop1()) ----
void beginCore1();
void updateCore1();

// ---- core 0 API ----
bool enabled();
void snapshot(Snapshot& out);
bool windowStats(WindowStats& out);        // false if no window started
uint32_t startWindow();                    // returns the new window id
bool requestCalibration();                 // false if not running
void zeroYaw();
bool setTau(float seconds);                // false if out of range
void setAutoBias(bool on);
void setMotorsStopped(bool stopped);       // call every loop pass
bool popSample(Sample& out);               // false if the queue is empty
uint32_t sampleQueueDrops();               // samples lost to a full queue

const char* stateName(State state);
const char* calName(CalState cal);

constexpr float MIN_TAU_S = 0.05F;
constexpr float MAX_TAU_S = 5.0F;
}  // namespace Imu
