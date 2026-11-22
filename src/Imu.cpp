#include "Imu.h"

#if R2W_IMU

#include <Wire.h>
#include <cmath>
#include <cstring>

#include "Config.h"
#include "Hardware.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "pico/mutex.h"

// Implementation notes (proven in the skr-pico-mpu6050-test project):
// - 500 Hz output, DLPF ~44 Hz, +/-4 g, +/-500 deg/s, latched data-ready INT
//   that any read clears.
// - Samples are read with a non-blocking burst on the I2C0 FIFO: 1 register
//   write + 14 reads (15 of 16 TX FIFO entries), then polled from loop1().
// - Stall recovery: if INT stays high with no interrupt and no read for three
//   periods, read anyway. Timing from the last read as well as the last
//   interrupt stops back-to-back reads when INT is disconnected or the E0-STOP
//   filter holds the pin high.

namespace
{
constexpr uint8_t MPU_ADDR = 0x68;
constexpr uint32_t PERIOD_US = 2000;                 // 500 Hz
constexpr uint32_t READ_TIMEOUT_US = 3000;
constexpr uint32_t STALL_US = 3 * PERIOD_US;
constexpr uint32_t STALE_MS = 100;
constexpr uint32_t NO_SENSOR_RETRY_MS = 1000;
constexpr uint32_t ERROR_STREAK_REINIT = 50;
constexpr size_t SAMPLE_BYTES = 14;

constexpr float ACCEL_LSB_PER_G = 8192.0F;           // +/-4 g
constexpr float GYRO_LSB_PER_DPS = 65.5F;            // +/-500 deg/s
constexpr float ACC_REJECT_G = 0.15F;

constexpr uint32_t CAL_SETTLE = 100;                 // samples discarded first
constexpr uint32_t CAL_SAMPLES = 1000;               // 2 s
constexpr uint8_t CAL_MAX_ATTEMPTS = 5;
constexpr float CAL_GYRO_SD_MAX = 0.5F;              // deg/s
constexpr float CAL_ACC_SD_MAX = 0.03F;              // g

constexpr uint32_t AUTO_WINDOW = 500;                // 1 s
constexpr float AUTO_GYRO_SD_MAX = 0.25F;            // deg/s
constexpr float AUTO_RATE_MAX = 1.0F;                // deg/s
constexpr float AUTO_GAIN = 0.2F;

constexpr float D2R = 0.017453292519943295F;
constexpr float R2D = 57.29577951308232F;

namespace Reg
{
constexpr uint8_t SMPLRT_DIV = 0x19;
constexpr uint8_t CONFIG = 0x1A;
constexpr uint8_t GYRO_CONFIG = 0x1B;
constexpr uint8_t ACCEL_CONFIG = 0x1C;
constexpr uint8_t INT_PIN_CFG = 0x37;
constexpr uint8_t INT_ENABLE = 0x38;
constexpr uint8_t ACCEL_XOUT_H = 0x3B;
constexpr uint8_t PWR_MGMT_1 = 0x6B;
constexpr uint8_t WHO_AM_I = 0x75;
}  // namespace Reg

struct RegInit { uint8_t reg; uint8_t value; };
constexpr RegInit INIT_SEQUENCE[] = {
    {Reg::PWR_MGMT_1, 0x01},    // wake, gyro X clock
    {Reg::CONFIG, 0x03},        // DLPF ~44 Hz, 1 kHz internal rate
    {Reg::SMPLRT_DIV, 0x01},    // 1 kHz / 2 = 500 Hz
    {Reg::GYRO_CONFIG, 0x08},   // +/-500 deg/s
    {Reg::ACCEL_CONFIG, 0x08},  // +/-4 g
    {Reg::INT_PIN_CFG, 0x30},   // latched, active high, cleared by any read
    {Reg::INT_ENABLE, 0x01},    // data ready
};

inline float wrap180(float angle)
{
  while (angle > 180.0F) angle -= 360.0F;
  while (angle < -180.0F) angle += 360.0F;
  return angle;
}

// Running mean / variance / range (Welford, single precision).
struct RunStat
{
  uint32_t n = 0;
  float mean = 0.0F;
  float m2 = 0.0F;
  float min = 0.0F;
  float max = 0.0F;

  void reset() { *this = RunStat(); }
  void add(float value)
  {
    ++n;
    const float delta = value - mean;
    mean += delta / static_cast<float>(n);
    m2 += delta * (value - mean);
    if (n == 1 || value < min) min = value;
    if (n == 1 || value > max) max = value;
  }
  float sd() const { return n > 1 ? sqrtf(m2 / static_cast<float>(n - 1)) : 0.0F; }
};

// Test-window accumulators (built on core 1, copied to core 0 on request).
struct WindowAcc
{
  uint32_t id = 0;
  uint32_t startMs = 0;
  uint32_t lastMs = 0;
  RunStat channel[Imu::CHANNELS];
  float accelMagnitudeMin = 0.0F;
  float accelMagnitudeMax = 0.0F;
  float rollMin = 0.0F, rollMax = 0.0F, pitchMin = 0.0F, pitchMax = 0.0F;
  float rollStart = 0.0F, pitchStart = 0.0F;
  float yawChange = 0.0F;
  float lastYaw = 0.0F;
  bool attitudeSeen = false;
  uint32_t samples = 0;
  uint32_t movingSamples = 0;
  uint32_t late = 0, errors = 0, stalls = 0, accRejects = 0;
  uint32_t gapMinUs = 0, gapMaxUs = 0;
  uint32_t latencyMaxUs = 0, filterMaxUs = 0, loopMaxUs = 0;
};

// ---------------------------------------------------------------- shared
// Written by core 1, read by core 0, guarded by sharedMutex.
struct Shared
{
  Imu::Snapshot snapshot;
  uint32_t lastSampleMs = 0;
  WindowAcc window;
};

Shared shared;
auto_init_mutex(sharedMutex);

// core 0 -> core 1 requests. Core 0 sets, core 1 clears.
volatile bool requestCalibrate = false;
volatile bool requestZeroYaw = false;
volatile bool requestTau = false;
volatile float requestedTau = Config::IMU_FILTER_TAU_S;
volatile bool autoBiasEnabled = false;
volatile bool motorsStopped = true;
volatile uint32_t requestedWindowId = 0;
volatile bool core1Running = false;

// ---------------------------------------------------------------- core 1 only
volatile uint32_t isrCount = 0;
volatile uint32_t isrLastUs = 0;
volatile bool dataReady = false;

void onDataReady()
{
  isrLastUs = micros();
  ++isrCount;
  dataReady = true;
}

i2c_hw_t* const i2cHw = i2c_get_hw(i2c0);

Imu::State state = Imu::State::Starting;
uint8_t whoAmI = 0;
uint32_t lastInitAttemptMs = 0;
uint32_t errorStreak = 0;

bool readBusy = false;
uint32_t readStartUs = 0;
uint32_t readIsrUs = 0;
bool readIsStall = false;
uint32_t lastReadUs = 0;
uint32_t prevIsrUs = 0;
bool havePrevIsr = false;

uint32_t samples = 0, errors = 0, stalls = 0, late = 0;
uint32_t loopMaxUs = 0;
uint32_t rateWindowStartMs = 0, rateWindowCount = 0;
uint16_t sampleHz = 0;

// Calibration
Imu::CalState calState = Imu::CalState::None;
uint8_t calAttempts = 0;
uint32_t calSettle = 0;
RunStat calStat[Imu::CHANNELS];

// Attitude
bool attitudeValid = false;
float roll = 0.0F, pitch = 0.0F, yaw = 0.0F;
float bias[3] = {};
float tau = Config::IMU_FILTER_TAU_S;
uint32_t accRejects = 0;
uint32_t autoUpdates = 0;
RunStat autoStat[3];
float autoAccelMagnitude = 1.0F;
float autoAccelDeviationMax = 0.0F;
uint32_t motorsStoppedSinceMs = 0;
bool motorsWereStopped = true;

WindowAcc window;

// Sample queue (core 1 producer, core 0 consumer). Lock-free: each side
// owns one index; memory barriers order the data and the index updates.
constexpr uint32_t QUEUE_SIZE = 256;  // power of two; 0.5 s at 500 Hz
Imu::Sample sampleQueue[QUEUE_SIZE];
volatile uint32_t queueHead = 0;      // written by core 1
volatile uint32_t queueTail = 0;      // written by core 0
volatile uint32_t queueDrops = 0;     // written by core 1

void pushSample(const Imu::Sample& sample)
{
  const uint32_t head = queueHead;
  if (head - queueTail >= QUEUE_SIZE) {
    queueDrops = queueDrops + 1;
    return;
  }
  sampleQueue[head & (QUEUE_SIZE - 1)] = sample;
  __dmb();
  queueHead = head + 1;
}

bool writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegisters(uint8_t reg, uint8_t* buffer, size_t count)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, count) != count) return false;
  for (size_t i = 0; i < count; ++i) buffer[i] = Wire.read();
  return true;
}

// Queue [register write][read+RESTART][read]...[read+STOP]; the I2C hardware
// clocks it out while loop1() carries on.
void burstStart()
{
  while (i2cHw->rxflr) (void)i2cHw->data_cmd;
  i2cHw->enable = 0;
  i2cHw->tar = MPU_ADDR;
  i2cHw->enable = 1;
  i2cHw->data_cmd = Reg::ACCEL_XOUT_H;
  for (size_t i = 0; i < SAMPLE_BYTES; ++i) {
    uint32_t command = I2C_IC_DATA_CMD_CMD_BITS;
    if (i == 0) command |= I2C_IC_DATA_CMD_RESTART_BITS;
    if (i == SAMPLE_BYTES - 1) command |= I2C_IC_DATA_CMD_STOP_BITS;
    i2cHw->data_cmd = command;
  }
}

// 1 = done, 0 = in progress, -1 = aborted (no acknowledge)
int burstPoll(uint8_t* out)
{
  if (i2cHw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS) {
    (void)i2cHw->clr_tx_abrt;
    while (i2cHw->rxflr) (void)i2cHw->data_cmd;
    return -1;
  }
  if (i2cHw->rxflr < SAMPLE_BYTES) return 0;
  for (size_t i = 0; i < SAMPLE_BYTES; ++i) {
    out[i] = static_cast<uint8_t>(i2cHw->data_cmd);
  }
  return 1;
}

void burstAbort()
{
  i2cHw->enable |= I2C_IC_ENABLE_ABORT_BITS;
  const uint32_t startedUs = micros();
  while ((i2cHw->enable & I2C_IC_ENABLE_ABORT_BITS) &&
         micros() - startedUs < 1000) {
  }
  (void)i2cHw->clr_tx_abrt;
  while (i2cHw->rxflr) (void)i2cHw->data_cmd;
}

void startCalibration()
{
  calState = Imu::CalState::Running;
  calAttempts = 1;
  calSettle = CAL_SETTLE;
  for (RunStat& stat : calStat) stat.reset();
  attitudeValid = false;
  state = Imu::State::Calibrating;
}

// Reset, configure and read back. Blocking (~120 ms); only at start-up or
// after a fault.
bool configureSensor()
{
  uint8_t id = 0;
  if (!readRegisters(Reg::WHO_AM_I, &id, 1)) return false;
  whoAmI = id;
  if (!writeRegister(Reg::PWR_MGMT_1, 0x80)) return false;
  delay(100);
  for (const RegInit& item : INIT_SEQUENCE) {
    if (!writeRegister(item.reg, item.value)) return false;
    if (item.reg == Reg::PWR_MGMT_1) delay(10);
  }
  for (const RegInit& item : INIT_SEQUENCE) {
    uint8_t value = 0;
    if (!readRegisters(item.reg, &value, 1) || value != item.value) {
      return false;
    }
  }
  return true;
}

void initialiseSensor()
{
  lastInitAttemptMs = millis();
  readBusy = false;
  havePrevIsr = false;
  errorStreak = 0;
  if (configureSensor()) {
    startCalibration();
  } else {
    state = Imu::State::NoSensor;
    attitudeValid = false;
  }
}

// Map sensor axes into the robot frame.
inline void toRobotFrame(const float sensor[3], float robot[3])
{
  for (int i = 0; i < 3; ++i) {
    robot[i] = static_cast<float>(Config::IMU_AXIS_SIGN[i]) *
               sensor[Config::IMU_AXIS_FROM[i]];
  }
}

void calibrationStep(const float gyro[3], const float accel[3])
{
  if (calSettle) {
    --calSettle;
    return;
  }
  for (int i = 0; i < 3; ++i) {
    calStat[i].add(gyro[i]);
    calStat[3 + i].add(accel[i]);
  }
  if (calStat[0].n < CAL_SAMPLES) return;

  bool still = true;
  for (int i = 0; i < 3; ++i) {
    still &= calStat[i].sd() < CAL_GYRO_SD_MAX;
    still &= calStat[3 + i].sd() < CAL_ACC_SD_MAX;
  }
  if (!still && calAttempts < CAL_MAX_ATTEMPTS) {
    ++calAttempts;
    for (RunStat& stat : calStat) stat.reset();
    return;
  }

  for (int i = 0; i < 3; ++i) bias[i] = calStat[i].mean;
  const float ax = calStat[3].mean, ay = calStat[4].mean, az = calStat[5].mean;
  autoAccelMagnitude = sqrtf(ax * ax + ay * ay + az * az);
  roll = atan2f(ay, az) * R2D;
  pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * R2D;
  yaw = 0.0F;
  attitudeValid = true;
  calState = still ? Imu::CalState::Done : Imu::CalState::Forced;
  state = Imu::State::Ok;
  for (RunStat& stat : autoStat) stat.reset();
  autoAccelDeviationMax = 0.0F;
}

// Complementary filter (roll, pitch) and integrated yaw. Rates in deg/s
// (bias-corrected), acceleration in g.
void attitudeStep(float dt, float p, float q, float r, float ax, float ay,
                  float az)
{
  const float sr = sinf(roll * D2R), cr = cosf(roll * D2R);
  float cp = cosf(pitch * D2R);
  if (fabsf(cp) < 0.05F) cp = copysignf(0.05F, cp);
  const float tp = sinf(pitch * D2R) / cp;

  roll = wrap180(roll + (p + sr * tp * q + cr * tp * r) * dt);
  pitch += (cr * q - sr * r) * dt;
  yaw = wrap180(yaw + ((sr * q + cr * r) / cp) * dt);

  const float magnitude = sqrtf(ax * ax + ay * ay + az * az);
  if (fabsf(magnitude - 1.0F) < ACC_REJECT_G) {
    const float accelRoll = atan2f(ay, az) * R2D;
    const float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * R2D;
    const float k = dt / (tau + dt);
    roll = wrap180(roll + k * wrap180(accelRoll - roll));
    pitch += k * (accelPitch - pitch);
  } else {
    ++accRejects;
  }
  if (pitch > 90.0F) pitch = 90.0F;
  if (pitch < -90.0F) pitch = -90.0F;
}

// Bias tracking while still. Only runs when enabled and the motors have been
// stopped for IMU_AUTO_BIAS_SETTLE_MS, so a real slow turn is never absorbed.
void autoBiasStep(const float gyro[3], float accelMagnitude, bool allowed)
{
  if (!allowed) {
    if (autoStat[0].n) {
      for (RunStat& stat : autoStat) stat.reset();
      autoAccelDeviationMax = 0.0F;
    }
    return;
  }
  for (int i = 0; i < 3; ++i) autoStat[i].add(gyro[i]);
  autoAccelDeviationMax =
      max(autoAccelDeviationMax, fabsf(accelMagnitude - autoAccelMagnitude));
  if (autoStat[0].n < AUTO_WINDOW) return;

  bool still = autoAccelDeviationMax < 0.03F;
  for (int i = 0; i < 3; ++i) {
    still &= autoStat[i].sd() < AUTO_GYRO_SD_MAX;
    still &= fabsf(autoStat[i].mean - bias[i]) < AUTO_RATE_MAX;
  }
  if (still) {
    for (int i = 0; i < 3; ++i) bias[i] += AUTO_GAIN * (autoStat[i].mean - bias[i]);
    ++autoUpdates;
  }
  for (RunStat& stat : autoStat) stat.reset();
  autoAccelDeviationMax = 0.0F;
}

void resetWindow(uint32_t id)
{
  window = WindowAcc();
  window.id = id;
  window.startMs = millis();
  window.lastMs = window.startMs;
}

void windowStep(const float rates[3], const float accel[3],
                float accelMagnitude, bool moving, bool gapValid,
                uint32_t gapUs, uint32_t latencyUs, uint32_t filterUs,
                uint32_t nowMs)
{
  if (window.id == 0) return;
  for (int i = 0; i < 3; ++i) {
    window.channel[i].add(rates[i]);
    window.channel[3 + i].add(accel[i]);
  }
  const bool first = window.samples == 0;
  ++window.samples;
  if (moving) ++window.movingSamples;
  if (first || accelMagnitude < window.accelMagnitudeMin) {
    window.accelMagnitudeMin = accelMagnitude;
  }
  if (first || accelMagnitude > window.accelMagnitudeMax) {
    window.accelMagnitudeMax = accelMagnitude;
  }
  if (attitudeValid) {
    if (!window.attitudeSeen) {
      // Roll and pitch ranges are relative to the first valid attitude.
      window.attitudeSeen = true;
      window.rollStart = roll;
      window.pitchStart = pitch;
      window.lastYaw = yaw;
    }
    const float dRoll = wrap180(roll - window.rollStart);
    const float dPitch = pitch - window.pitchStart;
    window.rollMin = min(window.rollMin, dRoll);
    window.rollMax = max(window.rollMax, dRoll);
    window.pitchMin = min(window.pitchMin, dPitch);
    window.pitchMax = max(window.pitchMax, dPitch);
    window.yawChange += wrap180(yaw - window.lastYaw);
    window.lastYaw = yaw;
  }
  if (gapValid) {
    if (window.gapMinUs == 0 || gapUs < window.gapMinUs) window.gapMinUs = gapUs;
    window.gapMaxUs = max(window.gapMaxUs, gapUs);
  }
  window.latencyMaxUs = max(window.latencyMaxUs, latencyUs);
  window.filterMaxUs = max(window.filterMaxUs, filterUs);
  window.loopMaxUs = max(window.loopMaxUs, loopMaxUs);
  window.lastMs = nowMs;
}

void publish(const float rates[3], const float accel[3], float tempC,
             uint32_t nowMs)
{
  mutex_enter_blocking(&sharedMutex);
  Imu::Snapshot& s = shared.snapshot;
  s.enabled = true;
  s.state = state;
  s.cal = calState;
  s.calAttempts = calAttempts;
  s.whoAmI = whoAmI;
  s.roll = roll;
  s.pitch = pitch;
  s.yaw = yaw;
  s.p = rates[0];
  s.q = rates[1];
  s.r = rates[2];
  s.ax = accel[0];
  s.ay = accel[1];
  s.az = accel[2];
  s.tempC = tempC;
  for (int i = 0; i < 3; ++i) s.bias[i] = bias[i];
  s.tau = tau;
  s.autoBias = autoBiasEnabled;
  s.motorsStopped = motorsStopped;
  s.autoUpdates = autoUpdates;
  s.samples = samples;
  s.errors = errors;
  s.stalls = stalls;
  s.late = late;
  s.accRejects = accRejects;
  s.sampleHz = sampleHz;
  s.windowId = window.id;
  shared.lastSampleMs = nowMs;
  if (window.id != 0) shared.window = window;
  mutex_exit(&sharedMutex);
}

void publishState()
{
  mutex_enter_blocking(&sharedMutex);
  shared.snapshot.enabled = true;
  shared.snapshot.state = state;
  shared.snapshot.cal = calState;
  shared.snapshot.calAttempts = calAttempts;
  shared.snapshot.whoAmI = whoAmI;
  shared.snapshot.errors = errors;
  shared.snapshot.stalls = stalls;
  shared.snapshot.tau = tau;
  shared.snapshot.autoBias = autoBiasEnabled;
  mutex_exit(&sharedMutex);
}

void processSample(const uint8_t* b, uint32_t readUs, uint32_t isrUs,
                   bool fromStall)
{
  const auto be16 = [](const uint8_t* p) {
    return static_cast<int16_t>((p[0] << 8) | p[1]);
  };
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();
  ++samples;
  ++rateWindowCount;
  errorStreak = 0;
  if (nowMs - rateWindowStartMs >= 1000) {
    sampleHz = static_cast<uint16_t>(rateWindowCount * 1000UL /
                                     (nowMs - rateWindowStartMs));
    rateWindowCount = 0;
    rateWindowStartMs = nowMs;
  }

  bool gapValid = false;
  uint32_t gapUs = 0;
  bool isLate = false;
  if (!fromStall) {
    if (havePrevIsr) {
      gapUs = isrUs - prevIsrUs;
      gapValid = true;
      if (gapUs > PERIOD_US + PERIOD_US / 2) {
        ++late;
        isLate = true;
      }
    }
    prevIsrUs = isrUs;
    havePrevIsr = true;
  } else {
    havePrevIsr = false;
  }
  const uint32_t latencyUs = fromStall ? readUs : nowUs - isrUs;

  const uint32_t filterStartUs = micros();
  const float sensorGyro[3] = {be16(b + 8) / GYRO_LSB_PER_DPS,
                               be16(b + 10) / GYRO_LSB_PER_DPS,
                               be16(b + 12) / GYRO_LSB_PER_DPS};
  const float sensorAccel[3] = {be16(b) / ACCEL_LSB_PER_G,
                                be16(b + 2) / ACCEL_LSB_PER_G,
                                be16(b + 4) / ACCEL_LSB_PER_G};
  const float tempC = be16(b + 6) / 340.0F + 36.53F;
  float gyro[3], accel[3];
  toRobotFrame(sensorGyro, gyro);
  toRobotFrame(sensorAccel, accel);
  const float accelMagnitude =
      sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);

  // Requests from core 0
  if (requestCalibrate) {
    requestCalibrate = false;
    startCalibration();
  }
  if (requestZeroYaw) {
    requestZeroYaw = false;
    yaw = 0.0F;
    if (window.attitudeSeen) window.lastYaw = 0.0F;
  }
  if (requestTau) {
    requestTau = false;
    tau = requestedTau;
  }
  const uint32_t wantedWindow = requestedWindowId;
  if (wantedWindow != window.id) resetWindow(wantedWindow);

  const bool stopped = motorsStopped;
  if (stopped && !motorsWereStopped) motorsStoppedSinceMs = nowMs;
  motorsWereStopped = stopped;

  if (calState == Imu::CalState::Running) calibrationStep(gyro, accel);

  const float rates[3] = {gyro[0] - bias[0], gyro[1] - bias[1],
                          gyro[2] - bias[2]};
  const uint32_t rejectsBefore = accRejects;
  if (attitudeValid) {
    float dt = gapValid ? gapUs * 1e-6F : PERIOD_US * 1e-6F;
    if (dt < 0.0005F) dt = 0.0005F;
    if (dt > 0.02F) dt = 0.02F;
    attitudeStep(dt, rates[0], rates[1], rates[2], accel[0], accel[1],
                 accel[2]);
    const bool settled =
        stopped && nowMs - motorsStoppedSinceMs >= Config::IMU_AUTO_BIAS_SETTLE_MS;
    autoBiasStep(gyro, accelMagnitude, autoBiasEnabled && settled);
  }
  const uint32_t filterUs = micros() - filterStartUs;

  if (window.id != 0) {
    if (isLate) ++window.late;
    window.accRejects += accRejects - rejectsBefore;
  }
  windowStep(rates, accel, accelMagnitude, !stopped, gapValid, gapUs,
             latencyUs, filterUs, nowMs);
  publish(rates, accel, tempC, nowMs);

  Imu::Sample sample;
  sample.n = samples;
  sample.tUs = fromStall ? nowUs : isrUs;
  sample.roll = roll;
  sample.pitch = pitch;
  sample.yaw = yaw;
  sample.p = rates[0];
  sample.q = rates[1];
  sample.r = rates[2];
  sample.ax = accel[0];
  sample.ay = accel[1];
  sample.az = accel[2];
  if (isLate) sample.flags |= Imu::SAMPLE_LATE;
  if (fromStall) sample.flags |= Imu::SAMPLE_STALL;
  if (accRejects != rejectsBefore) sample.flags |= Imu::SAMPLE_ACC_REJECT;
  if (calState == Imu::CalState::Running) {
    sample.flags |= Imu::SAMPLE_CALIBRATING;
  }
  if (!attitudeValid) sample.flags |= Imu::SAMPLE_NO_ATTITUDE;
  pushSample(sample);
  loopMaxUs = 0;
}

void readError(bool timeout)
{
  (void)timeout;
  ++errors;
  ++errorStreak;
  if (window.id != 0) ++window.errors;
  havePrevIsr = false;
  if (errorStreak >= ERROR_STREAK_REINIT) {
    state = Imu::State::NoSensor;
    attitudeValid = false;
  }
  publishState();
}
}  // namespace

// ================================================================= core 1
void Imu::beginCore1()
{
  // Clock the bus free if a reset interrupted a transfer (SDA held low).
  pinMode(HW::IMU_SDA, INPUT_PULLUP);
  pinMode(HW::IMU_SCL, INPUT_PULLUP);
  delayMicroseconds(50);
  if (digitalRead(HW::IMU_SCL) && !digitalRead(HW::IMU_SDA)) {
    pinMode(HW::IMU_SCL, OUTPUT);
    for (int i = 0; i < 9 && !digitalRead(HW::IMU_SDA); ++i) {
      digitalWrite(HW::IMU_SCL, LOW);
      delayMicroseconds(5);
      digitalWrite(HW::IMU_SCL, HIGH);
      delayMicroseconds(5);
    }
    pinMode(HW::IMU_SCL, INPUT_PULLUP);
  }

  Wire.setSDA(HW::IMU_SDA);
  Wire.setSCL(HW::IMU_SCL);
  Wire.setClock(Config::IMU_I2C_HZ);
  Wire.begin();

  rateWindowStartMs = millis();
  initialiseSensor();
  publishState();

  pinMode(HW::IMU_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(HW::IMU_INT), onDataReady, RISING);
  core1Running = true;
}

void Imu::updateCore1()
{
  const uint32_t loopStartUs = micros();

  if (state == State::NoSensor) {
    if (millis() - lastInitAttemptMs >= NO_SENSOR_RETRY_MS) {
      initialiseSensor();
      publishState();
    }
    return;
  }

  const bool pin = gpio_get(HW::IMU_INT);

  if (!readBusy) {
    bool start = false;
    bool stall = false;
    uint32_t isrUs = 0;
    noInterrupts();
    if (dataReady) {
      dataReady = false;
      start = true;
      isrUs = isrLastUs;
    }
    interrupts();
    const uint32_t nowUs = micros();
    if (!start && pin && nowUs - isrLastUs > STALL_US &&
        nowUs - lastReadUs > STALL_US) {
      start = true;
      stall = true;
      ++stalls;
      if (window.id != 0) ++window.stalls;
    }
    if (start) {
      lastReadUs = micros();
      readStartUs = lastReadUs;
      readIsrUs = isrUs;
      readIsStall = stall;
      burstStart();
      readBusy = true;
    }
  }

  if (readBusy) {
    uint8_t buffer[SAMPLE_BYTES];
    const int result = burstPoll(buffer);
    const uint32_t elapsedUs = micros() - readStartUs;
    if (result == 1) {
      readBusy = false;
      processSample(buffer, elapsedUs, readIsrUs, readIsStall);
    } else if (result < 0) {
      readBusy = false;
      readError(false);
    } else if (elapsedUs > READ_TIMEOUT_US) {
      burstAbort();
      readBusy = false;
      readError(true);
    }
  }

  const uint32_t loopUs = micros() - loopStartUs;
  if (loopUs > loopMaxUs) loopMaxUs = loopUs;
}

// ================================================================= core 0
bool Imu::enabled() { return true; }

void Imu::snapshot(Snapshot& out)
{
  uint32_t lastSampleMs = 0;
  mutex_enter_blocking(&sharedMutex);
  out = shared.snapshot;
  lastSampleMs = shared.lastSampleMs;
  mutex_exit(&sharedMutex);
  out.enabled = true;
  if (!core1Running) out.state = State::Starting;
  out.ageMs = lastSampleMs == 0 ? 0 : millis() - lastSampleMs;
  if ((out.state == State::Ok || out.state == State::Calibrating) &&
      lastSampleMs != 0 && out.ageMs > STALE_MS) {
    out.state = State::Stale;
  }
}

bool Imu::windowStats(WindowStats& out)
{
  WindowAcc acc;
  mutex_enter_blocking(&sharedMutex);
  acc = shared.window;
  mutex_exit(&sharedMutex);

  out = WindowStats();
  out.id = requestedWindowId;
  if (out.id == 0) return false;
  if (acc.id != out.id) return true;   // started; no sample yet

  out.samples = acc.samples;
  out.durationMs = acc.lastMs - acc.startMs;
  for (int i = 0; i < CHANNELS; ++i) {
    out.mean[i] = acc.channel[i].mean;
    out.sd[i] = acc.channel[i].sd();
    out.min[i] = acc.channel[i].min;
    out.max[i] = acc.channel[i].max;
  }
  out.accelMagnitudeMin = acc.accelMagnitudeMin;
  out.accelMagnitudeMax = acc.accelMagnitudeMax;
  out.rollMin = acc.rollMin;
  out.rollMax = acc.rollMax;
  out.pitchMin = acc.pitchMin;
  out.pitchMax = acc.pitchMax;
  out.rollStart = acc.rollStart;
  out.pitchStart = acc.pitchStart;
  out.yawChange = acc.yawChange;
  out.movingSamples = acc.movingSamples;
  out.late = acc.late;
  out.errors = acc.errors;
  out.stalls = acc.stalls;
  out.accRejects = acc.accRejects;
  out.gapMinUs = acc.gapMinUs;
  out.gapMaxUs = acc.gapMaxUs;
  out.latencyMaxUs = acc.latencyMaxUs;
  out.filterMaxUs = acc.filterMaxUs;
  out.loopMaxUs = acc.loopMaxUs;
  return true;
}

uint32_t Imu::startWindow()
{
  uint32_t id = requestedWindowId + 1;
  if (id == 0) id = 1;
  requestedWindowId = id;
  return id;
}

bool Imu::requestCalibration()
{
  if (!core1Running) return false;
  requestCalibrate = true;
  return true;
}

void Imu::zeroYaw() { requestZeroYaw = true; }

bool Imu::setTau(float seconds)
{
  if (!(seconds >= MIN_TAU_S && seconds <= MAX_TAU_S)) return false;
  requestedTau = seconds;
  requestTau = true;
  return true;
}

void Imu::setAutoBias(bool on) { autoBiasEnabled = on; }

void Imu::setMotorsStopped(bool stopped) { motorsStopped = stopped; }

bool Imu::popSample(Sample& out)
{
  const uint32_t tail = queueTail;
  if (queueHead == tail) return false;
  __dmb();
  out = sampleQueue[tail & (QUEUE_SIZE - 1)];
  __dmb();
  queueTail = tail + 1;
  return true;
}

uint32_t Imu::sampleQueueDrops() { return queueDrops; }

#else  // R2W_IMU == 0: stubs so the rest of the firmware builds unchanged.

bool Imu::enabled() { return false; }
void Imu::snapshot(Snapshot& out) { out = Snapshot(); }
bool Imu::windowStats(WindowStats& out)
{
  out = WindowStats();
  return false;
}
uint32_t Imu::startWindow() { return 0; }
bool Imu::requestCalibration() { return false; }
void Imu::zeroYaw() {}
bool Imu::setTau(float) { return false; }
void Imu::setAutoBias(bool) {}
void Imu::setMotorsStopped(bool) {}
bool Imu::popSample(Sample&) { return false; }
uint32_t Imu::sampleQueueDrops() { return 0; }
void Imu::beginCore1() {}
void Imu::updateCore1() {}

#endif  // R2W_IMU

const char* Imu::stateName(State state)
{
  switch (state) {
    case State::Off: return "OFF";
    case State::Starting: return "STARTING";
    case State::NoSensor: return "NO_SENSOR";
    case State::Calibrating: return "CALIBRATING";
    case State::Ok: return "OK";
    case State::Stale: return "STALE";
  }
  return "OFF";
}

const char* Imu::calName(CalState cal)
{
  switch (cal) {
    case CalState::None: return "NONE";
    case CalState::Running: return "RUNNING";
    case CalState::Done: return "OK";
    case CalState::Forced: return "FORCED";
  }
  return "NONE";
}
