#include "Balance.h"

#include <cmath>

#include "Stepper.h"
#include "pico/mutex.h"

namespace Balance
{
namespace
{
constexpr float D2R = 0.017453292519943295F;
constexpr float R2D = 57.29577951308232F;
constexpr float G = 9.80665F;
constexpr uint32_t RELEASE_DELAY_US = 500000;   // trip -> drivers off
constexpr uint32_t IMU_STALE_US = 20000;        // no sample for 20 ms -> trip
constexpr uint32_t ARM_REQUEST_TIMEOUT_MS = 200;
constexpr float MIN_STEP_RATE = 10.0F;          // steps/s; below this, no pulses
constexpr uint32_t MIN_INTERVAL_US = 20;        // 50 kHz ceiling
constexpr uint32_t RESTART_IF_INTERVAL_OVER_US = 2000;

Stepper* leftStepper = nullptr;
Stepper* rightStepper = nullptr;

auto_init_mutex(balanceMutex);

// core 0 -> core 1
volatile bool armRequest = false;
volatile bool armDry = false;
volatile float armStepsPerMetre = 0.0F;
volatile uint32_t armRequestMs = 0;     // core 0 only
volatile bool stopRequest = false;
volatile Reason stopReason = Reason::Command;
Params pendingParams;                   // under balanceMutex
volatile uint32_t paramsVersion = 1;

// shared (written by core 1)
volatile State sharedState = State::Off;
Status sharedStatus;                    // under balanceMutex

// ------------------------------------------------------------ core 1 only
Params p;
uint32_t appliedParamsVersion = 0;
State state = State::Off;
Reason reason = Reason::None;
bool dry = false;
float stepsPerMetre = 0.0F;
uint32_t armStartUs = 0;
uint32_t engageUs = 0;
uint32_t tripUs = 0;
uint32_t lastSampleUs = 0;
uint32_t lastStepUs = 0;
bool haveLastStep = false;
float v = 0.0F;
float x = 0.0F;
float u = 0.0F;
float scale = 0.0F;
uint32_t pinnedSinceUs = 0;
bool pinned = false;
uint32_t engages = 0;
uint32_t trips = 0;
float maxAbsU = 0.0F;
float maxAbsError = 0.0F;
float lastTheta = 0.0F;
float lastError = 0.0F;
float lastRate = 0.0F;

// Step generator: both wheels at the same rate.
bool genRunning = false;
bool genForward = true;
uint32_t genInterval = 0;
uint32_t genRate = 0;

void stopPulses()
{
  if (leftStepper) leftStepper->stopPulses();
  if (rightStepper) rightStepper->stopPulses();
  genRunning = false;
  genInterval = 0;
  genRate = 0;
}

void startPulses(bool forward, uint32_t interval)
{
  leftStepper->stopPulses();
  rightStepper->stopPulses();
  leftStepper->setDirection(forward);
  rightStepper->setDirection(forward);
  leftStepper->startPulses(UINT32_MAX, interval);
  rightStepper->startPulses(UINT32_MAX, interval);
  genRunning = true;
  genForward = forward;
  genInterval = interval;
}

// Apply a wheel speed straight away. The PIO program only reads a new
// interval after finishing the current one, so when the current interval is
// long (slow) and the new one much shorter, the state machines are restarted
// instead (which also gives an immediate step). Direction changes restart.
void drive(float speedMps)
{
  if (!leftStepper || !rightStepper) return;
  const float rate = speedMps * stepsPerMetre;
  const float magnitude = fabsf(rate);
  if (magnitude < MIN_STEP_RATE) {
    if (genRunning) stopPulses();
    return;
  }
  uint32_t interval = static_cast<uint32_t>(1000000.0F / magnitude);
  if (interval < MIN_INTERVAL_US) interval = MIN_INTERVAL_US;
  const bool forward = rate > 0.0F;
  genRate = static_cast<uint32_t>(magnitude);

  if (!genRunning || forward != genForward) {
    startPulses(forward, interval);
    return;
  }
  if (interval == genInterval) return;
  if (genInterval > RESTART_IF_INTERVAL_OVER_US && interval < genInterval / 2) {
    startPulses(forward, interval);
    return;
  }
  const bool leftOk = leftStepper->updatePulseInterval(interval);
  const bool rightOk = rightStepper->updatePulseInterval(interval);
  if (leftOk && rightOk) genInterval = interval;  // else retry next sample
}

void enableDrivers()
{
  leftStepper->enable();
  rightStepper->enable();
}

void disableDrivers()
{
  leftStepper->disable();
  rightStepper->disable();
  genRunning = false;
  genInterval = 0;
  genRate = 0;
}

void trip(Reason why, uint32_t nowUs)
{
  if (state == State::Off || state == State::Tripped) return;
  stopPulses();                 // wheels locked from here
  reason = why;
  if (why != Reason::Command) ++trips;
  u = 0.0F;
  scale = 0.0F;
  if (dry) {
    state = State::Off;         // drivers were never on
  } else {
    state = State::Tripped;
    tripUs = nowUs;
  }
}

void publish(uint32_t nowUs)
{
  sharedState = state;
  if (!mutex_try_enter(&balanceMutex, nullptr)) return;
  Status& s = sharedStatus;
  s.state = state;
  s.reason = reason;
  s.dry = dry;
  s.thetaDeg = lastTheta;
  s.errorDeg = lastError;
  s.rateDps = lastRate;
  s.u = u;
  s.v = v;
  s.x = x;
  s.scale = scale;
  s.runMs = (state == State::Running || state == State::Dry)
                ? (nowUs - engageUs) / 1000
                : 0;
  s.engages = engages;
  s.trips = trips;
  s.maxAbsU = maxAbsU;
  s.maxAbsErrorDeg = maxAbsError;
  s.stepRateSps = genRunning ? genRate : 0;
  mutex_exit(&balanceMutex);
}

void takeParams()
{
  if (appliedParamsVersion == paramsVersion) return;
  if (!mutex_try_enter(&balanceMutex, nullptr)) return;
  p = pendingParams;
  appliedParamsVersion = paramsVersion;
  mutex_exit(&balanceMutex);
}

void engage(uint32_t nowUs)
{
  engageUs = nowUs;
  v = 0.0F;
  x = 0.0F;
  u = 0.0F;
  scale = 0.0F;
  pinned = false;
  maxAbsU = 0.0F;
  maxAbsError = 0.0F;
  ++engages;
}
}  // namespace

// ================================================================== core 0
void attach(Stepper& left, Stepper& right)
{
  leftStepper = &left;
  rightStepper = &right;
  mutex_enter_blocking(&balanceMutex);
  pendingParams = Params();
  mutex_exit(&balanceMutex);
}

bool arm(bool dryRun, float spm)
{
  if (sharedState != State::Off || armRequest) return false;
  if (!(spm > 0.0F)) return false;
  armDry = dryRun;
  armStepsPerMetre = spm;
  armRequestMs = millis();
  stopRequest = false;
  armRequest = true;
  return true;
}

void stop(Reason why)
{
  armRequest = false;
  stopReason = why;
  stopRequest = true;
}

bool setParams(const Params& n)
{
  const bool ok =
      std::isfinite(n.kp) && n.kp >= 0.0F && n.kp <= 500.0F &&
      std::isfinite(n.kd) && n.kd >= 0.0F && n.kd <= 50.0F &&
      std::isfinite(n.kv) && fabsf(n.kv) <= 100.0F &&
      std::isfinite(n.kx) && fabsf(n.kx) <= 100.0F &&
      n.aMax > 0.0F && n.aMax <= 20.0F && n.vMax > 0.0F && n.vMax <= 1.0F &&
      n.softStartS >= 0.0F && n.softStartS <= 5.0F &&
      n.tiltFwdDeg > n.trimDeg && n.trimDeg > n.tiltBackDeg &&
      n.tiltFwdDeg <= 45.0F && n.tiltBackDeg >= -45.0F &&
      n.armWindowDeg >= 0.2F && n.armWindowDeg <= 10.0F &&
      n.armRateDps >= 1.0F && n.armRateDps <= 200.0F &&
      n.armTimeoutS >= 1.0F && n.armTimeoutS <= 120.0F &&
      n.runawayS >= 0.05F && n.runawayS <= 5.0F;
  if (!ok) return false;
  mutex_enter_blocking(&balanceMutex);
  pendingParams = n;
  paramsVersion = paramsVersion + 1;
  mutex_exit(&balanceMutex);
  return true;
}

Params params()
{
  mutex_enter_blocking(&balanceMutex);
  const Params out = pendingParams;
  mutex_exit(&balanceMutex);
  return out;
}

Status status()
{
  mutex_enter_blocking(&balanceMutex);
  Status out = sharedStatus;
  mutex_exit(&balanceMutex);
  // An arm request not yet taken by core 1 times out here.
  if (armRequest && millis() - armRequestMs > ARM_REQUEST_TIMEOUT_MS) {
    armRequest = false;
  }
  return out;
}

bool active() { return sharedState != State::Off || armRequest; }

State currentState() { return sharedState; }

// ================================================================== core 1
StepOutput step(uint32_t tUs, float pitchDeg, float rollDeg, float qDps,
                float rDps, bool attitudeValid, bool sampleBad)
{
  (void)sampleBad;
  takeParams();
  lastSampleUs = tUs;
  const float dt =
      haveLastStep ? fminf(fmaxf((tUs - lastStepUs) * 1e-6F, 0.0005F), 0.02F)
                   : 0.002F;
  lastStepUs = tUs;
  haveLastStep = true;

  // Pitch rate exactly as the attitude filter propagates pitch.
  const float rr = rollDeg * D2R;
  const float rateDps = cosf(rr) * qDps - sinf(rr) * rDps;
  lastTheta = pitchDeg;
  lastError = pitchDeg - p.trimDeg;
  lastRate = rateDps;

  if (stopRequest) {
    stopRequest = false;
    trip(stopReason, tUs);
  }

  if (armRequest && state == State::Off) {
    armRequest = false;
    dry = armDry;
    stepsPerMetre = armStepsPerMetre;
    reason = Reason::None;
    stopPulses();
    if (dry) {
      state = State::Dry;
      engage(tUs);
    } else {
      enableDrivers();          // wheels locked while the robot is lifted
      state = State::Arming;
      armStartUs = tUs;
    }
  }

  StepOutput out;
  switch (state) {
    case State::Off:
    case State::Tripped:
      publish(tUs);
      return out;
    case State::Arming:
      if (!attitudeValid) {
        trip(Reason::Imu, tUs);
      } else if ((tUs - armStartUs) * 1e-6F > p.armTimeoutS) {
        trip(Reason::ArmTimeout, tUs);
      } else if (fabsf(lastError) <= p.armWindowDeg &&
                 fabsf(rateDps) <= p.armRateDps) {
        state = State::Running;
        engage(tUs);
      }
      out.flags = FLAG_ACTIVE;
      publish(tUs);
      return out;
    case State::Running:
    case State::Dry:
      break;
  }

  // ---- control law ----
  if (state == State::Running) {
    if (!attitudeValid) {
      trip(Reason::Imu, tUs);
    } else if (pitchDeg > p.tiltFwdDeg) {
      trip(Reason::TiltFwd, tUs);
    } else if (pitchDeg < p.tiltBackDeg) {
      trip(Reason::TiltBack, tUs);
    }
    if (state != State::Running) {
      publish(tUs);
      return out;
    }
  }

  const float runS = (tUs - engageUs) * 1e-6F;
  scale = p.softStartS > 0.0F ? fminf(1.0F, runS / p.softStartS) : 1.0F;
  const float e = lastError * D2R;
  const float edot = rateDps * D2R;
  float cmd = p.kp * e + p.kd * edot + p.kv * v + p.kx * x;
  cmd *= scale;
  if (cmd > p.aMax) cmd = p.aMax;
  if (cmd < -p.aMax) cmd = -p.aMax;
  u = cmd;
  v += u * dt;
  if (v > p.vMax) v = p.vMax;
  if (v < -p.vMax) v = -p.vMax;
  x += v * dt;
  if (fabsf(u) > maxAbsU) maxAbsU = fabsf(u);
  if (fabsf(lastError) > maxAbsError) maxAbsError = fabsf(lastError);

  if (state == State::Running) {
    const bool atLimit = fabsf(v) >= p.vMax * 0.999F;
    if (atLimit && !pinned) {
      pinned = true;
      pinnedSinceUs = tUs;
    } else if (!atLimit) {
      pinned = false;
    }
    if (pinned && (tUs - pinnedSinceUs) * 1e-6F >= p.runawayS) {
      trip(Reason::Runaway, tUs);
      publish(tUs);
      return out;
    }
    drive(v);
    out.flags = FLAG_ACTIVE;
  } else {
    out.flags = FLAG_ACTIVE | FLAG_DRY;
  }
  out.uMmS2 = u * 1000.0F;
  out.vSps = static_cast<int32_t>(v * stepsPerMetre);
  publish(tUs);
  return out;
}

void poll(uint32_t nowUs)
{
  if (state == State::Tripped && nowUs - tripUs >= RELEASE_DELAY_US) {
    disableDrivers();
    state = State::Off;
    publish(nowUs);
    return;
  }
  if ((state == State::Arming || state == State::Running ||
       state == State::Dry) &&
      nowUs - lastSampleUs > IMU_STALE_US) {
    trip(Reason::Imu, nowUs);
    publish(nowUs);
  }
  if (stopRequest && (state == State::Off)) stopRequest = false;
}

float accelCompensationG()
{
  return (p.accelComp && state == State::Running) ? u / G : 0.0F;
}

const char* stateName(State s)
{
  switch (s) {
    case State::Off: return "OFF";
    case State::Arming: return "ARMING";
    case State::Running: return "RUNNING";
    case State::Dry: return "DRY";
    case State::Tripped: return "TRIPPED";
  }
  return "UNKNOWN";
}

const char* reasonName(Reason r)
{
  switch (r) {
    case Reason::None: return "NONE";
    case Reason::Command: return "COMMAND";
    case Reason::Estop: return "ESTOP";
    case Reason::TiltFwd: return "TILT_FWD";
    case Reason::TiltBack: return "TILT_BACK";
    case Reason::Runaway: return "RUNAWAY";
    case Reason::Imu: return "IMU";
    case Reason::ArmTimeout: return "ARM_TIMEOUT";
  }
  return "UNKNOWN";
}
}  // namespace Balance
