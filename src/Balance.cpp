#include "Balance.h"

#include <cmath>

#include "Config.h"
#include "Imu.h"
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
constexpr float HALF_TRACK_M = Config::WHEEL_TRACK_MM / 2000.0F;
constexpr float TURN_ACCEL_DPS2 = 180.0F;
constexpr float DRIVE_JERK = 4.0F;      // m/s^3
constexpr float DRIVE_APPROACH = 4.0F;  // 1/s
constexpr float MOVE_DECEL = 0.4F;      // m/s^2, braking profile for MOVE
constexpr float TURN_DECEL_DPS2 = 144.0F;  // braking profile for TURN (< TURN_ACCEL)

Stepper* leftStepper = nullptr;
Stepper* rightStepper = nullptr;

auto_init_mutex(balanceMutex);

// core 0 -> core 1
volatile bool armRequest = false;
volatile bool armDry = false;
volatile float armStepsPerMetre = 0.0F;
volatile uint32_t armRequestMs = 0;     // core 0 only
volatile bool stopRequest = false;
volatile float driveV = 0.0F;           // m/s target
volatile float driveW = 0.0F;           // deg/s target
volatile uint32_t driveUntilMs = 0;     // lease end (millis)
volatile bool moveRequest = false;      // profiled moves (core 0 -> core 1)
volatile float moveRequestM = 0.0F;
volatile bool turnRequest = false;
volatile float turnRequestDeg = 0.0F;
volatile bool profileCancel = false;
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
// ACCEL_COMP=2: recent u (one per step, 2 ms) and a filtered dq/dt.
constexpr int U_HIST = 16;
float uHist[U_HIST] = {};
int uHead = 0;
float qdotRad = 0.0F;          // filtered pitch angular acceleration, rad/s^2
float prevRateRad = 0.0F;
bool havePrevRate = false;
constexpr float QDOT_CUTOFF_HZ = 30.0F;
// Optional notch on the feedback part of u (biquad at the 500 Hz loop rate).
struct Notch
{
  float hz = 0.0F, q = 0.0F;
  float b0 = 1.0F, b1 = 0.0F, b2 = 0.0F, a1 = 0.0F, a2 = 0.0F;
  float x1 = 0.0F, x2 = 0.0F, y1 = 0.0F, y2 = 0.0F;
  void design(float f, float quality)
  {
    hz = f;
    q = quality;
    if (f <= 0.0F) return;
    const float w0 = 6.2831853F * f / 500.0F;
    const float alpha = sinf(w0) / (2.0F * quality);
    const float c = cosf(w0);
    const float a0 = 1.0F + alpha;
    b0 = 1.0F / a0;
    b1 = -2.0F * c / a0;
    b2 = 1.0F / a0;
    a1 = -2.0F * c / a0;
    a2 = (1.0F - alpha) / a0;
  }
  void reset() { x1 = x2 = y1 = y2 = 0.0F; }
  float step(float in)
  {
    if (hz <= 0.0F) return in;
    const float out = b0 * in + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;
    x1 = in;
    y2 = y1;
    y1 = out;
    return out;
  }
};
Notch notch;
uint32_t pinnedSinceUs = 0;
bool pinned = false;
uint32_t engages = 0;
uint32_t trips = 0;
float maxAbsU = 0.0F;
float maxAbsError = 0.0F;
float lastTheta = 0.0F;
float lastError = 0.0F;
float lastRate = 0.0F;
float trimEst = 0.0F;
float vRef = 0.0F;
float aRef = 0.0F;      // m/s^2, drive acceleration set point
volatile bool kickRequest = false;   // core 0 -> core 1
volatile float kickRequestU = 0.0F;
volatile uint32_t kickRequestMs = 0;
float kickU = 0.0F;
uint32_t kickEndUs = 0;
bool kickOn = false;
float thetaRefDeg = 0.0F;
float wRef = 0.0F;      // deg/s
float xRef = 0.0F;
bool moveActive = false;
float moveTarget = 0.0F;       // x_ref target, m
uint32_t moveDeadlineUs = 0;
bool turnActive = false;
float turnTarget = 0.0F;       // heading target, deg
uint32_t turnDeadlineUs = 0;
float headingDeg = 0.0F;       // integrated IMU yaw rate
float xInt = 0.0F;             // integral of (x - x_ref), m*s

// One step generator per wheel.
struct Generator
{
  Stepper* stepper = nullptr;
  bool running = false;
  bool forward = true;
  uint32_t interval = 0;
  uint32_t rate = 0;
};
Generator leftGen;
Generator rightGen;

void stopGenerator(Generator& g)
{
  if (g.stepper) g.stepper->stopPulses();
  g.running = false;
  g.interval = 0;
  g.rate = 0;
}

void stopPulses()
{
  stopGenerator(leftGen);
  stopGenerator(rightGen);
}

void startGenerator(Generator& g, bool forward, uint32_t interval)
{
  g.stepper->stopPulses();
  g.stepper->setDirection(forward);
  g.stepper->startPulses(UINT32_MAX, interval);
  g.running = true;
  g.forward = forward;
  g.interval = interval;
}

// Apply a wheel speed straight away. The PIO program only reads a new
// interval after finishing the current one, so when the current interval is
// long (slow) and the new one much shorter, the state machine is restarted
// instead (which also gives an immediate step). Direction changes restart.
void driveWheel(Generator& g, float speedMps)
{
  if (!g.stepper) return;
  const float rate = speedMps * stepsPerMetre;
  const float magnitude = fabsf(rate);
  if (magnitude < MIN_STEP_RATE) {
    if (g.running) stopGenerator(g);
    return;
  }
  uint32_t interval = static_cast<uint32_t>(1000000.0F / magnitude);
  if (interval < MIN_INTERVAL_US) interval = MIN_INTERVAL_US;
  const bool forward = rate > 0.0F;
  g.rate = static_cast<uint32_t>(magnitude);

  if (!g.running || forward != g.forward) {
    startGenerator(g, forward, interval);
    return;
  }
  if (interval == g.interval) return;
  if (g.interval > RESTART_IF_INTERVAL_OVER_US && interval < g.interval / 2) {
    startGenerator(g, forward, interval);
    return;
  }
  if (g.stepper->updatePulseInterval(interval)) g.interval = interval;
  // else the FIFO is busy: retry next sample
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
  leftGen.running = rightGen.running = false;
  leftGen.interval = rightGen.interval = 0;
  leftGen.rate = rightGen.rate = 0;
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
  s.trimEstDeg = trimEst;
  s.vRef = vRef;
  s.wRefDps = wRef;
  s.xRef = xRef;
  s.scale = scale;
  s.runMs = (state == State::Running || state == State::Dry)
                ? (nowUs - engageUs) / 1000
                : 0;
  s.engages = engages;
  s.trips = trips;
  s.maxAbsU = maxAbsU;
  s.maxAbsErrorDeg = maxAbsError;
  s.stepRateSps = leftGen.running ? leftGen.rate : 0;
  s.moving = (moveActive ? 1 : 0) | (turnActive ? 2 : 0);
  s.headingDeg = headingDeg;
  s.xInt = p.ki * xInt;
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

void resetMotion()
{
  for (float& h : uHist) h = 0.0F;
  notch.reset();
  qdotRad = 0.0F;
  v = 0.0F;
  x = 0.0F;
  u = 0.0F;
  vRef = 0.0F;
  aRef = 0.0F;
  thetaRefDeg = 0.0F;
  wRef = 0.0F;
  xRef = 0.0F;
  moveActive = false;
  turnActive = false;
  headingDeg = 0.0F;
  xInt = 0.0F;
  trimEst = p.trimDeg;
}

void engage(uint32_t nowUs)
{
  engageUs = nowUs;
  resetMotion();
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
  leftGen.stepper = &left;
  rightGen.stepper = &right;
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

bool moveBy(float distM)
{
  if (sharedState != State::Running || !std::isfinite(distM) || fabsf(distM) > 2.0F)
    return false;
  driveUntilMs = millis();     // end any leased drive
  moveRequestM = distM;
  moveRequest = true;
  return true;
}

bool turnBy(float angleDeg)
{
  if (sharedState != State::Running || !std::isfinite(angleDeg) ||
      fabsf(angleDeg) > 1080.0F)
    return false;
  driveUntilMs = millis();
  turnRequestDeg = angleDeg;
  turnRequest = true;
  return true;
}

bool setDrive(float vMps, float wDps, uint32_t leaseMs)
{
  if (!std::isfinite(vMps) || !std::isfinite(wDps)) return false;
  profileCancel = true;        // any drive command ends a profiled move
  // Clamp rather than reject: every client (web page, robotdctl, scripts)
  // gets the balance-mode limits without having to know them.
  driveV = fmaxf(-MAX_DRIVE_MPS, fminf(MAX_DRIVE_MPS, vMps));
  driveW = fmaxf(-MAX_DRIVE_DPS, fminf(MAX_DRIVE_DPS, wDps));
  if (leaseMs < 100) leaseMs = 100;
  if (leaseMs > 2000) leaseMs = 2000;
  driveUntilMs = millis() + leaseMs;
  return true;
}

bool kick(float uMps2, uint32_t durationMs)
{
  if (!std::isfinite(uMps2) || fabsf(uMps2) > 4.0F || durationMs < 10 ||
      durationMs > 200 || currentState() != State::Running) {
    return false;
  }
  kickRequestU = uMps2;
  kickRequestMs = durationMs;
  kickRequest = true;
  return true;
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
      n.runawayS >= 0.05F && n.runawayS <= 5.0F &&
      n.trimRate >= 0.0F && n.trimRate <= 10.0F &&
      n.trimRangeDeg >= 0.0F && n.trimRangeDeg <= 10.0F &&
      n.driveAccel > 0.0F && n.driveAccel <= 3.0F &&
      n.driveFf >= 0.0F && n.driveFf <= 2.0F &&
      n.driveLeanDeg >= 0.0F && n.driveLeanDeg <= 5.0F &&
      std::isfinite(n.moveSpeed) && n.moveSpeed >= 0.02F && n.moveSpeed <= MAX_DRIVE_MPS &&
      std::isfinite(n.turnRateDps) && n.turnRateDps >= 5.0F && n.turnRateDps <= MAX_DRIVE_DPS &&
      std::isfinite(n.ki) && n.ki >= 0.0F && n.ki <= 20.0F &&
      std::isfinite(n.kiMax) && n.kiMax >= 0.1F && n.kiMax <= 5.0F &&
      n.accelComp <= 2 &&
      std::isfinite(n.imuHeightM) && n.imuHeightM >= 0.0F && n.imuHeightM <= 0.5F &&
      std::isfinite(n.compDelayS) && n.compDelayS >= 0.0F && n.compDelayS <= 0.03F &&
      std::isfinite(n.notchHz) && (n.notchHz == 0.0F || (n.notchHz >= 5.0F && n.notchHz <= 100.0F)) &&
      std::isfinite(n.notchQ) && n.notchQ >= 0.3F && n.notchQ <= 10.0F;
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

  // Pitch rate exactly as the attitude filter propagates pitch: q less the
  // learned yaw coupling. (Until 24 Nov this was cos(roll) q - sin(roll) r,
  // but the roll estimate drifts 2-3 deg in spins, since r leaks into the
  // roll gyro at about 9 %, and sin(roll) r then made a fake pitch rate of
  // 2-3 deg/s in every turn.)
  (void)rollDeg;
  const float rateDps = qDps - Imu::yawPitchCoupling() * rDps;
  if (state == State::Off || state == State::Arming) trimEst = p.trimDeg;
  lastTheta = pitchDeg;
  lastError = pitchDeg - trimEst;
  lastRate = rateDps;
  {
    const float rateRad = rateDps * D2R;
    if (havePrevRate) {
      const float raw = (rateRad - prevRateRad) / dt;
      const float a = dt / (dt + 1.0F / (6.2831853F * QDOT_CUTOFF_HZ));
      qdotRad += a * (raw - qdotRad);
    }
    prevRateRad = rateRad;
    havePrevRate = true;
  }

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
    resetMotion();
    driveV = 0.0F;
    driveW = 0.0F;
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

  headingDeg += rDps * dt;

  // Profiled moves (MOVE / TURN while balancing).
  if (profileCancel) {
    profileCancel = false;
    moveActive = false;
    turnActive = false;
  }
  if (moveRequest) {
    moveRequest = false;
    moveActive = true;
    moveTarget = xRef + moveRequestM;
    moveDeadlineUs = tUs + static_cast<uint32_t>(
        (fabsf(moveRequestM) / p.moveSpeed + 5.0F) * 1e6F);
  }
  if (turnRequest) {
    turnRequest = false;
    turnActive = true;
    turnTarget = headingDeg + turnRequestDeg;
    turnDeadlineUs = tUs + static_cast<uint32_t>(
        (fabsf(turnRequestDeg) / p.turnRateDps + 5.0F) * 1e6F);
  }

  // Drive set points: zero when the lease has expired; ramped.
  const bool leased = static_cast<int32_t>(driveUntilMs - millis()) > 0;
  float vTarget = leased ? driveV : 0.0F;
  float wTarget = leased ? driveW : 0.0F;
  float aFf = 0.0F;             // profile deceleration, fed forward
  if (moveActive) {
    const float rem = moveTarget - xRef;
    // Start braking a little early: the jerk limit delays the deceleration
    // by about DECEL/JERK, during which v_ref travels |v_ref| * DECEL/(2 JERK).
    const float remEff =
        fmaxf(0.0F, fabsf(rem) - fabsf(vRef) * MOVE_DECEL / (2.0F * DRIVE_JERK));
    const float vBrake = sqrtf(2.0F * MOVE_DECEL * remEff);
    const float vAbs = fminf(p.moveSpeed, vBrake);
    vTarget = rem >= 0.0F ? vAbs : -vAbs;
    // On the braking branch, heading for the target, the target speed falls
    // at MOVE_DECEL: feed that forward so v_ref doesn't lag it. Only when
    // moving towards the target (after an overshoot it would hold v_ref at
    // zero against the approach law: run 094839 stalled 13 mm past).
    const bool towards = (rem > 0.0F && vRef > 0.0F) || (rem < 0.0F && vRef < 0.0F);
    // Scaled by how close v_ref is to the profile, so it fades out instead of
    // stalling v_ref short of a nearby target (checked in simulation: 10 mm to
    // 1 m moves end within 1 mm and finish).
    if (vBrake < p.moveSpeed && towards)
      aFf = (rem >= 0.0F ? -MOVE_DECEL : MOVE_DECEL) *
            fminf(1.0F, fabsf(vRef) / fmaxf(vAbs, 0.001F));
    if ((fabsf(rem) < 0.003F && fabsf(vRef) < 0.01F) ||
        static_cast<int32_t>(tUs - moveDeadlineUs) >= 0) {
      moveActive = false;
      vTarget = 0.0F;
      aFf = 0.0F;
    }
  }
  if (turnActive) {
    const float rem = turnTarget - headingDeg;
    const float wAbs = fminf(p.turnRateDps,
                             sqrtf(2.0F * TURN_DECEL_DPS2 * fabsf(rem)));
    wTarget = rem >= 0.0F ? wAbs : -wAbs;
    if ((fabsf(rem) < 0.5F && fabsf(wRef) < 2.0F) ||
        static_cast<int32_t>(tUs - turnDeadlineUs) >= 0) {
      turnActive = false;
      wTarget = 0.0F;
    }
  }
  // S-curve: a_ref heads for the acceleration that closes the speed gap
  // (at most DRIVE_ACCEL), changing at most DRIVE_JERK per second.
  const float aWant = fmaxf(-p.driveAccel,
                            fminf(p.driveAccel,
                                  aFf + DRIVE_APPROACH * (vTarget - vRef)));
  const float dj = DRIVE_JERK * dt;
  aRef += fmaxf(-dj, fminf(dj, aWant - aRef));
  vRef += aRef * dt;
  const float dw = TURN_ACCEL_DPS2 * dt;
  wRef += fmaxf(-dw, fminf(dw, wTarget - wRef));
  xRef += vRef * dt;

  // Automatic trim: walk trim_est until the robot rests at x_ref. Only while
  // calm (near the speed set point and not rotating), so pushes and
  // manoeuvres don't drag it. (The old gate also needed |e| < 1 deg, which
  // stopped it ever correcting a trim more than 1 deg out: run 115606 sat
  // 0.9 m from home with e = 4 deg.)
  const bool calm = fabsf(v - vRef) < 0.05F && fabsf(lastRate) < 5.0F;
  if (state == State::Running && calm && p.kx > 0.0F && p.trimRate > 0.0F) {
    trimEst -= p.trimRate * (x - xRef) * dt;
    const float lo = fmaxf(p.trimDeg - p.trimRangeDeg, p.tiltBackDeg + 2.0F);
    const float hi = fminf(p.trimDeg + p.trimRangeDeg, p.tiltFwdDeg - 2.0F);
    if (trimEst < lo) trimEst = lo;
    if (trimEst > hi) trimEst = hi;
    lastError = pitchDeg - trimEst;
  }

  // Drive feed-forward (23 Nov, run 111831): the robot needs a lean of
  // a/g to accelerate, plus a steady lean of about 1.5-2.5 deg while moving
  // (rolling drag reacts on the body through the motors). Without these
  // the lean only comes from the speed and position errors, so it lags
  // its set point by seconds.
  thetaRefDeg = p.driveFf * aRef / G * R2D + p.driveLeanDeg * tanhf(vRef / 0.05F);
  const float e = (lastError - thetaRefDeg) * D2R;
  const float edot = rateDps * D2R;
  if (p.notchHz != notch.hz || p.notchQ != notch.q) {
    notch.design(p.notchHz, p.notchQ);
    notch.reset();
  }
  // Position integral: only while not driving forwards or backwards.
  const bool driving = fabsf(vRef) > 0.005F || fabsf(vTarget) > 0.0F;
  if (p.ki > 0.0F && state == State::Running && !driving) {
    xInt += (x - xRef) * dt;
    const float lim = p.kiMax / p.ki;
    if (xInt > lim) xInt = lim;
    if (xInt < -lim) xInt = -lim;
  } else if (driving || p.ki <= 0.0F) {
    xInt = 0.0F;
  }
  const float feedback =
      p.kp * e + p.kd * edot + p.kv * (v - vRef) + p.kx * (x - xRef) + p.ki * xInt;
  float cmd = notch.step(feedback) + p.driveFf * aRef;
  if (kickRequest) {
    kickRequest = false;
    kickU = kickRequestU;
    kickEndUs = tUs + kickRequestMs * 1000U;
    kickOn = true;
  }
  if (kickOn && static_cast<int32_t>(tUs - kickEndUs) >= 0) kickOn = false;
  if (kickOn) cmd += kickU / (scale > 0.0F ? scale : 1.0F);
  cmd *= scale;
  if (cmd > p.aMax) cmd = p.aMax;
  if (cmd < -p.aMax) cmd = -p.aMax;
  u = cmd;
  uHead = (uHead + 1) % U_HIST;
  uHist[uHead] = u;
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
  }
  const float turn = wRef * D2R * HALF_TRACK_M;
  const float vLeft = v - turn;
  const float vRight = v + turn;
  if (state == State::Running) {
    driveWheel(leftGen, vLeft);
    driveWheel(rightGen, vRight);
    out.flags = FLAG_ACTIVE | (kickOn ? FLAG_KICK : 0);
  } else {
    out.flags = FLAG_ACTIVE | FLAG_DRY;
  }
  out.uMmS2 = u * 1000.0F;
  out.vLeftSps = static_cast<int32_t>(vLeft * stepsPerMetre);
  out.vRightSps = static_cast<int32_t>(vRight * stepsPerMetre);
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
  if (state != State::Running) return 0.0F;
  if (p.accelComp == 1) return u / G;
  if (p.accelComp == 2) {
    // What the IMU itself feels: the wheel acceleration (commanded u, about
    // compDelayS late) plus the body's angular acceleration times the IMU
    // height. At h ~ l_eff the two nearly cancel, so u/g alone is wrong.
    int n = static_cast<int>(p.compDelayS / 0.002F + 0.5F);
    if (n > U_HIST - 1) n = U_HIST - 1;
    const float uDelayed = uHist[(uHead - n + U_HIST) % U_HIST];
    return (uDelayed + p.imuHeightM * qdotRad) / G;
  }
  return 0.0F;
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
