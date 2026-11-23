#pragma once

#include <Arduino.h>

class Stepper;

// Two-wheel balance controller (commissioning plan Phases 8-9).
//
// Runs on core 1 inside the IMU sample path, so every 500 Hz sample is
// followed straight away by a control update with no cross-core latency.
// While balancing, core 1 owns both steppers (core 0 keeps the robot in
// Robot::Mode::Balance, which locks out every other motion command).
//
// Control law (wheel acceleration command, robot frame, forward positive):
//   e = theta - trim_est                           (rad)
//   u = Kp*e + Kd*theta_dot + Kv*(v - v_ref) + Kx*(x - x_ref)
//                                                  (m/s^2), clamped to +/-a_max
//   v += u*dt (clamped to +/-v_max);  x += v*dt;  x_ref += v_ref*dt
// v_ref / w_ref come from drive commands (SET_VELOCITY while balancing), with
// a lease; they ramp at driveAccel. Turning adds -/+ w_ref*track/2 to the
// left/right wheel speeds.
// Automatic trim (when Kx > 0): trim_est -= trimRate*(x - x_ref)*dt, within
// +/-trimRange of trim. A wrong trim makes the robot settle off its target
// position (Kp*e = -Kx*(x - x_ref) at rest); this walks trim_est until that
// offset is gone, e.g. after the IMU mount shifts.
// Kv and Kx default to 0 (Phase 9 PD). They take the same sign convention as
// an LQR gain row for state [x, v, theta, theta_dot], where the position and
// speed gains are normally positive (the base moves *further* first).
//
// Safety (all on core 1, independent of the Pi):
//   ARMING      waits until the robot is lifted into the arming window
//               (|theta - trim| <= armWindow, |theta_dot| <= armRate), then
//               engages with a soft start (output ramped over softStart s)
//   trips       pitch beyond tiltFwd / tiltBack (absolute IMU pitch), speed
//               pinned at v_max for runawayTime, IMU stale or invalid,
//               arming timeout. On a trip the steps stop at once (wheels
//               locked), the drivers are disabled 0.5 s later.
//   DRY         full controller with the drivers disabled (sign checks).
namespace Balance
{
enum class State : uint8_t
{
  Off,        // drivers released, core 0 owns the motors
  Arming,     // waiting for the robot to enter the arming window
  Running,    // balancing
  Dry,        // controller running, drivers disabled (no motion)
  Tripped     // stopped by a safety check; Off after the release delay
};

enum class Reason : uint8_t
{
  None,
  Command,    // BAL_STOP or STOP
  Estop,
  TiltFwd,
  TiltBack,
  Runaway,
  Imu,
  ArmTimeout
};

struct Params
{
  // Full-state pole placement (l_eff 0.20 m; poles 10 rad/s zeta 0.7 and
  // 1.2 rad/s zeta 0.9), robust for l_eff 0.12-0.30 m. Phase 9 PD was
  // KP 39 KD 2.9 KV 0 KX 0.
  float kp = 36.7F;          // m/s^2 per rad
  float kd = 4.19F;          // m/s^2 per rad/s
  float kv = 4.81F;          // m/s^2 per m/s
  float kx = 2.94F;          // m/s^2 per m
  float trimDeg = -3.9F;     // IMU pitch at the balance point
  float aMax = 6.0F;         // m/s^2
  float vMax = 0.55F;        // m/s
  float softStartS = 0.3F;
  float tiltFwdDeg = 5.0F;   // absolute IMU pitch limits
  float tiltBackDeg = -20.6F;
  float armWindowDeg = 2.0F;
  float armRateDps = 20.0F;
  float armTimeoutS = 20.0F;
  float runawayS = 0.3F;
  bool accelComp = true;     // subtract u/g from the accelerometer X axis
                             // (big improvement in push recovery, 23 Nov)
  float trimRate = 0.25F;    // deg per (m*s); 0 = no automatic trim.
                             // Adapts only while calm (see Balance.cpp).
  float trimRangeDeg = 4.0F; // trim_est stays within trim +/- this
  float driveAccel = 0.5F;   // m/s^2 ramp for v_ref (jerk-limited S-curve)
  float driveFf = 1.0F;      // drive feed-forward: lean a_ref/g and add
                             // a_ref to u (0 = off, as before 23 Nov)
  float driveLeanDeg = 1.5F; // extra lean while moving, for rolling drag:
                             // DRIVE_LEAN * tanh(v_ref / 0.05 m/s)
};

struct Status
{
  State state = State::Off;
  Reason reason = Reason::None;   // why it last stopped
  bool dry = false;
  float thetaDeg = 0.0F;          // latest pitch
  float errorDeg = 0.0F;          // theta - trim
  float rateDps = 0.0F;           // theta_dot
  float u = 0.0F;                 // m/s^2
  float v = 0.0F;                 // m/s
  float x = 0.0F;                 // m
  float trimEstDeg = 0.0F;        // trim in use (automatic trim)
  float vRef = 0.0F;              // m/s, drive set point (ramped)
  float wRefDps = 0.0F;           // deg/s
  float xRef = 0.0F;              // m
  float scale = 0.0F;             // soft-start factor 0..1
  uint32_t runMs = 0;             // time since engaging
  uint32_t engages = 0;           // successful engagements since boot
  uint32_t trips = 0;
  float maxAbsU = 0.0F;           // since engaging
  float maxAbsErrorDeg = 0.0F;
  uint32_t stepRateSps = 0;       // current step rate (left wheel)
};

// ---- core 0 (setup) ----
void attach(Stepper& left, Stepper& right);

// ---- core 1 (called from the IMU code) ----
// One control update per IMU sample. Returns the commanded wheel acceleration
// (mm/s^2) and speed (steps/s) for telemetry, and sets *flags bits.
struct StepOutput
{
  float uMmS2 = 0.0F;
  int32_t vLeftSps = 0;
  int32_t vRightSps = 0;
  uint8_t flags = 0;    // FLAG_ACTIVE / FLAG_DRY
};
StepOutput step(uint32_t tUs, float pitchDeg, float rollDeg, float qDps,
                float rDps, bool attitudeValid, bool sampleBad);
void poll(uint32_t nowUs);      // every core 1 loop pass: IMU-stale watchdog
float accelCompensationG();     // for the attitude filter (0 unless enabled)
bool active();                  // anything but Off, or an arm request pending
State currentState();           // latest state (cheap, no lock)

constexpr uint8_t FLAG_ACTIVE = 0x20;  // telemetry flag: balance engaged
constexpr uint8_t FLAG_DRY = 0x40;     // telemetry flag: dry run

// ---- core 0 API ----
// arm(): request arming. stepsPerMetre from the current microstepping and
// wheel diameter. dry = run the controller without the motors.
bool arm(bool dry, float stepsPerMetre);   // false unless Off
void stop(Reason reason);                  // any state -> Tripped/Off
bool setParams(const Params& params);      // false if a value is invalid
// Drive set points while balancing (from SET_VELOCITY). Returns false if out
// of range. They fall back to zero when the lease expires.
bool setDrive(float vMps, float wDps, uint32_t leaseMs);
constexpr float MAX_DRIVE_MPS = 0.3F;
constexpr float MAX_DRIVE_DPS = 90.0F;
Params params();
Status status();
const char* stateName(State state);
const char* reasonName(Reason reason);
}  // namespace Balance
