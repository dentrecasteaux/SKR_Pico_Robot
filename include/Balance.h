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
// Kv and Kx take the same sign convention as an LQR gain row for state
// [x, v, theta, theta_dot], where the position and speed gains are normally
// positive (the base moves *further* first). Defaults: the "everyday" set
// below. Full description: docs/engineering-reference.md section 4.2.
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
  // History: Phase 9 PD KP 39 KD 2.9 KV 0 KX 0; then full-state pole
  // placement 36.7 / 4.19 / 4.81 / 2.94 (l_eff 0.20 m design) until 24 Nov.
  // "Everyday" gains (Phase 11 step 2, 24 Nov): with ACCEL_COMP=2, tau 0.5
  // and the 20.5 Hz notch, kick ring 2.3 Hz zeta 0.43 and clean driving to
  // 0.38 m/s. Before 24 Nov: 36.7 / 4.19 / 4.81 / 2.94 (pole placement).
  float kp = 51.4F;          // m/s^2 per rad
  float kd = 4.37F;          // m/s^2 per rad/s
  float kv = 6.89F;          // m/s^2 per m/s
  float kx = 6.48F;          // m/s^2 per m
  float trimDeg = 1.9F;      // IMU pitch at the balance point (IMU wedged
                             // under the top plate, 23 Nov; was -3.9)
  float aMax = 6.0F;         // m/s^2
  float vMax = 0.55F;        // m/s
  float softStartS = 0.3F;
  float tiltFwdDeg = 10.7F;  // absolute IMU pitch limits, 2 deg inside the
  float tiltBackDeg = -14.7F; // training-wheel stops (+12.72 / -16.68)
  float armWindowDeg = 2.0F;
  float armRateDps = 20.0F;
  float armTimeoutS = 20.0F;
  float runawayS = 0.3F;
  uint8_t accelComp = 2;     // accelerometer X-axis compensation while
                             // balancing: 0 off; 1 subtract u/g (23 Nov,
                             // better push recovery, but wrong at 2-5 Hz);
                             // 2 subtract the IMU's own acceleration,
                             // u delayed by compDelayS + imuHeightM * dq/dt
                             // (24 Nov, phase11-model.md)
  float imuHeightM = 0.098F; // IMU above the axle centreline (measured)
  float compDelayS = 0.010F; // command -> wheel acceleration delay (model fit)
  float notchHz = 20.5F;     // notch on the feedback part of u; 0 = off.
                             // For the ~20 Hz drive shake at speed (24 Nov)
  float notchQ = 2.0F;       // notch quality factor (width = notchHz / Q)
  float trimRate = 0.25F;    // deg per (m*s); 0 = no automatic trim.
                             // Adapts only while calm (see Balance.cpp).
  float trimRangeDeg = 4.0F; // trim_est stays within trim +/- this
  float driveAccel = 0.8F;   // m/s^2 ramp for v_ref (jerk-limited S-curve)
  float driveFf = 1.0F;      // drive feed-forward: lean a_ref/g and add
                             // a_ref to u (0 = off, as before 23 Nov)
  float driveLeanDeg = 2.0F; // extra lean while moving, for rolling drag:
                             // DRIVE_LEAN * tanh(v_ref / 0.05 m/s)
  // Profiled moves while balancing (MOVE / TURN, 24 Nov): cruise speeds.
  float moveSpeed = 0.2F;    // m/s (MOVE_V)
  float turnRateDps = 60.0F; // deg/s (TURN_W)
  // Position integral (KI, 24 Nov; 0 = off): u += KI * integral(x - x_ref) dt,
  // only while not driving forwards or backwards (standing, turning), reset
  // when a drive starts, |KI * integral| <= KI_LIMIT. Pulls the robot out of
  // the tyre contact band back to x_ref. Same sign convention as KX.
  float ki = 0.0F;           // m/s^2 per (m*s)
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
  uint8_t moving = 0;             // profiled move: bit 0 MOVE, bit 1 TURN
  float headingDeg = 0.0F;        // integrated IMU yaw rate since engaging
  float xInt = 0.0F;              // position integral term KI*int (m/s^2)
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
constexpr uint8_t FLAG_KICK = 0x80;    // with FLAG_ACTIVE: a kick is applied
                                       // (0x80 means "motors busy" otherwise)

// ---- core 0 API ----
// arm(): request arming. stepsPerMetre from the current microstepping and
// wheel diameter. dry = run the controller without the motors.
bool arm(bool dry, float stepsPerMetre);   // false unless Off
void stop(Reason reason);                  // any state -> Tripped/Off
bool setParams(const Params& params);      // false if a value is invalid
// Drive set points while balancing (from SET_VELOCITY). Returns false if out
// of range. They fall back to zero when the lease expires.
bool setDrive(float vMps, float wDps, uint32_t leaseMs);
// Profiled moves while Running (MOVE DIST= / TURN ANGLE= while balancing).
// The firmware ramps up to MOVE_V / TURN_W and brakes onto the target:
// distance on x_ref, angle on the integrated IMU yaw rate (real rotation).
// Any drive command (SET_VELOCITY, STOP) cancels them. False unless Running
// or out of range (|dist| <= 2 m, |angle| <= 1080 deg).
bool moveBy(float distM);
bool turnBy(float angleDeg);

// Identification kick (Phase 9.4): add uMps2 to the controller's output for
// durationMs, while Running. |u| <= 4 m/s^2, 10-200 ms. False if not running.
bool kick(float uMps2, uint32_t durationMs);
constexpr float MAX_DRIVE_MPS = 0.3F;
constexpr float MAX_DRIVE_DPS = 90.0F;
Params params();
Status status();
const char* stateName(State state);
const char* reasonName(Reason reason);
}  // namespace Balance
