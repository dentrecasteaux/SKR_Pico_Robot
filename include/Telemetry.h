#pragma once

#include <Arduino.h>

#include "Robot.h"

// Telemetry for the balancing work (commissioning plan Phase 6).
//
// Core 0 drains every IMU sample from core 1's queue (500 Hz), adds the robot
// state and keeps it in two RAM rings:
//
// - a 1 s live ring that LOG_START streams to the Pi as "R2W/1 LOG ..." lines,
//   every DIV-th sample. Lines are written only when the USB transmit buffer
//   has room, so a slow reader never blocks the firmware: the stream falls
//   behind by up to 1 s, after which records are counted as dropped.
// - a 2 s "black box" ring that records continuously until it is frozen by
//   BB_FREEZE, an ESTOP or (later) a balance trip. BB_DUMP then sends the
//   frozen contents as "R2W/1 BB ..." lines, again without blocking, followed
//   by one "R2W/1 BBEND ..." line. BB_ARM resumes recording.
//
// LOG and BB lines share one positional format (see docs/protocol.md):
//   seq t_us pitch roll yaw p q r ax ay az u v_left v_right mode flags
// Angles are centidegrees, rates centidegrees/s, accelerations milli-g,
// u (commanded wheel acceleration, Phase 9; 0 until then) mm/s², wheel speeds
// steps/s.
namespace Telemetry
{
struct Record
{
  uint32_t seq = 0;      // core 1 sample number since boot: a gap means
                         // samples were lost in the core 1 -> core 0 queue
  uint32_t tUs = 0;      // micros() of the IMU data-ready interrupt
  float roll = 0.0F;     // degrees
  float pitch = 0.0F;
  float yaw = 0.0F;
  float p = 0.0F;        // deg/s
  float q = 0.0F;
  float r = 0.0F;
  float ax = 0.0F;       // g
  float ay = 0.0F;
  float az = 0.0F;
  float u = 0.0F;        // commanded wheel acceleration, mm/s² (Phase 9)
  int32_t vLeft = 0;     // steps/s, forward positive
  int32_t vRight = 0;
  uint8_t mode = 0;      // Robot::Mode
  uint8_t flags = 0;     // Imu::SAMPLE_* bits, plus FLAG_MOTORS_BUSY
};

constexpr uint8_t FLAG_MOTORS_BUSY = 0x80;

enum class BlackBoxState : uint8_t { Recording, Frozen, Dumping };

struct Stats
{
  bool logging = false;
  uint32_t divisor = 0;
  uint32_t seq = 0;             // latest record number
  uint32_t sent = 0;            // LOG lines sent since LOG_START
  uint32_t streamDrops = 0;     // LOG records skipped because the reader lagged
  uint32_t imuDrops = 0;        // queue drops since LOG_START
  uint32_t imuDropsTotal = 0;   // queue drops since boot (includes start-up,
                                // while core 0 waits for the USB host)
  BlackBoxState bbState = BlackBoxState::Recording;
  uint32_t bbCount = 0;         // records held (up to BB_SIZE)
  uint32_t bbDumped = 0;        // records sent by the current/last dump
  const char* bbReason = "";    // why it was frozen
  uint32_t bbFrozenAtUs = 0;    // t_us of the last record before the freeze
};

constexpr uint32_t LIVE_SIZE = 512;   // 1.02 s at 500 Hz
constexpr uint32_t BB_SIZE = 1024;    // 2.05 s at 500 Hz
constexpr uint32_t MAX_DIVISOR = 50;  // 10 Hz

class Logger
{
public:
  // Call every core 0 loop pass. Drains the IMU queue, then writes as many
  // LOG / BB lines as the output can take without blocking.
  void update(const Robot& robot, Print& out);

  bool startLog(uint32_t divisor);  // false if divisor is 0 or > MAX_DIVISOR
  void stopLog();
  void freeze(const char* reason);  // no effect unless recording
  bool arm();                       // false while a dump is in progress
  bool startDump();                 // false unless frozen and not logging
  Stats stats() const;

  static const char* bbStateName(BlackBoxState state);

private:
  static size_t format(char* buffer, size_t size, const char* kind,
                       const Record& record);
  void stream(Print& out);
  void dump(Print& out);

  Record live_[LIVE_SIZE];
  Record bb_[BB_SIZE];
  uint32_t seq_ = 0;          // records produced; live_[n % LIVE_SIZE]
  uint32_t lastSampleN_ = 0;  // seq of the latest record
  uint32_t imuDropsBase_ = 0; // Imu::sampleQueueDrops() at LOG_START
  uint32_t streamNext_ = 0;   // next record number to stream
  bool logging_ = false;
  uint32_t divisor_ = 2;
  uint32_t sent_ = 0;
  uint32_t streamDrops_ = 0;

  BlackBoxState bbState_ = BlackBoxState::Recording;
  uint32_t bbHead_ = 0;       // records written to bb_ (mod BB_SIZE)
  uint32_t bbCount_ = 0;
  uint32_t bbDumped_ = 0;
  bool bbEndPending_ = false;
  const char* bbReason_ = "";
  uint32_t bbFrozenAtUs_ = 0;
  uint32_t lastTUs_ = 0;
};
}  // namespace Telemetry
