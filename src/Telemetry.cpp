#include "Telemetry.h"

#include <cmath>
#include <cstdio>

#include "Balance.h"
#include "Imu.h"
#include "Protocol.h"

namespace Telemetry
{
namespace
{
// Longest line: "R2W/1 LOG " + 16 numbers. Keep a margin for the BBEND line.
constexpr size_t LINE_SIZE = 160;
// Samples drained per update(), so one pass can never take long.
constexpr int MAX_DRAIN = 64;

long scaled(float value, float scale)
{
  return static_cast<long>(lroundf(value * scale));
}
}  // namespace

void Logger::update(const Robot& robot, Print& out)
{
  int32_t vLeft = 0;
  int32_t vRight = 0;
  robot.wheelSpeeds(vLeft, vRight);
  const uint8_t mode = static_cast<uint8_t>(robot.mode());
  const bool busy = robot.motorsBusy();

  Imu::Sample sample;
  for (int i = 0; i < MAX_DRAIN && Imu::popSample(sample); ++i) {
    Record& record = live_[seq_ % LIVE_SIZE];
    record.seq = sample.n;
    record.tUs = sample.tUs;
    record.roll = sample.roll;
    record.pitch = sample.pitch;
    record.yaw = sample.yaw;
    record.p = sample.p;
    record.q = sample.q;
    record.r = sample.r;
    record.ax = sample.ax;
    record.ay = sample.ay;
    record.az = sample.az;
    record.u = sample.uMmS2;
    if (sample.flags & Balance::FLAG_ACTIVE) {
      record.vLeft = sample.vSps;   // commanded by the balance controller
      record.vRight = sample.vRightSps;
      record.mode = static_cast<uint8_t>(Robot::Mode::Balance);
    } else {
      record.vLeft = vLeft;
      record.vRight = vRight;
      record.mode = mode;
    }
    record.flags = sample.flags;
    if (!(sample.flags & Balance::FLAG_ACTIVE) && busy) record.flags |= FLAG_MOTORS_BUSY;
    lastTUs_ = sample.tUs;
    lastSampleN_ = sample.n;
    ++seq_;

    if (bbState_ == BlackBoxState::Recording) {
      bb_[bbHead_ % BB_SIZE] = record;
      ++bbHead_;
      if (bbCount_ < BB_SIZE) ++bbCount_;
    }
  }

  if (logging_) stream(out);
  if (bbState_ == BlackBoxState::Dumping) dump(out);
}

size_t Logger::format(char* buffer, size_t size, const char* kind,
                      const Record& r)
{
  const int n = snprintf(
      buffer, size,
      "%s %s %lu %lu %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %u %u\n",
      Protocol::VERSION, kind, static_cast<unsigned long>(r.seq),
      static_cast<unsigned long>(r.tUs), scaled(r.pitch, 100.0F),
      scaled(r.roll, 100.0F), scaled(r.yaw, 100.0F), scaled(r.p, 100.0F),
      scaled(r.q, 100.0F), scaled(r.r, 100.0F), scaled(r.ax, 1000.0F),
      scaled(r.ay, 1000.0F), scaled(r.az, 1000.0F), scaled(r.u, 1.0F),
      static_cast<long>(r.vLeft), static_cast<long>(r.vRight),
      static_cast<unsigned>(r.mode), static_cast<unsigned>(r.flags));
  if (n <= 0) return 0;
  return static_cast<size_t>(n) < size ? static_cast<size_t>(n) : size - 1;
}

void Logger::stream(Print& out)
{
  char line[LINE_SIZE];
  while (streamNext_ != seq_) {
    // Fell more than one live ring behind: skip what has been overwritten.
    if (seq_ - streamNext_ > LIVE_SIZE) {
      const uint32_t skip = seq_ - streamNext_ - LIVE_SIZE;
      streamDrops_ += (skip + divisor_ - 1) / divisor_;
      streamNext_ = seq_ - LIVE_SIZE;
    }
    if (streamNext_ % divisor_ != 0) {
      ++streamNext_;
      continue;
    }
    const size_t length =
        format(line, sizeof(line), "LOG", live_[streamNext_ % LIVE_SIZE]);
    if (out.availableForWrite() < static_cast<int>(length)) return;
    out.write(reinterpret_cast<const uint8_t*>(line), length);
    ++sent_;
    ++streamNext_;
  }
}

void Logger::dump(Print& out)
{
  char line[LINE_SIZE];
  const uint32_t start = bbHead_ - bbCount_;
  while (bbDumped_ < bbCount_) {
    const size_t length = format(line, sizeof(line), "BB",
                                 bb_[(start + bbDumped_) % BB_SIZE]);
    if (out.availableForWrite() < static_cast<int>(length)) return;
    out.write(reinterpret_cast<const uint8_t*>(line), length);
    ++bbDumped_;
  }
  if (bbEndPending_) {
    const int n = snprintf(line, sizeof(line),
                           "%s BBEND COUNT=%lu REASON=%s FROZEN_T_US=%lu\n",
                           Protocol::VERSION,
                           static_cast<unsigned long>(bbCount_), bbReason_,
                           static_cast<unsigned long>(bbFrozenAtUs_));
    if (n <= 0 || out.availableForWrite() < n) return;
    out.write(reinterpret_cast<const uint8_t*>(line), static_cast<size_t>(n));
    bbEndPending_ = false;
  }
  bbState_ = BlackBoxState::Frozen;
}

bool Logger::startLog(uint32_t divisor)
{
  if (divisor == 0 || divisor > MAX_DIVISOR) return false;
  divisor_ = divisor;
  // Start with the next record that matches the divisor.
  streamNext_ = seq_;
  sent_ = 0;
  streamDrops_ = 0;
  imuDropsBase_ = Imu::sampleQueueDrops();
  logging_ = true;
  return true;
}

void Logger::stopLog() { logging_ = false; }

void Logger::freeze(const char* reason)
{
  if (bbState_ != BlackBoxState::Recording) return;
  bbState_ = BlackBoxState::Frozen;
  bbReason_ = reason;
  bbFrozenAtUs_ = lastTUs_;
}

bool Logger::arm()
{
  if (bbState_ == BlackBoxState::Dumping) return false;
  bbState_ = BlackBoxState::Recording;
  bbHead_ = 0;
  bbCount_ = 0;
  bbDumped_ = 0;
  bbReason_ = "";
  bbFrozenAtUs_ = 0;
  return true;
}

bool Logger::startDump()
{
  if (bbState_ != BlackBoxState::Frozen || logging_) return false;
  bbDumped_ = 0;
  bbEndPending_ = true;
  bbState_ = BlackBoxState::Dumping;
  return true;
}

Stats Logger::stats() const
{
  Stats s;
  s.logging = logging_;
  s.divisor = divisor_;
  s.seq = lastSampleN_;
  s.sent = sent_;
  s.streamDrops = streamDrops_;
  s.imuDropsTotal = Imu::sampleQueueDrops();
  s.imuDrops = s.imuDropsTotal - imuDropsBase_;
  s.bbState = bbState_;
  s.bbCount = bbCount_;
  s.bbDumped = bbDumped_;
  s.bbReason = bbReason_;
  s.bbFrozenAtUs = bbFrozenAtUs_;
  return s;
}

const char* Logger::bbStateName(BlackBoxState state)
{
  switch (state) {
    case BlackBoxState::Recording: return "RECORDING";
    case BlackBoxState::Frozen: return "FROZEN";
    case BlackBoxState::Dumping: return "DUMPING";
  }
  return "UNKNOWN";
}
}  // namespace Telemetry
