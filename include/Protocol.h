#pragma once

#include <Arduino.h>

namespace Protocol
{
constexpr const char* VERSION = "R2W/1";

enum class CommandType
{
  Ping,
  Status,
  SetVelocity,
  Move,
  Turn,
  Stop,
  Estop,
  ClearEstop,
  Configure,
  Imu,
  ImuCalibrate,
  ImuZeroYaw,
  ImuSet,
  ImuWindowStart,
  ImuWindow,
  LogStart,
  LogStop,
  LogStatus,
  BbFreeze,
  BbArm,
  BbDump,
  Hold,
  BalSet,
  BalArm,
  BalStop,
  BalStatus
};

// BAL_SET / BAL_ARM carry generic KEY=number pairs, checked by RobotLink.
constexpr size_t MAX_PAIRS = 16;
struct Pair
{
  const char* key = nullptr;   // points into the parsed line
  float value = 0.0F;
};

enum class ChopperMode { StealthChop, SpreadCycle };

struct Command
{
  uint32_t sequence = 0;
  CommandType type = CommandType::Ping;
  float linearMmPerSecond = 0.0F;
  float turnDegreesPerSecond = 0.0F;
  uint32_t leaseMs = 0;
  float distanceMm = 0.0F;
  float angleDegrees = 0.0F;
  uint32_t currentMa = 0;
  uint32_t microsteps = 0;
  float accelerationMmPerSecondSquared = 0.0F;
  ChopperMode chopperMode = ChopperMode::StealthChop;
  bool hasTau = false;
  float tauSeconds = 0.0F;
  bool hasAutoBias = false;
  bool autoBias = false;
  uint32_t logDivisor = 2;     // LOG_START DIV=, default 250 Hz
  Pair pairs[MAX_PAIRS];
  size_t pairCount = 0;
};

struct ParseResult
{
  bool valid = false;
  uint32_t sequence = 0;
  const char* errorCode = nullptr;
};

ParseResult parseCommand(char* line, Command& command);
}  // namespace Protocol
