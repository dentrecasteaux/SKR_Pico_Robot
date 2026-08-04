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
  Configure
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
};

struct ParseResult
{
  bool valid = false;
  uint32_t sequence = 0;
  const char* errorCode = nullptr;
};

ParseResult parseCommand(char* line, Command& command);
}  // namespace Protocol
