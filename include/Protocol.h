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
  ClearEstop
};

struct Command
{
  uint32_t sequence = 0;
  CommandType type = CommandType::Ping;
  float linearMmPerSecond = 0.0F;
  float turnDegreesPerSecond = 0.0F;
  uint32_t leaseMs = 0;
  float distanceMm = 0.0F;
  float angleDegrees = 0.0F;
};

struct ParseResult
{
  bool valid = false;
  uint32_t sequence = 0;
  const char* errorCode = nullptr;
};

ParseResult parseCommand(char* line, Command& command);
}  // namespace Protocol
