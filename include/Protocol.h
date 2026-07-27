#pragma once

#include <Arduino.h>

namespace Protocol
{
constexpr const char* VERSION = "R2W/1";

enum class CommandType
{
  Ping,
  Status
};

struct Command
{
  uint32_t sequence = 0;
  CommandType type = CommandType::Ping;
};

struct ParseResult
{
  bool valid = false;
  uint32_t sequence = 0;
  const char* errorCode = nullptr;
};

ParseResult parseCommand(char* line, Command& command);
}  // namespace Protocol
