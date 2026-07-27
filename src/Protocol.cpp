#include "Protocol.h"

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace
{
bool isProtocolPrefix(const char* token)
{
  return std::strncmp(token, "R2W/", 4) == 0;
}

bool parseUnsigned(const char* text, uint32_t& value)
{
  if (text[0] < '0' || text[0] > '9') return false;

  errno = 0;
  char* end = nullptr;
  const unsigned long parsed = std::strtoul(text, &end, 10);
  if (errno == ERANGE || end == text || *end != '\0' ||
      parsed > std::numeric_limits<uint32_t>::max()) {
    return false;
  }
  value = static_cast<uint32_t>(parsed);
  return true;
}

bool parseFloat(const char* text, float& value)
{
  errno = 0;
  char* end = nullptr;
  const float parsed = std::strtof(text, &end);
  if (errno == ERANGE || end == text || *end != '\0' ||
      !std::isfinite(parsed)) {
    return false;
  }
  value = parsed;
  return true;
}

Protocol::CommandType commandType(const char* name, bool& recognised)
{
  recognised = true;
  if (std::strcmp(name, "PING") == 0) return Protocol::CommandType::Ping;
  if (std::strcmp(name, "STATUS") == 0) return Protocol::CommandType::Status;
  if (std::strcmp(name, "SET_VELOCITY") == 0) {
    return Protocol::CommandType::SetVelocity;
  }
  if (std::strcmp(name, "MOVE") == 0) return Protocol::CommandType::Move;
  if (std::strcmp(name, "TURN") == 0) return Protocol::CommandType::Turn;
  if (std::strcmp(name, "STOP") == 0) return Protocol::CommandType::Stop;
  if (std::strcmp(name, "ESTOP") == 0) return Protocol::CommandType::Estop;
  if (std::strcmp(name, "CLEAR_ESTOP") == 0) {
    return Protocol::CommandType::ClearEstop;
  }

  recognised = false;
  return Protocol::CommandType::Ping;
}
}  // namespace

Protocol::ParseResult Protocol::parseCommand(char* line, Command& command)
{
  ParseResult result;
  char* savePosition = nullptr;

  char* token = ::strtok_r(line, " ", &savePosition);
  if (token == nullptr || std::strcmp(token, VERSION) != 0) {
    result.errorCode =
        token != nullptr && isProtocolPrefix(token) ? "BAD_VERSION" : "BAD_FRAME";
    return result;
  }

  token = ::strtok_r(nullptr, " ", &savePosition);
  if (token == nullptr || std::strcmp(token, "CMD") != 0) {
    result.errorCode = "BAD_FRAME";
    return result;
  }

  token = ::strtok_r(nullptr, " ", &savePosition);
  if (token == nullptr || !parseUnsigned(token, result.sequence)) {
    result.errorCode = "BAD_SEQUENCE";
    return result;
  }

  token = ::strtok_r(nullptr, " ", &savePosition);
  if (token == nullptr) {
    result.errorCode = "BAD_FRAME";
    return result;
  }

  bool recognised = false;
  command.type = commandType(token, recognised);
  if (!recognised) {
    result.errorCode = "UNKNOWN_COMMAND";
    return result;
  }

  bool hasLinear = false;
  bool hasTurn = false;
  bool hasLease = false;
  bool hasDistance = false;
  bool hasAngle = false;

  while ((token = ::strtok_r(nullptr, " ", &savePosition)) != nullptr) {
    char* equals = std::strchr(token, '=');
    if (equals == nullptr || equals == token || equals[1] == '\0') {
      result.errorCode = "INVALID_VALUE";
      return result;
    }
    *equals = '\0';
    const char* value = equals + 1;

    if (std::strcmp(token, "V") == 0) {
      if (hasLinear) {
        result.errorCode = "DUPLICATE_FIELD";
        return result;
      }
      hasLinear = true;
      if (!parseFloat(value, command.linearMmPerSecond)) {
        result.errorCode = "INVALID_VALUE";
        return result;
      }
    } else if (std::strcmp(token, "W") == 0) {
      if (hasTurn) {
        result.errorCode = "DUPLICATE_FIELD";
        return result;
      }
      hasTurn = true;
      if (!parseFloat(value, command.turnDegreesPerSecond)) {
        result.errorCode = "INVALID_VALUE";
        return result;
      }
    } else if (std::strcmp(token, "LEASE") == 0) {
      if (hasLease) {
        result.errorCode = "DUPLICATE_FIELD";
        return result;
      }
      hasLease = true;
      if (!parseUnsigned(value, command.leaseMs)) {
        result.errorCode = "INVALID_VALUE";
        return result;
      }
    } else if (std::strcmp(token, "DIST") == 0) {
      if (hasDistance) {
        result.errorCode = "DUPLICATE_FIELD";
        return result;
      }
      hasDistance = true;
      if (!parseFloat(value, command.distanceMm)) {
        result.errorCode = "INVALID_VALUE";
        return result;
      }
    } else if (std::strcmp(token, "ANGLE") == 0) {
      if (hasAngle) {
        result.errorCode = "DUPLICATE_FIELD";
        return result;
      }
      hasAngle = true;
      if (!parseFloat(value, command.angleDegrees)) {
        result.errorCode = "INVALID_VALUE";
        return result;
      }
    } else {
      result.errorCode = "UNKNOWN_FIELD";
      return result;
    }
  }

  const bool noFields =
      !hasLinear && !hasTurn && !hasLease && !hasDistance && !hasAngle;
  switch (command.type) {
    case CommandType::Ping:
    case CommandType::Status:
    case CommandType::Stop:
    case CommandType::Estop:
    case CommandType::ClearEstop:
      if (!noFields) {
        result.errorCode = "UNKNOWN_FIELD";
        return result;
      }
      break;
    case CommandType::SetVelocity:
      if (!hasLinear || !hasTurn || !hasLease) {
        result.errorCode = "MISSING_FIELD";
        return result;
      }
      if (hasDistance || hasAngle) {
        result.errorCode = "UNKNOWN_FIELD";
        return result;
      }
      break;
    case CommandType::Move:
      if (!hasDistance) {
        result.errorCode = "MISSING_FIELD";
        return result;
      }
      if (hasLinear || hasTurn || hasLease || hasAngle) {
        result.errorCode = "UNKNOWN_FIELD";
        return result;
      }
      break;
    case CommandType::Turn:
      if (!hasAngle) {
        result.errorCode = "MISSING_FIELD";
        return result;
      }
      if (hasLinear || hasTurn || hasLease || hasDistance) {
        result.errorCode = "UNKNOWN_FIELD";
        return result;
      }
      break;
  }

  command.sequence = result.sequence;
  result.valid = true;
  return result;
}
