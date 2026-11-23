#include "Protocol.h"

#include "Config.h"

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
  if (std::strcmp(name, "CONFIGURE") == 0) {
    return Protocol::CommandType::Configure;
  }
  if (std::strcmp(name, "IMU") == 0) return Protocol::CommandType::Imu;
  if (std::strcmp(name, "IMU_CAL") == 0) {
    return Protocol::CommandType::ImuCalibrate;
  }
  if (std::strcmp(name, "IMU_ZERO_YAW") == 0) {
    return Protocol::CommandType::ImuZeroYaw;
  }
  if (std::strcmp(name, "IMU_SET") == 0) return Protocol::CommandType::ImuSet;
  if (std::strcmp(name, "IMU_WINDOW_START") == 0) {
    return Protocol::CommandType::ImuWindowStart;
  }
  if (std::strcmp(name, "LOG_START") == 0) return Protocol::CommandType::LogStart;
  if (std::strcmp(name, "LOG_STOP") == 0) return Protocol::CommandType::LogStop;
  if (std::strcmp(name, "LOG_STATUS") == 0) {
    return Protocol::CommandType::LogStatus;
  }
  if (std::strcmp(name, "BB_FREEZE") == 0) return Protocol::CommandType::BbFreeze;
  if (std::strcmp(name, "BB_ARM") == 0) return Protocol::CommandType::BbArm;
  if (std::strcmp(name, "BB_DUMP") == 0) return Protocol::CommandType::BbDump;
  if (std::strcmp(name, "HOLD") == 0) return Protocol::CommandType::Hold;
  if (std::strcmp(name, "BAL_SET") == 0) return Protocol::CommandType::BalSet;
  if (std::strcmp(name, "BAL_ARM") == 0) return Protocol::CommandType::BalArm;
  if (std::strcmp(name, "BAL_STOP") == 0) return Protocol::CommandType::BalStop;
  if (std::strcmp(name, "BAL_KICK") == 0) return Protocol::CommandType::BalKick;
  if (std::strcmp(name, "BAL_STATUS") == 0) {
    return Protocol::CommandType::BalStatus;
  }
  if (std::strcmp(name, "IMU_WINDOW") == 0) {
    return Protocol::CommandType::ImuWindow;
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

  // Balance commands: generic KEY=number pairs, validated by RobotLink.
  if (command.type == CommandType::BalSet ||
      command.type == CommandType::BalArm ||
      command.type == CommandType::BalStop ||
      command.type == CommandType::BalStatus ||
      command.type == CommandType::BalKick) {
    command.pairCount = 0;
    while ((token = ::strtok_r(nullptr, " ", &savePosition)) != nullptr) {
      char* equals = std::strchr(token, '=');
      if (equals == nullptr || equals == token || equals[1] == '\0') {
        result.errorCode = "INVALID_VALUE";
        return result;
      }
      *equals = '\0';
      if (command.pairCount >= MAX_PAIRS) {
        result.errorCode = "UNKNOWN_FIELD";
        return result;
      }
      Pair& pair = command.pairs[command.pairCount];
      pair.key = token;
      if (!parseFloat(equals + 1, pair.value)) {
        result.errorCode = "INVALID_VALUE";
        return result;
      }
      ++command.pairCount;
    }
    if ((command.type == CommandType::BalStop ||
         command.type == CommandType::BalStatus) &&
        command.pairCount != 0) {
      result.errorCode = "UNKNOWN_FIELD";
      return result;
    }
    command.sequence = result.sequence;
    result.valid = true;
    return result;
  }

  bool hasLinear = false;
  bool hasTurn = false;
  bool hasLease = false;
  bool hasDistance = false;
  bool hasAngle = false;
  bool hasCurrent = false;
  bool hasMicrosteps = false;
  bool hasAcceleration = false;
  bool hasMode = false;
  bool hasTau = false;
  bool hasAutoBias = false;
  bool hasDivisor = false;

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
    } else if (std::strcmp(token, "CURRENT_MA") == 0) {
      if (hasCurrent || !parseUnsigned(value, command.currentMa)) {
        result.errorCode = hasCurrent ? "DUPLICATE_FIELD" : "INVALID_VALUE";
        return result;
      }
      hasCurrent = true;
    } else if (std::strcmp(token, "MICROSTEPS") == 0) {
      if (hasMicrosteps || !parseUnsigned(value, command.microsteps)) {
        result.errorCode =
            hasMicrosteps ? "DUPLICATE_FIELD" : "INVALID_VALUE";
        return result;
      }
      hasMicrosteps = true;
    } else if (std::strcmp(token, "ACCEL_MM_S2") == 0) {
      if (hasAcceleration ||
          !parseFloat(value, command.accelerationMmPerSecondSquared)) {
        result.errorCode =
            hasAcceleration ? "DUPLICATE_FIELD" : "INVALID_VALUE";
        return result;
      }
      hasAcceleration = true;
    } else if (std::strcmp(token, "TMC_MODE") == 0) {
      if (hasMode) {
        result.errorCode = "DUPLICATE_FIELD";
        return result;
      }
      hasMode = true;
      if (std::strcmp(value, "STEALTHCHOP") == 0) {
        command.chopperMode = ChopperMode::StealthChop;
      } else if (std::strcmp(value, "SPREADCYCLE") == 0) {
        command.chopperMode = ChopperMode::SpreadCycle;
      } else {
        result.errorCode = "INVALID_VALUE";
        return result;
      }
    } else if (std::strcmp(token, "TAU") == 0) {
      if (hasTau || !parseFloat(value, command.tauSeconds)) {
        result.errorCode = hasTau ? "DUPLICATE_FIELD" : "INVALID_VALUE";
        return result;
      }
      hasTau = true;
    } else if (std::strcmp(token, "AUTO_BIAS") == 0) {
      uint32_t flag = 0;
      if (hasAutoBias || !parseUnsigned(value, flag) || flag > 1) {
        result.errorCode = hasAutoBias ? "DUPLICATE_FIELD" : "INVALID_VALUE";
        return result;
      }
      hasAutoBias = true;
      command.autoBias = flag == 1;
    } else if (std::strcmp(token, "DIV") == 0) {
      if (hasDivisor || !parseUnsigned(value, command.logDivisor)) {
        result.errorCode = hasDivisor ? "DUPLICATE_FIELD" : "INVALID_VALUE";
        return result;
      }
      hasDivisor = true;
    } else {
      result.errorCode = "UNKNOWN_FIELD";
      return result;
    }
  }
  command.hasTau = hasTau;
  command.hasAutoBias = hasAutoBias;

  const bool hasImuFields = hasTau || hasAutoBias;
  const bool noFields =
      !hasLinear && !hasTurn && !hasLease && !hasDistance && !hasAngle &&
      !hasCurrent && !hasMicrosteps && !hasAcceleration && !hasMode &&
      !hasImuFields && !hasDivisor;
  if (hasDivisor && command.type != CommandType::LogStart) {
    result.errorCode = "UNKNOWN_FIELD";
    return result;
  }
  const bool hasConfiguration =
      hasCurrent || hasMicrosteps || hasAcceleration || hasMode ||
      hasImuFields;
  switch (command.type) {
    case CommandType::Ping:
    case CommandType::Status:
    case CommandType::Stop:
    case CommandType::Estop:
    case CommandType::ClearEstop:
    case CommandType::Imu:
    case CommandType::ImuCalibrate:
    case CommandType::ImuZeroYaw:
    case CommandType::ImuWindowStart:
    case CommandType::ImuWindow:
    case CommandType::LogStop:
    case CommandType::LogStatus:
    case CommandType::BbFreeze:
    case CommandType::BbArm:
    case CommandType::BbDump:
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
      if (hasDistance || hasAngle || hasConfiguration) {
        result.errorCode = "UNKNOWN_FIELD";
        return result;
      }
      break;
    case CommandType::Move:
      if (!hasDistance) {
        result.errorCode = "MISSING_FIELD";
        return result;
      }
      if (hasLinear || hasTurn || hasLease || hasAngle || hasConfiguration) {
        result.errorCode = "UNKNOWN_FIELD";
        return result;
      }
      break;
    case CommandType::Turn:
      if (!hasAngle) {
        result.errorCode = "MISSING_FIELD";
        return result;
      }
      if (hasLinear || hasTurn || hasLease || hasDistance ||
          hasConfiguration) {
        result.errorCode = "UNKNOWN_FIELD";
        return result;
      }
      break;
    case CommandType::Configure:
      if (!hasCurrent || !hasMicrosteps || !hasAcceleration || !hasMode) {
        result.errorCode = "MISSING_FIELD";
        return result;
      }
      if (hasLinear || hasTurn || hasLease || hasDistance || hasAngle ||
          hasImuFields) {
        result.errorCode = "UNKNOWN_FIELD";
        return result;
      }
      break;
    case CommandType::Hold:
      if (hasLinear || hasTurn || hasDistance || hasAngle ||
          hasConfiguration) {
        result.errorCode = "UNKNOWN_FIELD";
        return result;
      }
      if (!hasLease) command.leaseMs = Config::DEFAULT_HOLD_LEASE_MS;
      break;
    case CommandType::LogStart:
      if (hasLinear || hasTurn || hasLease || hasDistance || hasAngle ||
          hasConfiguration) {
        result.errorCode = "UNKNOWN_FIELD";
        return result;
      }
      break;
    case CommandType::ImuSet:
      if (!hasImuFields) {
        result.errorCode = "MISSING_FIELD";
        return result;
      }
      if (hasLinear || hasTurn || hasLease || hasDistance || hasAngle ||
          hasCurrent || hasMicrosteps || hasAcceleration || hasMode) {
        result.errorCode = "UNKNOWN_FIELD";
        return result;
      }
      break;
  }

  command.sequence = result.sequence;
  result.valid = true;
  return result;
}
