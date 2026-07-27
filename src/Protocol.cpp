#include "Protocol.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace
{
bool isProtocolPrefix(const char* token)
{
  return std::strncmp(token, "R2W/", 4) == 0;
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
  if (token == nullptr) {
    result.errorCode = "BAD_SEQUENCE";
    return result;
  }

  errno = 0;
  char* end = nullptr;
  const unsigned long sequence = std::strtoul(token, &end, 10);
  if (errno == ERANGE || end == token || *end != '\0' ||
      sequence > std::numeric_limits<uint32_t>::max()) {
    result.errorCode = "BAD_SEQUENCE";
    return result;
  }
  result.sequence = static_cast<uint32_t>(sequence);

  token = ::strtok_r(nullptr, " ", &savePosition);
  if (token == nullptr) {
    result.errorCode = "BAD_FRAME";
    return result;
  }

  if (std::strcmp(token, "PING") == 0) {
    command.type = CommandType::Ping;
  } else if (std::strcmp(token, "STATUS") == 0) {
    command.type = CommandType::Status;
  } else {
    result.errorCode = "UNKNOWN_COMMAND";
    return result;
  }

  if (::strtok_r(nullptr, " ", &savePosition) != nullptr) {
    result.errorCode = "UNKNOWN_FIELD";
    return result;
  }

  command.sequence = result.sequence;
  result.valid = true;
  return result;
}
