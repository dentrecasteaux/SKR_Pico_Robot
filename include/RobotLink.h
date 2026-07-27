#pragma once

#include <Arduino.h>

#include "Robot.h"
#include "SerialTransport.h"

class RobotLink
{
public:
  using LegacyCommandHandler = void (*)(const char* command);

  RobotLink(Stream& stream, Robot& robot,
            LegacyCommandHandler legacyCommandHandler);

  void update();

private:
  void processLine(char* line);
  void sendAck(uint32_t sequence, uint32_t jobId = 0);
  void sendError(uint32_t sequence, const char* code);
  void sendCommandError(uint32_t sequence, Robot::CommandResult result);
  const char* commandErrorCode(Robot::CommandResult result) const;
  void cacheReply(const char* commandLine, uint32_t sequence, bool accepted,
                  uint32_t jobId = 0, const char* errorCode = nullptr);
  bool replayCachedReply(const char* commandLine, uint32_t sequence);
  void sendPendingCompletion();
  void sendDone(const Robot::JobCompletion& completion);
  void sendPong(uint32_t sequence);
  void sendStatus(uint32_t sequence);
  void printDriverStatus(const Robot::DriverStatus& status);
  const char* jobResultName(Robot::JobResult result) const;

  SerialTransport transport_;
  Robot& robot_;
  LegacyCommandHandler legacyCommandHandler_;
  uint32_t lastValidCommandMs_ = 0;

  static constexpr size_t CACHED_COMMAND_SIZE = 160;
  char cachedCommand_[CACHED_COMMAND_SIZE] = {};
  uint32_t cachedSequence_ = 0;
  uint32_t cachedJobId_ = 0;
  const char* cachedErrorCode_ = nullptr;
  bool cachedReplyValid_ = false;
  bool cachedReplyAccepted_ = false;
};
