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
};
