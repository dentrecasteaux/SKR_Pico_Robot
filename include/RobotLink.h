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
  void sendAck(uint32_t sequence);
  void sendError(uint32_t sequence, const char* code);
  void sendPong(uint32_t sequence);
  void sendStatus(uint32_t sequence);
  void printDriverStatus(const Robot::DriverStatus& status);

  SerialTransport transport_;
  Robot& robot_;
  LegacyCommandHandler legacyCommandHandler_;
  uint32_t lastValidCommandMs_ = 0;
};
