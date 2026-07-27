#pragma once

#include <Arduino.h>

#include "Motor.h"
#include "SerialTransport.h"
#include "TMC2209.h"

class RobotLink
{
public:
  using LegacyCommandHandler = void (*)(const char* command);

  RobotLink(Stream& stream, Motor& leftMotor, Motor& rightMotor,
            TMC2209& leftDriver, TMC2209& rightDriver,
            LegacyCommandHandler legacyCommandHandler);

  void update();

private:
  void processLine(char* line);
  void sendAck(uint32_t sequence);
  void sendError(uint32_t sequence, const char* code);
  void sendPong(uint32_t sequence);
  void sendStatus(uint32_t sequence);
  void printDriverStatus(TMC2209& driver, const Motor& motor);

  SerialTransport transport_;
  Motor& leftMotor_;
  Motor& rightMotor_;
  TMC2209& leftDriver_;
  TMC2209& rightDriver_;
  LegacyCommandHandler legacyCommandHandler_;
  uint32_t lastValidCommandMs_ = 0;
};
