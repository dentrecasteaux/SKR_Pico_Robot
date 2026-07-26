#pragma once

#include <Arduino.h>

#include "Motor.h"

class MotionController
{
public:
  MotionController(Motor& leftMotor, Motor& rightMotor);

  void setDrive(int32_t linearSpeed, int32_t turnSpeed);
  void setVelocity(float linearMmPerSecond, float turnDegreesPerSecond);
  void stop();

private:
  Motor& leftMotor_;
  Motor& rightMotor_;
};
