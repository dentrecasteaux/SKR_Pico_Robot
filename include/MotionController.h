#pragma once

#include <Arduino.h>

#include "Motor.h"

class MotionController
{
public:
  MotionController(Motor& leftMotor, Motor& rightMotor);

  void setDrive(int32_t linearSpeed, int32_t turnSpeed);
  void setVelocity(float linearMmPerSecond, float turnDegreesPerSecond);
  bool moveDistance(float distanceMm);
  bool turnAngle(float angleDegrees);
  void stop();
  void configureMotion(uint16_t microsteps,
                       float accelerationMmPerSecondSquared);
  uint16_t microsteps() const;
  float accelerationMmPerSecondSquared() const;

private:
  float stepsPerMillimetre() const;
  void setWheelSpeeds(int32_t left, int32_t right, int32_t maximumMagnitude);

  Motor& leftMotor_;
  Motor& rightMotor_;
  uint16_t microsteps_;
  float accelerationMmPerSecondSquared_;
};
