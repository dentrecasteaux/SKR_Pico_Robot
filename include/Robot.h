#pragma once

#include <Arduino.h>

#include "MotionController.h"
#include "Motor.h"
#include "TMC2209.h"

class Robot
{
public:
  enum class Mode
  {
    Idle,
    Velocity,
    Move,
    Turn
  };

  struct DriverStatus
  {
    bool connected = false;
    bool active = false;
    uint32_t flags = 0;
  };

  struct Status
  {
    Mode mode = Mode::Idle;
    bool estopLatched = false;
    uint32_t activeJob = 0;
    float linearSetpointMmPerSecond = 0.0F;
    float turnSetpointDegreesPerSecond = 0.0F;
    DriverStatus leftDriver;
    DriverStatus rightDriver;
  };

  Robot(Motor& leftMotor, Motor& rightMotor,
        MotionController& motionController, TMC2209& leftDriver,
        TMC2209& rightDriver, arduino::UART& stepperUart);

  void begin();
  void update();

  bool setVelocity(float linearMmPerSecond, float turnDegreesPerSecond);
  bool startMove(float distanceMm);
  bool startTurn(float angleDegrees);
  void stop();
  Status status();

private:
  DriverStatus driverStatus(TMC2209& driver, const Motor& motor);

  Motor& leftMotor_;
  Motor& rightMotor_;
  MotionController& motionController_;
  TMC2209& leftDriver_;
  TMC2209& rightDriver_;
  arduino::UART& stepperUart_;
  Mode mode_ = Mode::Idle;
  float linearSetpointMmPerSecond_ = 0.0F;
  float turnSetpointDegreesPerSecond_ = 0.0F;
};
