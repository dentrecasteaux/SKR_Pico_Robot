#pragma once

#include <Arduino.h>

#include "Stepper.h"

class Motor
{
public:
  Motor(Stepper& stepper, uint32_t accelerationStepsPerSecondSquared);

  void begin();
  void enable();
  void disable();

  bool startTest(uint32_t steps, uint32_t stepIntervalUs);
  bool startMove(uint32_t steps, bool forward, uint32_t stepIntervalUs);
  void setSpeed(int32_t stepsPerSecond);
  void update(uint32_t nowUs);

  bool isEnabled() const;
  bool isBusy() const;
  int32_t speed() const;
  uint32_t stepFrequencyHz() const;

private:
  enum class Mode { Idle, PulseTrain, Continuous };

  void updateSpeed(uint32_t nowUs);
  void updatePulseFrequency();

  Stepper& stepper_;
  uint32_t accelerationStepsPerSecondSquared_;
  Mode mode_ = Mode::Idle;
  uint32_t stepIntervalUs_ = 0;
  uint32_t lastSpeedUpdateUs_ = 0;
  uint32_t programmedFrequencyHz_ = 0;
  int32_t targetSpeed_ = 0;
  float currentSpeed_ = 0.0F;
};
