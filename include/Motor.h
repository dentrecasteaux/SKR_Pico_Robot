#pragma once

#include <Arduino.h>

#include "Stepper.h"

class Motor
{
public:
  explicit Motor(Stepper& stepper);

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

private:
  enum class Mode { Idle, PulseTrain, Continuous };

  Stepper& stepper_;
  Mode mode_ = Mode::Idle;
  bool pulseHigh_ = false;
  uint32_t remainingSteps_ = 0;
  uint32_t stepIntervalUs_ = 0;
  uint32_t lastTransitionUs_ = 0;
  int32_t speed_ = 0;
};
