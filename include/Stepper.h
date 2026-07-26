#pragma once

#include <Arduino.h>

// A low-level stepper interface. This version configures STEP, DIR, and
// enable pins but deliberately does not generate step pulses.
class Stepper
{
public:
  Stepper(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin,
          bool directionInverted);

  void begin();
  void enable();
  void disable();

  bool startTest(uint32_t steps, uint32_t stepIntervalUs);
  void update(uint32_t nowUs);

  bool isEnabled() const;
  bool isBusy() const;

private:
  uint8_t stepPin_;
  uint8_t directionPin_;
  uint8_t enablePin_;
  bool directionInverted_;
  bool enabled_ = false;
  bool pulseHigh_ = false;
  uint32_t remainingSteps_ = 0;
  uint32_t stepIntervalUs_ = 0;
  uint32_t lastTransitionUs_ = 0;
};
