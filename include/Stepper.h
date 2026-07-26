#pragma once

#include <Arduino.h>

// A low-level stepper interface. This version configures STEP, DIR, and
// enable pins but deliberately does not generate step pulses.
class Stepper
{
public:
  Stepper(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin);

  void begin();
  void enable();
  void disable();

  bool isEnabled() const;

private:
  uint8_t stepPin_;
  uint8_t directionPin_;
  uint8_t enablePin_;
  bool enabled_ = false;
};
