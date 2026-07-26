#pragma once

#include <Arduino.h>

// A low-level stepper interface. This first version only controls the
// TMC2209 enable input; it deliberately does not generate step pulses.
class Stepper
{
public:
  explicit Stepper(uint8_t enablePin);

  void begin();
  void enable();
  void disable();

  bool isEnabled() const;

private:
  uint8_t enablePin_;
  bool enabled_ = false;
};
