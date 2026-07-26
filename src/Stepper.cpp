#include "Stepper.h"

Stepper::Stepper(uint8_t enablePin)
    : enablePin_(enablePin)
{
}

void Stepper::begin()
{
  pinMode(enablePin_, OUTPUT);
  disable();
}

void Stepper::enable()
{
  // The TMC2209 enable input on the SKR Pico is active low.
  digitalWrite(enablePin_, LOW);
  enabled_ = true;
}

void Stepper::disable()
{
  digitalWrite(enablePin_, HIGH);
  enabled_ = false;
}

bool Stepper::isEnabled() const
{
  return enabled_;
}
