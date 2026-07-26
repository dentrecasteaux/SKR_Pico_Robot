#include "Stepper.h"

Stepper::Stepper(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin)
    : stepPin_(stepPin),
      directionPin_(directionPin),
      enablePin_(enablePin)
{
}

void Stepper::begin()
{
  pinMode(stepPin_, OUTPUT);
  pinMode(directionPin_, OUTPUT);
  pinMode(enablePin_, OUTPUT);

  digitalWrite(stepPin_, LOW);
  digitalWrite(directionPin_, LOW);
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
