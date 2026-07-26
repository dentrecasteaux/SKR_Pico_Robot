#include "Stepper.h"

Stepper::Stepper(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin,
                 bool directionInverted)
    : stepPin_(stepPin),
      directionPin_(directionPin),
      enablePin_(enablePin),
      directionInverted_(directionInverted)
{
}

void Stepper::begin()
{
  pinMode(stepPin_, OUTPUT);
  pinMode(directionPin_, OUTPUT);
  pinMode(enablePin_, OUTPUT);

  digitalWrite(stepPin_, LOW);
  setDirection(false);
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
  endPulse();
  digitalWrite(enablePin_, HIGH);
  enabled_ = false;
}

void Stepper::setDirection(bool forward)
{
  const bool level = forward ^ directionInverted_;
  digitalWrite(directionPin_, level ? HIGH : LOW);
}

void Stepper::beginPulse()
{
  digitalWrite(stepPin_, HIGH);
}

void Stepper::endPulse()
{
  digitalWrite(stepPin_, LOW);
}

bool Stepper::isEnabled() const
{
  return enabled_;
}
