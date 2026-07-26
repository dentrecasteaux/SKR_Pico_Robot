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
  remainingSteps_ = 0;
  pulseHigh_ = false;
  digitalWrite(stepPin_, LOW);
  digitalWrite(enablePin_, HIGH);
  enabled_ = false;
}

bool Stepper::startTest(uint32_t steps, uint32_t stepIntervalUs)
{
  return startMove(steps, true, stepIntervalUs);
}

bool Stepper::startMove(uint32_t steps, bool forward, uint32_t stepIntervalUs)
{
  if (isBusy() || steps == 0 || stepIntervalUs == 0) {
    return false;
  }

  setDirection(forward);
  enable();
  remainingSteps_ = steps;
  stepIntervalUs_ = stepIntervalUs;
  lastTransitionUs_ = micros();
  return true;
}

void Stepper::setDirection(bool forward)
{
  const bool level = forward ^ directionInverted_;
  digitalWrite(directionPin_, level ? HIGH : LOW);
}

void Stepper::update(uint32_t nowUs)
{
  if (!isBusy()) {
    return;
  }

  const uint32_t elapsedUs = nowUs - lastTransitionUs_;

  if (!pulseHigh_ && elapsedUs >= stepIntervalUs_) {
    digitalWrite(stepPin_, HIGH);
    pulseHigh_ = true;
    lastTransitionUs_ = nowUs;
  } else if (pulseHigh_ && elapsedUs >= 5) {
    digitalWrite(stepPin_, LOW);
    pulseHigh_ = false;
    lastTransitionUs_ = nowUs;

    if (--remainingSteps_ == 0) {
      disable();
    }
  }
}

bool Stepper::isEnabled() const
{
  return enabled_;
}

bool Stepper::isBusy() const
{
  return remainingSteps_ > 0;
}
