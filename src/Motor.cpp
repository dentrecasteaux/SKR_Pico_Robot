#include "Motor.h"

namespace
{
constexpr uint32_t PULSE_WIDTH_US = 5;
}

Motor::Motor(Stepper& stepper) : stepper_(stepper) {}

void Motor::begin() { stepper_.begin(); }
void Motor::enable() { stepper_.enable(); }

void Motor::disable()
{
  mode_ = Mode::Idle;
  pulseHigh_ = false;
  remainingSteps_ = 0;
  speed_ = 0;
  stepper_.disable();
}

bool Motor::startTest(uint32_t steps, uint32_t stepIntervalUs)
{
  return startMove(steps, true, stepIntervalUs);
}

bool Motor::startMove(uint32_t steps, bool forward, uint32_t stepIntervalUs)
{
  if (isBusy() || steps == 0 || stepIntervalUs == 0) return false;
  stepper_.setDirection(forward);
  stepper_.enable();
  mode_ = Mode::PulseTrain;
  remainingSteps_ = steps;
  stepIntervalUs_ = stepIntervalUs;
  lastTransitionUs_ = micros();
  return true;
}

void Motor::setSpeed(int32_t stepsPerSecond)
{
  if (stepsPerSecond == 0) {
    disable();
    return;
  }

  stepper_.endPulse();
  pulseHigh_ = false;
  stepper_.setDirection(stepsPerSecond > 0);
  stepper_.enable();
  mode_ = Mode::Continuous;
  speed_ = stepsPerSecond;
  stepIntervalUs_ = 1000000UL / static_cast<uint32_t>(abs(stepsPerSecond));
  lastTransitionUs_ = micros();
}

void Motor::update(uint32_t nowUs)
{
  if (!isBusy()) return;
  const uint32_t elapsedUs = nowUs - lastTransitionUs_;

  if (!pulseHigh_ && elapsedUs >= stepIntervalUs_) {
    stepper_.beginPulse();
    pulseHigh_ = true;
    lastTransitionUs_ = nowUs;
  } else if (pulseHigh_ && elapsedUs >= PULSE_WIDTH_US) {
    stepper_.endPulse();
    pulseHigh_ = false;
    lastTransitionUs_ = nowUs;
    if (mode_ == Mode::PulseTrain && --remainingSteps_ == 0) disable();
  }
}

bool Motor::isEnabled() const { return stepper_.isEnabled(); }
bool Motor::isBusy() const { return mode_ != Mode::Idle; }
int32_t Motor::speed() const { return speed_; }
