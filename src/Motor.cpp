#include "Motor.h"

#include <cmath>

namespace
{
constexpr uint32_t PULSE_WIDTH_US = 5;
}

Motor::Motor(Stepper& stepper, uint32_t accelerationStepsPerSecondSquared)
    : stepper_(stepper),
      accelerationStepsPerSecondSquared_(accelerationStepsPerSecondSquared)
{
}

void Motor::begin() { stepper_.begin(); }
void Motor::enable() { stepper_.enable(); }

void Motor::disable()
{
  mode_ = Mode::Idle;
  pulseHigh_ = false;
  remainingSteps_ = 0;
  targetSpeed_ = 0;
  currentSpeed_ = 0.0F;
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
  if (mode_ == Mode::PulseTrain) {
    disable();
  }

  targetSpeed_ = stepsPerSecond;
  if (mode_ == Mode::Idle && stepsPerSecond != 0) {
    mode_ = Mode::Continuous;
    stepper_.setDirection(stepsPerSecond > 0);
    stepper_.enable();
    lastTransitionUs_ = micros();
    lastSpeedUpdateUs_ = lastTransitionUs_;
  }
}

void Motor::update(uint32_t nowUs)
{
  if (!isBusy()) return;
  updateSpeed(nowUs);
  if (mode_ == Mode::Idle) return;

  if (mode_ == Mode::Continuous) {
    const uint32_t magnitude = static_cast<uint32_t>(fabsf(currentSpeed_));
    if (magnitude == 0) return;
    stepIntervalUs_ = 1000000UL / magnitude;
  }

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

void Motor::updateSpeed(uint32_t nowUs)
{
  if (mode_ != Mode::Continuous) return;

  const uint32_t elapsedUs = nowUs - lastSpeedUpdateUs_;
  lastSpeedUpdateUs_ = nowUs;
  const float change = accelerationStepsPerSecondSquared_ * elapsedUs / 1000000.0F;

  if (currentSpeed_ < 0.0F && targetSpeed_ > 0) {
    currentSpeed_ = fminf(0.0F, currentSpeed_ + change);
  } else if (currentSpeed_ > 0.0F && targetSpeed_ < 0) {
    currentSpeed_ = fmaxf(0.0F, currentSpeed_ - change);
  } else if (currentSpeed_ < targetSpeed_) {
    currentSpeed_ = fminf(static_cast<float>(targetSpeed_), currentSpeed_ + change);
  } else if (currentSpeed_ > targetSpeed_) {
    currentSpeed_ = fmaxf(static_cast<float>(targetSpeed_), currentSpeed_ - change);
  }

  if (currentSpeed_ == 0.0F) {
    if (targetSpeed_ == 0) {
      disable();
    } else {
      stepper_.setDirection(targetSpeed_ > 0);
      stepper_.enable();
    }
  }
}

bool Motor::isEnabled() const { return stepper_.isEnabled(); }
bool Motor::isBusy() const { return mode_ != Mode::Idle; }
int32_t Motor::speed() const { return static_cast<int32_t>(currentSpeed_); }
