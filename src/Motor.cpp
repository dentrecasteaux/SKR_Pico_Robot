#include "Motor.h"

#include <cmath>

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
  stepper_.stopPulses();
  targetSpeed_ = 0;
  currentSpeed_ = 0.0F;
  programmedFrequencyHz_ = 0;
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
  if (!stepper_.startPulses(steps, stepIntervalUs)) {
    stepper_.disable();
    return false;
  }
  mode_ = Mode::PulseTrain;
  stepIntervalUs_ = stepIntervalUs;
  programmedFrequencyHz_ = 1000000UL / stepIntervalUs;
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
    lastSpeedUpdateUs_ = micros();
  }
}

void Motor::update(uint32_t nowUs)
{
  if (!isBusy()) return;
  if (mode_ == Mode::PulseTrain) {
    if (stepper_.pulsesComplete()) disable();
    return;
  }

  updateSpeed(nowUs);
  if (mode_ == Mode::Continuous) updatePulseFrequency();
}

void Motor::updatePulseFrequency()
{
  const uint32_t magnitude = static_cast<uint32_t>(fabsf(currentSpeed_));
  if (magnitude == programmedFrequencyHz_) return;

  if (magnitude == 0) {
    stepper_.stopPulses();
    programmedFrequencyHz_ = 0;
    return;
  }

  const uint32_t intervalUs = 1000000UL / magnitude;
  const bool updated = programmedFrequencyHz_ == 0
                           ? stepper_.startPulses(UINT32_MAX, intervalUs)
                           : stepper_.updatePulseInterval(intervalUs);
  if (updated) {
    stepIntervalUs_ = intervalUs;
    programmedFrequencyHz_ = magnitude;
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
      stepper_.stopPulses();
      programmedFrequencyHz_ = 0;
      stepper_.setDirection(targetSpeed_ > 0);
      stepper_.enable();
    }
  }
}

bool Motor::isEnabled() const { return stepper_.isEnabled(); }
bool Motor::isBusy() const { return mode_ != Mode::Idle; }
int32_t Motor::speed() const { return static_cast<int32_t>(currentSpeed_); }

uint32_t Motor::stepFrequencyHz() const
{
  return programmedFrequencyHz_;
}
