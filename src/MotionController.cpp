#include "MotionController.h"

#include "Config.h"

namespace
{
int64_t absoluteValue(int64_t value)
{
  return value < 0 ? -value : value;
}
}  // namespace

MotionController::MotionController(Motor& leftMotor, Motor& rightMotor)
    : leftMotor_(leftMotor), rightMotor_(rightMotor)
{
}

void MotionController::setDrive(int32_t linearSpeed, int32_t turnSpeed)
{
  const int64_t requestedLeft = static_cast<int64_t>(linearSpeed) - turnSpeed;
  const int64_t requestedRight = static_cast<int64_t>(linearSpeed) + turnSpeed;
  const int64_t largestMagnitude =
      absoluteValue(requestedLeft) > absoluteValue(requestedRight)
          ? absoluteValue(requestedLeft)
          : absoluteValue(requestedRight);

  if (largestMagnitude <= Config::MAX_SPEED_STEPS_PER_SECOND) {
    leftMotor_.setSpeed(static_cast<int32_t>(requestedLeft));
    rightMotor_.setSpeed(static_cast<int32_t>(requestedRight));
    return;
  }

  leftMotor_.setSpeed(static_cast<int32_t>(
      requestedLeft * Config::MAX_SPEED_STEPS_PER_SECOND / largestMagnitude));
  rightMotor_.setSpeed(static_cast<int32_t>(
      requestedRight * Config::MAX_SPEED_STEPS_PER_SECOND / largestMagnitude));
}

void MotionController::stop()
{
  leftMotor_.disable();
  rightMotor_.disable();
}
