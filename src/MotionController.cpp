#include "MotionController.h"

#include "Config.h"

namespace
{
constexpr float PI_VALUE = 3.14159265358979323846F;

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

void MotionController::setVelocity(float linearMmPerSecond,
                                   float turnDegreesPerSecond)
{
  const float wheelCircumferenceMm = PI_VALUE * Config::WHEEL_DIAMETER_MM;
  const float stepsPerMillimetre =
      (Config::MOTOR_FULL_STEPS_PER_REVOLUTION * Config::TMC_MICROSTEPS) /
      wheelCircumferenceMm;
  const float turnRadiansPerSecond = turnDegreesPerSecond * PI_VALUE / 180.0F;
  const float halfTrackMm = Config::WHEEL_TRACK_MM / 2.0F;

  const float leftMmPerSecond = linearMmPerSecond - turnRadiansPerSecond * halfTrackMm;
  const float rightMmPerSecond = linearMmPerSecond + turnRadiansPerSecond * halfTrackMm;

  const int32_t leftStepsPerSecond =
      static_cast<int32_t>(leftMmPerSecond * stepsPerMillimetre);
  const int32_t rightStepsPerSecond =
      static_cast<int32_t>(rightMmPerSecond * stepsPerMillimetre);

  const int32_t linearStepsPerSecond =
      (leftStepsPerSecond + rightStepsPerSecond) / 2;
  const int32_t turnStepsPerSecond =
      (rightStepsPerSecond - leftStepsPerSecond) / 2;
  setDrive(linearStepsPerSecond, turnStepsPerSecond);
}

void MotionController::stop()
{
  leftMotor_.disable();
  rightMotor_.disable();
}
