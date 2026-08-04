#include "MotionController.h"

#include <cmath>

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
    : leftMotor_(leftMotor),
      rightMotor_(rightMotor),
      microsteps_(Config::TMC_MICROSTEPS),
      accelerationMmPerSecondSquared_(
          Config::DEFAULT_ACCELERATION_MM_PER_SECOND_SQUARED)
{
  configureMotion(microsteps_, accelerationMmPerSecondSquared_);
}

void MotionController::setDrive(int32_t linearSpeed, int32_t turnSpeed)
{
  const int64_t requestedLeft = static_cast<int64_t>(linearSpeed) - turnSpeed;
  const int64_t requestedRight = static_cast<int64_t>(linearSpeed) + turnSpeed;
  setWheelSpeeds(static_cast<int32_t>(requestedLeft),
                 static_cast<int32_t>(requestedRight),
                 Config::MAX_SPEED_STEPS_PER_SECOND);
}

void MotionController::setWheelSpeeds(int32_t left, int32_t right,
                                      int32_t maximumMagnitude)
{
  const int64_t requestedLeft = left;
  const int64_t requestedRight = right;
  const int64_t largestMagnitude =
      absoluteValue(requestedLeft) > absoluteValue(requestedRight)
          ? absoluteValue(requestedLeft)
          : absoluteValue(requestedRight);

  if (largestMagnitude <= maximumMagnitude) {
    leftMotor_.setSpeed(static_cast<int32_t>(requestedLeft));
    rightMotor_.setSpeed(static_cast<int32_t>(requestedRight));
    return;
  }

  leftMotor_.setSpeed(static_cast<int32_t>(
      requestedLeft * maximumMagnitude / largestMagnitude));
  rightMotor_.setSpeed(static_cast<int32_t>(
      requestedRight * maximumMagnitude / largestMagnitude));
}

void MotionController::setVelocity(float linearMmPerSecond,
                                   float turnDegreesPerSecond)
{
  const float stepsPerMm = stepsPerMillimetre();
  const float turnRadiansPerSecond = turnDegreesPerSecond * PI_VALUE / 180.0F;
  const float halfTrackMm = Config::WHEEL_TRACK_MM / 2.0F;

  const float leftMmPerSecond = linearMmPerSecond - turnRadiansPerSecond * halfTrackMm;
  const float rightMmPerSecond = linearMmPerSecond + turnRadiansPerSecond * halfTrackMm;

  const int32_t leftStepsPerSecond =
      static_cast<int32_t>(leftMmPerSecond * stepsPerMm);
  const int32_t rightStepsPerSecond =
      static_cast<int32_t>(rightMmPerSecond * stepsPerMm);

  setWheelSpeeds(leftStepsPerSecond, rightStepsPerSecond,
                 Config::MAX_PIO_STEP_FREQUENCY_HZ);
}

bool MotionController::moveDistance(float distanceMm)
{
  if (leftMotor_.isBusy() || rightMotor_.isBusy() || distanceMm == 0.0F) {
    return false;
  }

  const uint32_t steps = static_cast<uint32_t>(
      lroundf(fabsf(distanceMm) * stepsPerMillimetre()));
  if (steps == 0) return false;

  const bool forward = distanceMm > 0.0F;
  return leftMotor_.startMove(steps, forward,
                              Config::CALIBRATION_MOVE_STEP_INTERVAL_US) &&
         rightMotor_.startMove(steps, forward,
                               Config::CALIBRATION_MOVE_STEP_INTERVAL_US);
}

bool MotionController::turnAngle(float angleDegrees)
{
  if (leftMotor_.isBusy() || rightMotor_.isBusy() || angleDegrees == 0.0F) {
    return false;
  }

  const float wheelTravelMm = fabsf(angleDegrees) * PI_VALUE / 180.0F *
                              Config::WHEEL_TRACK_MM / 2.0F;
  const uint32_t steps =
      static_cast<uint32_t>(lroundf(wheelTravelMm * stepsPerMillimetre()));
  if (steps == 0) return false;

  const bool turnLeft = angleDegrees > 0.0F;
  return leftMotor_.startMove(steps, !turnLeft,
                              Config::CALIBRATION_MOVE_STEP_INTERVAL_US) &&
         rightMotor_.startMove(steps, turnLeft,
                               Config::CALIBRATION_MOVE_STEP_INTERVAL_US);
}

void MotionController::stop()
{
  leftMotor_.disable();
  rightMotor_.disable();
}

void MotionController::configureMotion(
    uint16_t microsteps, float accelerationMmPerSecondSquared)
{
  microsteps_ = microsteps;
  accelerationMmPerSecondSquared_ = accelerationMmPerSecondSquared;
  const uint32_t accelerationSteps = static_cast<uint32_t>(
      lroundf(accelerationMmPerSecondSquared_ * stepsPerMillimetre()));
  leftMotor_.setAcceleration(accelerationSteps);
  rightMotor_.setAcceleration(accelerationSteps);
}

uint16_t MotionController::microsteps() const { return microsteps_; }

float MotionController::accelerationMmPerSecondSquared() const
{
  return accelerationMmPerSecondSquared_;
}

float MotionController::stepsPerMillimetre() const
{
  return (Config::MOTOR_FULL_STEPS_PER_REVOLUTION * microsteps_) /
         (PI_VALUE * Config::WHEEL_DIAMETER_MM);
}
