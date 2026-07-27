#include "Robot.h"

#include "Config.h"

Robot::Robot(Motor& leftMotor, Motor& rightMotor,
             MotionController& motionController, TMC2209& leftDriver,
             TMC2209& rightDriver, arduino::UART& stepperUart)
    : leftMotor_(leftMotor),
      rightMotor_(rightMotor),
      motionController_(motionController),
      leftDriver_(leftDriver),
      rightDriver_(rightDriver),
      stepperUart_(stepperUart)
{
}

void Robot::begin()
{
  leftMotor_.begin();
  rightMotor_.begin();

  stepperUart_.begin(Config::TMC_UART_BAUD);
  leftDriver_.begin();
  rightDriver_.begin();
}

void Robot::update()
{
  const uint32_t nowUs = micros();
  leftMotor_.update(nowUs);
  rightMotor_.update(nowUs);

  if ((mode_ == Mode::Move || mode_ == Mode::Turn) &&
      !leftMotor_.isBusy() && !rightMotor_.isBusy()) {
    mode_ = Mode::Idle;
  }
}

bool Robot::setVelocity(float linearMmPerSecond,
                        float turnDegreesPerSecond)
{
  if (linearMmPerSecond < -Config::MAX_LINEAR_SPEED_MM_PER_SECOND ||
      linearMmPerSecond > Config::MAX_LINEAR_SPEED_MM_PER_SECOND ||
      turnDegreesPerSecond < -Config::MAX_TURN_SPEED_DEGREES_PER_SECOND ||
      turnDegreesPerSecond > Config::MAX_TURN_SPEED_DEGREES_PER_SECOND) {
    return false;
  }

  motionController_.setVelocity(linearMmPerSecond, turnDegreesPerSecond);
  linearSetpointMmPerSecond_ = linearMmPerSecond;
  turnSetpointDegreesPerSecond_ = turnDegreesPerSecond;
  mode_ = linearMmPerSecond == 0.0F && turnDegreesPerSecond == 0.0F
              ? Mode::Idle
              : Mode::Velocity;
  return true;
}

bool Robot::startMove(float distanceMm)
{
  if (!motionController_.moveDistance(distanceMm)) {
    return false;
  }

  linearSetpointMmPerSecond_ = 0.0F;
  turnSetpointDegreesPerSecond_ = 0.0F;
  mode_ = Mode::Move;
  return true;
}

bool Robot::startTurn(float angleDegrees)
{
  if (!motionController_.turnAngle(angleDegrees)) {
    return false;
  }

  linearSetpointMmPerSecond_ = 0.0F;
  turnSetpointDegreesPerSecond_ = 0.0F;
  mode_ = Mode::Turn;
  return true;
}

void Robot::stop()
{
  motionController_.stop();
  linearSetpointMmPerSecond_ = 0.0F;
  turnSetpointDegreesPerSecond_ = 0.0F;
  mode_ = Mode::Idle;
}

Robot::Status Robot::status()
{
  Status result;
  result.mode = mode_;
  result.linearSetpointMmPerSecond = linearSetpointMmPerSecond_;
  result.turnSetpointDegreesPerSecond = turnSetpointDegreesPerSecond_;
  result.leftDriver = driverStatus(leftDriver_, leftMotor_);
  result.rightDriver = driverStatus(rightDriver_, rightMotor_);
  return result;
}

Robot::DriverStatus Robot::driverStatus(TMC2209& driver, const Motor& motor)
{
  DriverStatus result;
  result.connected = driver.isConnected();
  result.active = motor.isEnabled();
  if (result.connected) {
    result.flags = driver.status();
  }
  return result;
}
