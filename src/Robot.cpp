#include "Robot.h"

#include <cmath>

#include "Config.h"

namespace
{
constexpr uint32_t DRIVER_FAULT_MASK =
    (1UL << 25) | (1UL << 26) | (1UL << 27) | (1UL << 28);
}

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
    completeActiveJob(JobResult::Ok);
    mode_ = Mode::Idle;
  }

  if (mode_ == Mode::Velocity && velocityLeaseActive_ &&
      millis() - velocityLeaseStartedMs_ >= velocityLeaseDurationMs_) {
    motionController_.stop();
    velocityLeaseActive_ = false;
    linearSetpointMmPerSecond_ = 0.0F;
    turnSetpointDegreesPerSecond_ = 0.0F;
    leaseExpiredFault_ = true;
    mode_ = Mode::Idle;
  }
}

bool Robot::setVelocity(float linearMmPerSecond,
                        float turnDegreesPerSecond)
{
  if (estopLatched_ ||
      linearMmPerSecond < -Config::MAX_LINEAR_SPEED_MM_PER_SECOND ||
      linearMmPerSecond > Config::MAX_LINEAR_SPEED_MM_PER_SECOND ||
      turnDegreesPerSecond < -Config::MAX_TURN_SPEED_DEGREES_PER_SECOND ||
      turnDegreesPerSecond > Config::MAX_TURN_SPEED_DEGREES_PER_SECOND) {
    return false;
  }

  cancelActiveJob(JobResult::Replaced);
  motionController_.setVelocity(linearMmPerSecond, turnDegreesPerSecond);
  velocityLeaseActive_ = false;
  linearSetpointMmPerSecond_ = linearMmPerSecond;
  turnSetpointDegreesPerSecond_ = turnDegreesPerSecond;
  mode_ = linearMmPerSecond == 0.0F && turnDegreesPerSecond == 0.0F
              ? Mode::Idle
              : Mode::Velocity;
  return true;
}

Robot::CommandResult Robot::setVelocity(float linearMmPerSecond,
                                        float turnDegreesPerSecond,
                                        uint32_t leaseMs)
{
  if (estopLatched_) return CommandResult::EstopLatched;
  if (linearMmPerSecond < -Config::MAX_LINEAR_SPEED_MM_PER_SECOND ||
      linearMmPerSecond > Config::MAX_LINEAR_SPEED_MM_PER_SECOND ||
      turnDegreesPerSecond < -Config::MAX_TURN_SPEED_DEGREES_PER_SECOND ||
      turnDegreesPerSecond > Config::MAX_TURN_SPEED_DEGREES_PER_SECOND ||
      leaseMs < Config::MIN_VELOCITY_LEASE_MS ||
      leaseMs > Config::MAX_VELOCITY_LEASE_MS) {
    return CommandResult::OutOfRange;
  }
  if (!motionAllowed()) return CommandResult::DriverFault;

  cancelActiveJob(JobResult::Replaced);
  motionController_.setVelocity(linearMmPerSecond, turnDegreesPerSecond);
  linearSetpointMmPerSecond_ = linearMmPerSecond;
  turnSetpointDegreesPerSecond_ = turnDegreesPerSecond;
  velocityLeaseStartedMs_ = millis();
  velocityLeaseDurationMs_ = leaseMs;
  velocityLeaseActive_ = true;
  leaseExpiredFault_ = false;
  mode_ = Mode::Velocity;
  return CommandResult::Accepted;
}

bool Robot::startMove(float distanceMm)
{
  uint32_t unusedJobId = 0;
  return startFiniteMotion(false, distanceMm, 0, unusedJobId, false) ==
         CommandResult::Accepted;
}

bool Robot::startTurn(float angleDegrees)
{
  uint32_t unusedJobId = 0;
  return startFiniteMotion(true, angleDegrees, 0, unusedJobId, false) ==
         CommandResult::Accepted;
}

Robot::CommandResult Robot::startMove(float distanceMm,
                                      uint32_t originSequence,
                                      uint32_t& jobId)
{
  return startFiniteMotion(false, distanceMm, originSequence, jobId, true);
}

Robot::CommandResult Robot::startTurn(float angleDegrees,
                                      uint32_t originSequence,
                                      uint32_t& jobId)
{
  return startFiniteMotion(true, angleDegrees, originSequence, jobId, true);
}

Robot::CommandResult Robot::startFiniteMotion(bool turn, float value,
                                              uint32_t originSequence,
                                              uint32_t& jobId, bool trackJob)
{
  if (estopLatched_) return CommandResult::EstopLatched;
  if (value == 0.0F ||
      (!turn && std::fabs(value) > Config::MAX_MOVE_DISTANCE_MM) ||
      (turn && std::fabs(value) > Config::MAX_TURN_ANGLE_DEGREES)) {
    return CommandResult::OutOfRange;
  }
  if (mode_ == Mode::Move || mode_ == Mode::Turn) {
    return CommandResult::Busy;
  }
  if (!motionAllowed()) return CommandResult::DriverFault;

  if (mode_ == Mode::Velocity) {
    motionController_.stop();
    velocityLeaseActive_ = false;
  }

  const bool started =
      turn ? motionController_.turnAngle(value)
           : motionController_.moveDistance(value);
  if (!started) return CommandResult::Busy;

  jobId = 0;
  if (trackJob) {
    jobId = nextJobId_++;
    if (nextJobId_ == 0) nextJobId_ = 1;
    activeJobId_ = jobId;
    activeJobOriginSequence_ = originSequence;
  }
  linearSetpointMmPerSecond_ = 0.0F;
  turnSetpointDegreesPerSecond_ = 0.0F;
  leaseExpiredFault_ = false;
  mode_ = turn ? Mode::Turn : Mode::Move;
  return CommandResult::Accepted;
}

void Robot::stop()
{
  motionController_.stop();
  cancelActiveJob(JobResult::Stopped);
  velocityLeaseActive_ = false;
  linearSetpointMmPerSecond_ = 0.0F;
  turnSetpointDegreesPerSecond_ = 0.0F;
  if (!estopLatched_) mode_ = Mode::Idle;
}

void Robot::estop()
{
  motionController_.stop();
  cancelActiveJob(JobResult::Estopped);
  velocityLeaseActive_ = false;
  linearSetpointMmPerSecond_ = 0.0F;
  turnSetpointDegreesPerSecond_ = 0.0F;
  estopLatched_ = true;
  mode_ = Mode::Estop;
}

Robot::CommandResult Robot::clearEstop()
{
  if (leftMotor_.isBusy() || rightMotor_.isBusy()) {
    return CommandResult::Busy;
  }
  if (!motionAllowed()) return CommandResult::DriverFault;
  if (!estopLatched_) return CommandResult::Accepted;

  estopLatched_ = false;
  mode_ = Mode::Idle;
  return CommandResult::Accepted;
}

Robot::Status Robot::status()
{
  Status result;
  result.mode = mode_;
  result.estopLatched = estopLatched_;
  result.leaseExpiredFault = leaseExpiredFault_;
  result.activeJob = activeJobId_;
  result.linearSetpointMmPerSecond = linearSetpointMmPerSecond_;
  result.turnSetpointDegreesPerSecond = turnSetpointDegreesPerSecond_;
  if (velocityLeaseActive_) {
    const uint32_t elapsedMs = millis() - velocityLeaseStartedMs_;
    result.leaseLeftMs = elapsedMs < velocityLeaseDurationMs_
                             ? velocityLeaseDurationMs_ - elapsedMs
                             : 0;
  }
  result.lastJob = lastJobId_;
  result.lastJobResult = lastJobResult_;
  result.leftDriver = driverStatus(leftDriver_, leftMotor_);
  result.rightDriver = driverStatus(rightDriver_, rightMotor_);
  return result;
}

bool Robot::takeJobCompletion(JobCompletion& completion)
{
  if (!completionPending_) return false;
  completion = pendingCompletion_;
  completionPending_ = false;
  return true;
}

bool Robot::motionAllowed()
{
  const DriverStatus left = driverStatus(leftDriver_, leftMotor_);
  const DriverStatus right = driverStatus(rightDriver_, rightMotor_);
  return left.connected && right.connected &&
         (left.flags & DRIVER_FAULT_MASK) == 0 &&
         (right.flags & DRIVER_FAULT_MASK) == 0;
}

void Robot::cancelActiveJob(JobResult result)
{
  if (activeJobId_ != 0) completeActiveJob(result);
}

void Robot::completeActiveJob(JobResult result)
{
  if (activeJobId_ == 0) return;

  pendingCompletion_.jobId = activeJobId_;
  pendingCompletion_.originSequence = activeJobOriginSequence_;
  pendingCompletion_.result = result;
  completionPending_ = true;
  lastJobId_ = activeJobId_;
  lastJobResult_ = result;
  activeJobId_ = 0;
  activeJobOriginSequence_ = 0;
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
