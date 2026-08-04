#pragma once

#include <Arduino.h>

#include "Config.h"
#include "MotionController.h"
#include "Motor.h"
#include "TMC2209.h"

class Robot
{
public:
  enum class Mode
  {
    Idle,
    Velocity,
    Move,
    Turn,
    Estop
  };

  enum class CommandResult
  {
    Accepted,
    Busy,
    OutOfRange,
    EstopLatched,
    DriverFault
  };

  enum class JobResult
  {
    None,
    Ok,
    Stopped,
    Estopped,
    Fault,
    Replaced
  };

  struct DriverStatus
  {
    bool active = false;
    uint32_t stepFrequencyHz = 0;
    uint32_t pollDurationUs = 0;
    bool telemetryCached = false;
    uint32_t telemetryAgeMs = 0;
    TMC2209::Status telemetry;
  };

  struct Status
  {
    Mode mode = Mode::Idle;
    bool estopLatched = false;
    bool leaseExpiredFault = false;
    uint32_t activeJob = 0;
    float linearSetpointMmPerSecond = 0.0F;
    float turnSetpointDegreesPerSecond = 0.0F;
    uint32_t leaseLeftMs = 0;
    uint32_t lastJob = 0;
    JobResult lastJobResult = JobResult::None;
    uint32_t driverPollTotalUs = 0;
    uint16_t configuredCurrentMa = Config::TMC_RUN_CURRENT_MA;
    uint16_t configuredMicrosteps = Config::TMC_MICROSTEPS;
    float configuredAccelerationMmPerSecondSquared =
        Config::DEFAULT_ACCELERATION_MM_PER_SECOND_SQUARED;
    TMC2209::ChopperMode configuredChopperMode =
        TMC2209::ChopperMode::StealthChop;
    DriverStatus leftDriver;
    DriverStatus rightDriver;
  };

  struct JobCompletion
  {
    uint32_t jobId = 0;
    uint32_t originSequence = 0;
    JobResult result = JobResult::None;
  };

  Robot(Motor& leftMotor, Motor& rightMotor,
        MotionController& motionController, TMC2209& leftDriver,
        TMC2209& rightDriver, arduino::UART& stepperUart);

  void begin();
  void update();

  bool setVelocity(float linearMmPerSecond, float turnDegreesPerSecond);
  bool startMove(float distanceMm);
  bool startTurn(float angleDegrees);

  CommandResult setVelocity(float linearMmPerSecond,
                            float turnDegreesPerSecond, uint32_t leaseMs);
  CommandResult startMove(float distanceMm, uint32_t originSequence,
                          uint32_t& jobId);
  CommandResult startTurn(float angleDegrees, uint32_t originSequence,
                          uint32_t& jobId);
  void stop();
  void estop();
  CommandResult clearEstop();
  CommandResult configure(uint32_t currentMa, uint32_t microsteps,
                          float accelerationMmPerSecondSquared,
                          TMC2209::ChopperMode chopperMode);
  Status status();
  bool takeJobCompletion(JobCompletion& completion);

private:
  bool motionAllowed();
  void updateActiveDriverTelemetry();
  void pollDriverTelemetry(TMC2209& driver, TMC2209::Status& cachedTelemetry,
                           uint32_t& cachedAtMs, bool& cacheValid);
  CommandResult startFiniteMotion(bool turn, float value,
                                  uint32_t originSequence, uint32_t& jobId,
                                  bool trackJob);
  void cancelActiveJob(JobResult result);
  void completeActiveJob(JobResult result);
  DriverStatus driverStatus(TMC2209& driver, const Motor& motor,
                            TMC2209::Status& cachedTelemetry,
                            uint32_t& cachedAtMs, bool& cacheValid);

  Motor& leftMotor_;
  Motor& rightMotor_;
  MotionController& motionController_;
  TMC2209& leftDriver_;
  TMC2209& rightDriver_;
  arduino::UART& stepperUart_;
  Mode mode_ = Mode::Idle;
  bool estopLatched_ = false;
  bool velocityLeaseActive_ = false;
  uint32_t velocityLeaseStartedMs_ = 0;
  uint32_t velocityLeaseDurationMs_ = 0;
  float linearSetpointMmPerSecond_ = 0.0F;
  float turnSetpointDegreesPerSecond_ = 0.0F;
  uint32_t nextJobId_ = 1;
  uint32_t activeJobId_ = 0;
  uint32_t activeJobOriginSequence_ = 0;
  JobCompletion pendingCompletion_;
  bool completionPending_ = false;
  uint32_t lastJobId_ = 0;
  JobResult lastJobResult_ = JobResult::None;
  bool leaseExpiredFault_ = false;
  TMC2209::Settings driverSettings_ = {
      Config::TMC_RUN_CURRENT_MA, Config::TMC_MICROSTEPS,
      TMC2209::ChopperMode::StealthChop};
  TMC2209::Status leftTelemetryCache_;
  TMC2209::Status rightTelemetryCache_;
  uint32_t leftTelemetryCachedAtMs_ = 0;
  uint32_t rightTelemetryCachedAtMs_ = 0;
  bool leftTelemetryCacheValid_ = false;
  bool rightTelemetryCacheValid_ = false;
  bool pollLeftDriverNext_ = true;
  uint32_t lastActiveTelemetryPollMs_ = 0;
};
