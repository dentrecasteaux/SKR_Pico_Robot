#include "RobotLink.h"

#include <cstring>

#include "Protocol.h"

RobotLink::RobotLink(Stream& stream, Robot& robot,
                     LegacyCommandHandler legacyCommandHandler)
    : transport_(stream),
      robot_(robot),
      legacyCommandHandler_(legacyCommandHandler)
{
}

void RobotLink::update()
{
  sendPendingCompletion();

  const char* line = nullptr;
  while (true) {
    const SerialTransport::ReadResult result = transport_.readLine(line);
    if (result == SerialTransport::ReadResult::None) {
      return;
    }

    if (result == SerialTransport::ReadResult::LineTooLong) {
      sendError(0, "LINE_TOO_LONG");
      continue;
    }

    processLine(const_cast<char*>(line));
    sendPendingCompletion();
  }
}

void RobotLink::processLine(char* line)
{
  if (std::strncmp(line, "R2W", 3) != 0) {
    legacyCommandHandler_(line);
    return;
  }

  char originalLine[CACHED_COMMAND_SIZE];
  std::strncpy(originalLine, line, CACHED_COMMAND_SIZE);
  originalLine[CACHED_COMMAND_SIZE - 1] = '\0';

  Protocol::Command command;
  const Protocol::ParseResult result = Protocol::parseCommand(line, command);
  if (!result.valid) {
    sendError(result.sequence, result.errorCode);
    return;
  }

  if (cachedReplyValid_ && command.sequence == cachedSequence_) {
    replayCachedReply(originalLine, command.sequence);
    return;
  }

  lastValidCommandMs_ = millis();
  switch (command.type) {
    case Protocol::CommandType::Ping:
      sendPong(command.sequence);
      return;
    case Protocol::CommandType::Status:
      sendStatus(command.sequence);
      return;
    case Protocol::CommandType::SetVelocity: {
      const Robot::CommandResult commandResult = robot_.setVelocity(
          command.linearMmPerSecond, command.turnDegreesPerSecond,
          command.leaseMs);
      if (commandResult == Robot::CommandResult::Accepted) {
        cacheReply(originalLine, command.sequence, true);
        sendAck(command.sequence);
      } else {
        cacheReply(originalLine, command.sequence, false, 0,
                   commandErrorCode(commandResult));
        sendCommandError(command.sequence, commandResult);
      }
      return;
    }
    case Protocol::CommandType::Move:
    case Protocol::CommandType::Turn: {
      uint32_t jobId = 0;
      const Robot::CommandResult commandResult =
          command.type == Protocol::CommandType::Move
              ? robot_.startMove(command.distanceMm, command.sequence, jobId)
              : robot_.startTurn(command.angleDegrees, command.sequence, jobId);
      if (commandResult == Robot::CommandResult::Accepted) {
        cacheReply(originalLine, command.sequence, true, jobId);
        sendAck(command.sequence, jobId);
      } else {
        cacheReply(originalLine, command.sequence, false, 0,
                   commandErrorCode(commandResult));
        sendCommandError(command.sequence, commandResult);
      }
      return;
    }
    case Protocol::CommandType::Stop:
      robot_.stop();
      cacheReply(originalLine, command.sequence, true);
      sendAck(command.sequence);
      return;
    case Protocol::CommandType::Estop:
      robot_.estop();
      cacheReply(originalLine, command.sequence, true);
      sendAck(command.sequence);
      return;
    case Protocol::CommandType::ClearEstop: {
      const Robot::CommandResult commandResult = robot_.clearEstop();
      if (commandResult == Robot::CommandResult::Accepted) {
        cacheReply(originalLine, command.sequence, true);
        sendAck(command.sequence);
      } else {
        cacheReply(originalLine, command.sequence, false, 0,
                   commandErrorCode(commandResult));
        sendCommandError(command.sequence, commandResult);
      }
      return;
    }
    case Protocol::CommandType::Configure: {
      const TMC2209::ChopperMode mode =
          command.chopperMode == Protocol::ChopperMode::SpreadCycle
              ? TMC2209::ChopperMode::SpreadCycle
              : TMC2209::ChopperMode::StealthChop;
      const Robot::CommandResult commandResult = robot_.configure(
          command.currentMa, command.microsteps,
          command.accelerationMmPerSecondSquared, mode);
      if (commandResult == Robot::CommandResult::Accepted) {
        cacheReply(originalLine, command.sequence, true);
        sendAck(command.sequence);
      } else {
        cacheReply(originalLine, command.sequence, false, 0,
                   commandErrorCode(commandResult));
        sendCommandError(command.sequence, commandResult);
      }
      return;
    }
  }
}

void RobotLink::sendError(uint32_t sequence, const char* code)
{
  Print& output = transport_.output();
  output.print(Protocol::VERSION);
  output.print(" ERR ");
  output.print(sequence);
  output.print(" CODE=");
  output.println(code);
}

void RobotLink::sendAck(uint32_t sequence, uint32_t jobId)
{
  Print& output = transport_.output();
  output.print(Protocol::VERSION);
  output.print(" ACK ");
  output.print(sequence);
  output.print(" OK");
  if (jobId != 0) {
    output.print(" JOB=");
    output.print(jobId);
  }
  output.println();
}

void RobotLink::sendCommandError(uint32_t sequence,
                                 Robot::CommandResult result)
{
  sendError(sequence, commandErrorCode(result));
}

const char* RobotLink::commandErrorCode(Robot::CommandResult result) const
{
  switch (result) {
    case Robot::CommandResult::Accepted:
      return "INVALID_VALUE";
    case Robot::CommandResult::Busy:
      return "BUSY";
    case Robot::CommandResult::OutOfRange:
      return "OUT_OF_RANGE";
    case Robot::CommandResult::EstopLatched:
      return "ESTOP_LATCHED";
    case Robot::CommandResult::DriverFault:
      return "DRIVER_FAULT";
  }
  return "INVALID_VALUE";
}

void RobotLink::cacheReply(const char* commandLine, uint32_t sequence,
                           bool accepted, uint32_t jobId,
                           const char* errorCode)
{
  std::strncpy(cachedCommand_, commandLine, CACHED_COMMAND_SIZE);
  cachedCommand_[CACHED_COMMAND_SIZE - 1] = '\0';
  cachedSequence_ = sequence;
  cachedReplyAccepted_ = accepted;
  cachedJobId_ = jobId;
  cachedErrorCode_ = errorCode;
  cachedReplyValid_ = true;
}

bool RobotLink::replayCachedReply(const char* commandLine, uint32_t sequence)
{
  if (std::strcmp(commandLine, cachedCommand_) != 0) {
    sendError(sequence, "SEQUENCE_CONFLICT");
    return true;
  }

  if (cachedReplyAccepted_) {
    sendAck(sequence, cachedJobId_);
  } else {
    sendError(sequence, cachedErrorCode_);
  }
  return true;
}

void RobotLink::sendPendingCompletion()
{
  Robot::JobCompletion completion;
  if (robot_.takeJobCompletion(completion)) {
    sendDone(completion);
  }
}

void RobotLink::sendDone(const Robot::JobCompletion& completion)
{
  Print& output = transport_.output();
  output.print(Protocol::VERSION);
  output.print(" DONE JOB=");
  output.print(completion.jobId);
  output.print(" ORIGIN_SEQ=");
  output.print(completion.originSequence);
  output.print(" RESULT=");
  output.println(jobResultName(completion.result));
}

void RobotLink::sendPong(uint32_t sequence)
{
  Print& output = transport_.output();
  output.print(Protocol::VERSION);
  output.print(" PONG ");
  output.print(sequence);
  output.print(" UPTIME_MS=");
  output.println(millis());
}

void RobotLink::sendStatus(uint32_t sequence)
{
  const Robot::Status status = robot_.status();
  const char* mode = "IDLE";
  switch (status.mode) {
    case Robot::Mode::Idle:
      mode = "IDLE";
      break;
    case Robot::Mode::Velocity:
      mode = "VELOCITY";
      break;
    case Robot::Mode::Move:
      mode = "MOVE";
      break;
    case Robot::Mode::Turn:
      mode = "TURN";
      break;
    case Robot::Mode::Estop:
      mode = "ESTOP";
      break;
  }

  Print& output = transport_.output();
  output.print(Protocol::VERSION);
  output.print(" STAT ");
  output.print(sequence);
  output.print(" MODE=");
  output.print(mode);
  output.print(" ESTOP=");
  output.print(status.estopLatched ? 1 : 0);
  output.print(" FAULTS=");
  const bool driverFault =
      status.leftDriver.telemetry.hasFault() ||
      status.rightDriver.telemetry.hasFault();
  if (!status.leaseExpiredFault && !driverFault) {
    output.print("NONE");
  } else {
    if (status.leaseExpiredFault) output.print("LEASE_EXPIRED");
    if (status.leaseExpiredFault && driverFault) output.print(',');
    if (driverFault) output.print("DRIVER");
  }
  output.print(" JOB=");
  output.print(status.activeJob);
  output.print(" V_SET=");
  output.print(status.linearSetpointMmPerSecond);
  output.print(" W_SET=");
  output.print(status.turnSetpointDegreesPerSecond);
  output.print(" LEASE_LEFT_MS=");
  output.print(status.leaseLeftMs);
  output.print(" LAST_JOB=");
  output.print(status.lastJob);
  output.print(" LAST_RESULT=");
  output.print(jobResultName(status.lastJobResult));
  output.print(" X_DRIVER=");
  printDriverStatus(status.leftDriver);
  output.print(" Y_DRIVER=");
  printDriverStatus(status.rightDriver);
  output.print(" X_CS=");
  if (status.leftDriver.telemetry.connected)
    output.print(status.leftDriver.telemetry.currentScale);
  else
    output.print("NA");
  output.print(" Y_CS=");
  if (status.rightDriver.telemetry.connected)
    output.print(status.rightDriver.telemetry.currentScale);
  else
    output.print("NA");
  output.print(" X_TMC_MODE=");
  if (!status.leftDriver.telemetry.connected)
    output.print("UNKNOWN");
  else
    output.print(status.leftDriver.telemetry.stealthChop ? "STEALTHCHOP" :
                                                         "SPREADCYCLE");
  output.print(" Y_TMC_MODE=");
  if (!status.rightDriver.telemetry.connected)
    output.print("UNKNOWN");
  else
    output.print(status.rightDriver.telemetry.stealthChop ? "STEALTHCHOP" :
                                                          "SPREADCYCLE");
  output.print(" X_FULLSTEP=");
  output.print(status.leftDriver.telemetry.fullStepActive ? 1 : 0);
  output.print(" Y_FULLSTEP=");
  output.print(status.rightDriver.telemetry.fullStepActive ? 1 : 0);
  output.print(" X_STEP_HZ=");
  output.print(status.leftDriver.stepFrequencyHz);
  output.print(" Y_STEP_HZ=");
  output.print(status.rightDriver.stepFrequencyHz);
  output.print(" X_POLL_US=");
  output.print(status.leftDriver.pollDurationUs);
  output.print(" Y_POLL_US=");
  output.print(status.rightDriver.pollDurationUs);
  output.print(" DRIVER_POLL_US=");
  output.print(status.driverPollTotalUs);
  output.print(" X_TELEMETRY=");
  output.print(status.leftDriver.telemetryCached ? "CACHED" : "LIVE");
  output.print(" Y_TELEMETRY=");
  output.print(status.rightDriver.telemetryCached ? "CACHED" : "LIVE");
  output.print(" X_TELEMETRY_AGE_MS=");
  output.print(status.leftDriver.telemetryAgeMs);
  output.print(" Y_TELEMETRY_AGE_MS=");
  output.print(status.rightDriver.telemetryAgeMs);
  output.print(" RUN_CURRENT_MA=");
  output.print(status.configuredCurrentMa);
  output.print(" MICROSTEPS=");
  output.print(status.configuredMicrosteps);
  output.print(" ACCEL_MM_S2=");
  output.print(status.configuredAccelerationMmPerSecondSquared);
  output.print(" REQUESTED_TMC_MODE=");
  output.print(chopperModeName(status.configuredChopperMode));
  output.print(" UPTIME_MS=");
  output.print(millis());
  output.print(" RX_AGE_MS=");
  output.println(millis() - lastValidCommandMs_);
}

void RobotLink::printDriverStatus(const Robot::DriverStatus& status)
{
  Print& output = transport_.output();
  const TMC2209::Status& telemetry = status.telemetry;
  if (!telemetry.connected) {
    output.print("NO_REPLY");
    return;
  }

  if (!telemetry.hasFault()) {
    output.print(status.active ? "OK_ACTIVE" : "OK_IDLE");
    return;
  }

  bool separatorRequired = false;
  const auto printFault = [&output, &separatorRequired](const char* fault) {
    if (separatorRequired) output.print(',');
    output.print(fault);
    separatorRequired = true;
  };

  if (telemetry.overTemperature) printFault("OT");
  if (telemetry.overTemperaturePreWarning) printFault("OTPW");
  if (telemetry.shortToGroundA) printFault("S2GA");
  if (telemetry.shortToGroundB) printFault("S2GB");
  if (telemetry.shortToSupplyA) printFault("S2VSA");
  if (telemetry.shortToSupplyB) printFault("S2VSB");
}

const char* RobotLink::jobResultName(Robot::JobResult result) const
{
  switch (result) {
    case Robot::JobResult::None:
      return "NONE";
    case Robot::JobResult::Ok:
      return "OK";
    case Robot::JobResult::Stopped:
      return "STOPPED";
    case Robot::JobResult::Estopped:
      return "ESTOPPED";
    case Robot::JobResult::Fault:
      return "FAULT";
    case Robot::JobResult::Replaced:
      return "REPLACED";
  }
  return "NONE";
}

const char* RobotLink::chopperModeName(TMC2209::ChopperMode mode) const
{
  return mode == TMC2209::ChopperMode::SpreadCycle ? "SPREADCYCLE" :
                                                     "STEALTHCHOP";
}
