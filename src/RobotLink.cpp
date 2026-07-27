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

  Protocol::Command command;
  const Protocol::ParseResult result = Protocol::parseCommand(line, command);
  if (!result.valid) {
    sendError(result.sequence, result.errorCode);
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
        sendAck(command.sequence);
      } else {
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
        sendAck(command.sequence, jobId);
      } else {
        sendCommandError(command.sequence, commandResult);
      }
      return;
    }
    case Protocol::CommandType::Stop:
      robot_.stop();
      sendAck(command.sequence);
      return;
    case Protocol::CommandType::Estop:
      robot_.estop();
      sendAck(command.sequence);
      return;
    case Protocol::CommandType::ClearEstop: {
      const Robot::CommandResult commandResult = robot_.clearEstop();
      if (commandResult == Robot::CommandResult::Accepted) {
        sendAck(command.sequence);
      } else {
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
  const char* code = "INVALID_VALUE";
  switch (result) {
    case Robot::CommandResult::Accepted:
      return;
    case Robot::CommandResult::Busy:
      code = "BUSY";
      break;
    case Robot::CommandResult::OutOfRange:
      code = "OUT_OF_RANGE";
      break;
    case Robot::CommandResult::EstopLatched:
      code = "ESTOP_LATCHED";
      break;
    case Robot::CommandResult::DriverFault:
      code = "DRIVER_FAULT";
      break;
  }
  sendError(sequence, code);
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
  const uint32_t driverFaultMask = (1UL << 25) | (1UL << 26) | (1UL << 27) |
                                   (1UL << 28);
  const bool driverFault =
      !status.leftDriver.connected || !status.rightDriver.connected ||
      (status.leftDriver.flags & driverFaultMask) != 0 ||
      (status.rightDriver.flags & driverFaultMask) != 0;
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
  output.print(" UPTIME_MS=");
  output.print(millis());
  output.print(" RX_AGE_MS=");
  output.println(millis() - lastValidCommandMs_);
}

void RobotLink::printDriverStatus(const Robot::DriverStatus& status)
{
  Print& output = transport_.output();
  if (!status.connected) {
    output.print("NO_REPLY");
    return;
  }

  const uint32_t faultMask = (1UL << 25) | (1UL << 26) | (1UL << 27) |
                             (1UL << 28);
  if ((status.flags & faultMask) == 0) {
    output.print(status.active ? "OK_ACTIVE" : "OK_IDLE");
    return;
  }

  bool separatorRequired = false;
  const auto printFault = [&output, &separatorRequired](const char* fault) {
    if (separatorRequired) output.print(',');
    output.print(fault);
    separatorRequired = true;
  };

  if (status.flags & (1UL << 25)) printFault("OT");
  if (status.flags & (1UL << 26)) printFault("OTPW");
  if (status.flags & (1UL << 27)) printFault("S2GA");
  if (status.flags & (1UL << 28)) printFault("S2GB");
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
