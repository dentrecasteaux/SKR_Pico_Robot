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

void RobotLink::sendAck(uint32_t sequence)
{
  Print& output = transport_.output();
  output.print(Protocol::VERSION);
  output.print(" ACK ");
  output.print(sequence);
  output.println(" OK");
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
  }

  Print& output = transport_.output();
  output.print(Protocol::VERSION);
  output.print(" STAT ");
  output.print(sequence);
  output.print(" MODE=");
  output.print(mode);
  output.print(" ESTOP=");
  output.print(status.estopLatched ? 1 : 0);
  output.print(" FAULTS=NONE JOB=");
  output.print(status.activeJob);
  output.print(" V_SET=");
  output.print(status.linearSetpointMmPerSecond);
  output.print(" W_SET=");
  output.print(status.turnSetpointDegreesPerSecond);
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
