#include "RobotLink.h"

#include <cstring>

#include "Protocol.h"

RobotLink::RobotLink(Stream& stream, Motor& leftMotor, Motor& rightMotor,
                     TMC2209& leftDriver, TMC2209& rightDriver,
                     LegacyCommandHandler legacyCommandHandler)
    : transport_(stream),
      leftMotor_(leftMotor),
      rightMotor_(rightMotor),
      leftDriver_(leftDriver),
      rightDriver_(rightDriver),
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
  const bool motorsBusy = leftMotor_.isBusy() || rightMotor_.isBusy();
  const bool motorsEnabled =
      leftMotor_.isEnabled() || rightMotor_.isEnabled();

  Print& output = transport_.output();
  output.print(Protocol::VERSION);
  output.print(" STAT ");
  output.print(sequence);
  output.print(" MODE=");
  output.print(motorsBusy ? "MOVE" : motorsEnabled ? "VELOCITY" : "IDLE");
  output.print(" ESTOP=0 FAULTS=NONE JOB=0 V_SET=0 W_SET=0 X_DRIVER=");
  printDriverStatus(leftDriver_, leftMotor_);
  output.print(" Y_DRIVER=");
  printDriverStatus(rightDriver_, rightMotor_);
  output.print(" UPTIME_MS=");
  output.print(millis());
  output.print(" RX_AGE_MS=");
  output.println(millis() - lastValidCommandMs_);
}

void RobotLink::printDriverStatus(TMC2209& driver, const Motor& motor)
{
  Print& output = transport_.output();
  if (!driver.isConnected()) {
    output.print("NO_REPLY");
    return;
  }

  const uint32_t status = driver.status();
  const uint32_t faultMask = (1UL << 25) | (1UL << 26) | (1UL << 27) |
                             (1UL << 28);
  if ((status & faultMask) == 0) {
    output.print(motor.isEnabled() ? "OK_ACTIVE" : "OK_IDLE");
    return;
  }

  bool separatorRequired = false;
  const auto printFault = [&output, &separatorRequired](const char* fault) {
    if (separatorRequired) output.print(',');
    output.print(fault);
    separatorRequired = true;
  };

  if (status & (1UL << 25)) printFault("OT");
  if (status & (1UL << 26)) printFault("OTPW");
  if (status & (1UL << 27)) printFault("S2GA");
  if (status & (1UL << 28)) printFault("S2GB");
}
