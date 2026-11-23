#include "RobotLink.h"

#include <cstring>

#include "Balance.h"
#include "Config.h"
#include "Imu.h"
#include "Protocol.h"

RobotLink::RobotLink(Stream& stream, Robot& robot, Telemetry::Logger& logger,
                     LegacyCommandHandler legacyCommandHandler)
    : transport_(stream),
      robot_(robot),
      logger_(logger),
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
      // While balancing, SET_VELOCITY sets the balance controller's drive
      // set points (same units and lease), so every client can drive it.
      if (robot_.mode() == Robot::Mode::Balance) {
        if (Balance::setDrive(command.linearMmPerSecond / 1000.0F,
                              command.turnDegreesPerSecond, command.leaseMs)) {
          sendAck(command.sequence);
        } else {
          sendError(command.sequence, "OUT_OF_RANGE");
        }
        return;
      }
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
      // While balancing, STOP means "stop moving and keep balancing": ending
      // balancing would drop the robot onto a training wheel. BAL_STOP and
      // ESTOP end balancing. (robotd also sends STOP when a drive hold
      // expires.) STOP while arming cancels the arming.
      if (Balance::currentState() == Balance::State::Running) {
        Balance::setDrive(0.0F, 0.0F, 100);
        cacheReply(originalLine, command.sequence, true);
        sendAck(command.sequence);
        return;
      }
      Balance::stop(Balance::Reason::Command);
      robot_.stop();
      cacheReply(originalLine, command.sequence, true);
      sendAck(command.sequence);
      return;
    case Protocol::CommandType::Estop:
      Balance::stop(Balance::Reason::Estop);
      robot_.estop();
      logger_.freeze("ESTOP");
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
    case Protocol::CommandType::Imu:
    case Protocol::CommandType::ImuCalibrate:
    case Protocol::CommandType::ImuZeroYaw:
    case Protocol::CommandType::ImuSet:
    case Protocol::CommandType::ImuWindowStart:
    case Protocol::CommandType::ImuWindow:
      processImuCommand(command);
      return;
    case Protocol::CommandType::Hold: {
      const Robot::CommandResult commandResult =
          robot_.hold(command.leaseMs);
      if (commandResult == Robot::CommandResult::Accepted) {
        sendAck(command.sequence);
      } else {
        sendCommandError(command.sequence, commandResult);
      }
      return;
    }
    case Protocol::CommandType::LogStart:
    case Protocol::CommandType::LogStop:
    case Protocol::CommandType::LogStatus:
    case Protocol::CommandType::BbFreeze:
    case Protocol::CommandType::BbArm:
    case Protocol::CommandType::BbDump:
      processLogCommand(command);
      return;
    case Protocol::CommandType::BalSet:
    case Protocol::CommandType::BalArm:
    case Protocol::CommandType::BalStop:
    case Protocol::CommandType::BalStatus:
      processBalanceCommand(command);
      return;
  }
}

// Balance commands. BAL_SET changes parameters (allowed while balancing, for
// tuning); BAL_ARM hands the motors to core 1; BAL_STOP stops it.
void RobotLink::processBalanceCommand(const Protocol::Command& command)
{
  const uint32_t sequence = command.sequence;
  if (!Imu::enabled()) {
    sendError(sequence, "UNSUPPORTED_COMMAND");
    return;
  }
  switch (command.type) {
    case Protocol::CommandType::BalSet: {
      Balance::Params p = Balance::params();
      for (size_t i = 0; i < command.pairCount; ++i) {
        const char* k = command.pairs[i].key;
        const float v = command.pairs[i].value;
        if (std::strcmp(k, "KP") == 0) p.kp = v;
        else if (std::strcmp(k, "KD") == 0) p.kd = v;
        else if (std::strcmp(k, "KV") == 0) p.kv = v;
        else if (std::strcmp(k, "KX") == 0) p.kx = v;
        else if (std::strcmp(k, "TRIM") == 0) p.trimDeg = v;
        else if (std::strcmp(k, "AMAX") == 0) p.aMax = v;
        else if (std::strcmp(k, "VMAX") == 0) p.vMax = v;
        else if (std::strcmp(k, "SOFT") == 0) p.softStartS = v;
        else if (std::strcmp(k, "TILT_FWD") == 0) p.tiltFwdDeg = v;
        else if (std::strcmp(k, "TILT_BACK") == 0) p.tiltBackDeg = v;
        else if (std::strcmp(k, "ARM_WINDOW") == 0) p.armWindowDeg = v;
        else if (std::strcmp(k, "ARM_RATE") == 0) p.armRateDps = v;
        else if (std::strcmp(k, "ARM_TIMEOUT") == 0) p.armTimeoutS = v;
        else if (std::strcmp(k, "RUNAWAY") == 0) p.runawayS = v;
        else if (std::strcmp(k, "ACCEL_COMP") == 0) p.accelComp = v != 0.0F;
        else if (std::strcmp(k, "TRIM_RATE") == 0) p.trimRate = v;
        else if (std::strcmp(k, "TRIM_RANGE") == 0) p.trimRangeDeg = v;
        else if (std::strcmp(k, "DRIVE_ACCEL") == 0) p.driveAccel = v;
        else {
          sendError(sequence, "UNKNOWN_FIELD");
          return;
        }
      }
      if (!Balance::setParams(p)) {
        sendError(sequence, "OUT_OF_RANGE");
        return;
      }
      sendBalanceStatus(sequence);
      return;
    }
    case Protocol::CommandType::BalArm: {
      bool dry = false;
      for (size_t i = 0; i < command.pairCount; ++i) {
        if (std::strcmp(command.pairs[i].key, "DRY") == 0) {
          dry = command.pairs[i].value != 0.0F;
        } else {
          sendError(sequence, "UNKNOWN_FIELD");
          return;
        }
      }
      if (Balance::active()) {
        sendError(sequence, "BUSY");
        return;
      }
      const Robot::CommandResult entered = robot_.enterBalance();
      if (entered != Robot::CommandResult::Accepted) {
        sendCommandError(sequence, entered);
        return;
      }
      const float stepsPerMetre =
          Config::MOTOR_FULL_STEPS_PER_REVOLUTION * robot_.microsteps() /
          (3.14159265F * Config::WHEEL_DIAMETER_MM / 1000.0F);
      if (!Balance::arm(dry, stepsPerMetre)) {
        robot_.exitBalance();
        sendError(sequence, "BUSY");
        return;
      }
      logger_.arm();   // fresh black box for this attempt
      sendBalanceStatus(sequence);
      return;
    }
    case Protocol::CommandType::BalStop:
      Balance::stop(Balance::Reason::Command);
      sendBalanceStatus(sequence);
      return;
    case Protocol::CommandType::BalStatus:
      sendBalanceStatus(sequence);
      return;
    default:
      sendError(sequence, "UNKNOWN_COMMAND");
      return;
  }
}

void RobotLink::sendBalanceStatus(uint32_t sequence)
{
  const Balance::Status s = Balance::status();
  const Balance::Params p = Balance::params();
  Print& o = transport_.output();
  const auto f = [&o](const char* key, float value, int digits) {
    o.print(' ');
    o.print(key);
    o.print('=');
    o.print(value, digits);
  };
  const auto n = [&o](const char* key, uint32_t value) {
    o.print(' ');
    o.print(key);
    o.print('=');
    o.print(value);
  };
  o.print(Protocol::VERSION);
  o.print(" ACK ");
  o.print(sequence);
  o.print(" OK BAL=");
  o.print(Balance::stateName(s.state));
  o.print(" REASON=");
  o.print(Balance::reasonName(s.reason));
  n("DRY", s.dry ? 1 : 0);
  f("THETA", s.thetaDeg, 2);
  f("ERR", s.errorDeg, 2);
  f("RATE", s.rateDps, 1);
  f("U", s.u, 3);
  f("V", s.v, 3);
  f("X", s.x, 3);
  f("TRIM_EST", s.trimEstDeg, 2);
  f("V_REF", s.vRef, 3);
  f("W_REF", s.wRefDps, 1);
  f("X_REF", s.xRef, 3);
  f("SCALE", s.scale, 2);
  n("RUN_MS", s.runMs);
  n("ENGAGES", s.engages);
  n("TRIPS", s.trips);
  f("MAX_U", s.maxAbsU, 2);
  f("MAX_ERR", s.maxAbsErrorDeg, 2);
  n("STEP_HZ", s.stepRateSps);
  f("GYRO_QR", Imu::yawPitchCoupling(), 4);
  f("KP", p.kp, 3);
  f("KD", p.kd, 3);
  f("KV", p.kv, 3);
  f("KX", p.kx, 3);
  f("TRIM", p.trimDeg, 2);
  f("AMAX", p.aMax, 2);
  f("VMAX", p.vMax, 3);
  f("SOFT", p.softStartS, 2);
  f("TILT_FWD", p.tiltFwdDeg, 2);
  f("TILT_BACK", p.tiltBackDeg, 2);
  f("ARM_WINDOW", p.armWindowDeg, 2);
  f("ARM_RATE", p.armRateDps, 1);
  f("ARM_TIMEOUT", p.armTimeoutS, 1);
  f("RUNAWAY", p.runawayS, 2);
  n("ACCEL_COMP", p.accelComp ? 1 : 0);
  f("TRIM_RATE", p.trimRate, 2);
  f("TRIM_RANGE", p.trimRangeDeg, 1);
  f("DRIVE_ACCEL", p.driveAccel, 2);
  o.println();
}

// Telemetry commands never move the motors: accepted in every mode and not
// part of the motion-reply replay cache. Replies are ACKs with fields.
void RobotLink::processLogCommand(const Protocol::Command& command)
{
  const uint32_t sequence = command.sequence;
  switch (command.type) {
    case Protocol::CommandType::LogStart:
      if (!logger_.startLog(command.logDivisor)) {
        sendError(sequence, "OUT_OF_RANGE");
        return;
      }
      sendLogStatus(sequence);
      return;
    case Protocol::CommandType::LogStop:
      logger_.stopLog();
      sendLogStatus(sequence);
      return;
    case Protocol::CommandType::LogStatus:
      sendLogStatus(sequence);
      return;
    case Protocol::CommandType::BbFreeze:
      logger_.freeze("COMMAND");
      sendLogStatus(sequence);
      return;
    case Protocol::CommandType::BbArm:
      if (!logger_.arm()) {
        sendError(sequence, "BUSY");
        return;
      }
      sendLogStatus(sequence);
      return;
    case Protocol::CommandType::BbDump:
      if (!logger_.startDump()) {
        // Not frozen, or a LOG stream is running (stop it first).
        sendError(sequence, "BUSY");
        return;
      }
      sendLogStatus(sequence);
      return;
    default:
      sendError(sequence, "UNKNOWN_COMMAND");
      return;
  }
}

void RobotLink::sendLogStatus(uint32_t sequence)
{
  const Telemetry::Stats s = logger_.stats();
  Print& output = transport_.output();
  output.print(Protocol::VERSION);
  output.print(" ACK ");
  output.print(sequence);
  output.print(" OK LOG=");
  output.print(s.logging ? 1 : 0);
  output.print(" DIV=");
  output.print(s.divisor);
  output.print(" SEQ=");
  output.print(s.seq);
  output.print(" SENT=");
  output.print(s.sent);
  output.print(" STREAM_DROPS=");
  output.print(s.streamDrops);
  output.print(" IMU_DROPS=");
  output.print(s.imuDrops);
  output.print(" IMU_DROPS_TOTAL=");
  output.print(s.imuDropsTotal);
  output.print(" BB=");
  output.print(Telemetry::Logger::bbStateName(s.bbState));
  output.print(" BB_COUNT=");
  output.print(s.bbCount);
  output.print(" BB_DUMPED=");
  output.print(s.bbDumped);
  output.print(" BB_REASON=");
  output.print(s.bbReason[0] != '\0' ? s.bbReason : "NONE");
  output.print(" BB_FROZEN_T_US=");
  output.println(s.bbFrozenAtUs);
}

// IMU commands never move the motors, so they are accepted in every mode
// (including ESTOP) and are not part of the motion-reply replay cache.
void RobotLink::processImuCommand(const Protocol::Command& command)
{
  const uint32_t sequence = command.sequence;
  if (!Imu::enabled()) {
    sendError(sequence, "UNSUPPORTED_COMMAND");
    return;
  }

  switch (command.type) {
    case Protocol::CommandType::Imu:
      sendImu(sequence);
      return;
    case Protocol::CommandType::ImuCalibrate:
      // Calibration needs the robot still.
      if (robot_.motorsBusy()) {
        sendError(sequence, "BUSY");
        return;
      }
      if (!Imu::requestCalibration()) {
        sendError(sequence, "IMU_UNAVAILABLE");
        return;
      }
      sendAck(sequence);
      return;
    case Protocol::CommandType::ImuZeroYaw:
      Imu::zeroYaw();
      sendAck(sequence);
      return;
    case Protocol::CommandType::ImuSet:
      if (command.hasTau && !Imu::setTau(command.tauSeconds)) {
        sendError(sequence, "OUT_OF_RANGE");
        return;
      }
      if (command.hasAutoBias) Imu::setAutoBias(command.autoBias);
      sendAck(sequence);
      return;
    case Protocol::CommandType::ImuWindowStart:
      sendAckField(sequence, "WINDOW", Imu::startWindow());
      return;
    case Protocol::CommandType::ImuWindow:
      sendImuWindow(sequence);
      return;
    default:
      sendError(sequence, "UNKNOWN_COMMAND");
      return;
  }
}

void RobotLink::sendAckField(uint32_t sequence, const char* key,
                             uint32_t value)
{
  Print& output = transport_.output();
  output.print(Protocol::VERSION);
  output.print(" ACK ");
  output.print(sequence);
  output.print(" OK ");
  output.print(key);
  output.print('=');
  output.println(value);
}

void RobotLink::sendImu(uint32_t sequence)
{
  Imu::Snapshot s;
  Imu::snapshot(s);
  Print& output = transport_.output();
  const auto field = [&output](const char* key, float value, int digits) {
    output.print(' ');
    output.print(key);
    output.print('=');
    output.print(value, digits);
  };
  const auto count = [&output](const char* key, uint32_t value) {
    output.print(' ');
    output.print(key);
    output.print('=');
    output.print(value);
  };

  output.print(Protocol::VERSION);
  output.print(" IMU ");
  output.print(sequence);
  output.print(" STATE=");
  output.print(Imu::stateName(s.state));
  output.print(" CAL=");
  output.print(Imu::calName(s.cal));
  count("CAL_ATTEMPTS", s.calAttempts);
  output.print(" WHO_AM_I=0x");
  output.print(s.whoAmI, HEX);
  field("ROLL", s.roll, 2);
  field("PITCH", s.pitch, 2);
  field("YAW", s.yaw, 2);
  field("P", s.p, 2);
  field("Q", s.q, 2);
  field("R", s.r, 2);
  field("AX", s.ax, 3);
  field("AY", s.ay, 3);
  field("AZ", s.az, 3);
  field("TEMP_C", s.tempC, 1);
  field("TAU", s.tau, 2);
  count("AUTO_BIAS", s.autoBias ? 1 : 0);
  count("MOTORS_STOPPED", s.motorsStopped ? 1 : 0);
  count("AUTO_UPDATES", s.autoUpdates);
  field("BIAS_X", s.bias[0], 3);
  field("BIAS_Y", s.bias[1], 3);
  field("BIAS_Z", s.bias[2], 3);
  count("SAMPLE_HZ", s.sampleHz);
  count("SAMPLES", s.samples);
  count("ERRORS", s.errors);
  count("STALLS", s.stalls);
  count("LATE", s.late);
  count("ACC_REJECTS", s.accRejects);
  count("AGE_MS", s.ageMs);
  count("WINDOW", s.windowId);
  output.println();
}

void RobotLink::sendImuWindow(uint32_t sequence)
{
  Imu::WindowStats w;
  Imu::windowStats(w);
  Print& output = transport_.output();
  const auto field = [&output](const char* key, float value, int digits) {
    output.print(' ');
    output.print(key);
    output.print('=');
    output.print(value, digits);
  };
  const auto count = [&output](const char* key, uint32_t value) {
    output.print(' ');
    output.print(key);
    output.print('=');
    output.print(value);
  };
  static const char* const NAMES[Imu::CHANNELS] = {"GX", "GY", "GZ",
                                                   "AX", "AY", "AZ"};

  output.print(Protocol::VERSION);
  output.print(" IMUW ");
  output.print(sequence);
  count("WINDOW", w.id);
  count("SAMPLES", w.samples);
  count("DURATION_MS", w.durationMs);
  count("MOVING_SAMPLES", w.movingSamples);
  char key[12];
  for (int i = 0; i < Imu::CHANNELS; ++i) {
    snprintf(key, sizeof(key), "%s_MEAN", NAMES[i]);
    field(key, w.mean[i], 4);
    snprintf(key, sizeof(key), "%s_SD", NAMES[i]);
    field(key, w.sd[i], 4);
    snprintf(key, sizeof(key), "%s_MIN", NAMES[i]);
    field(key, w.min[i], 3);
    snprintf(key, sizeof(key), "%s_MAX", NAMES[i]);
    field(key, w.max[i], 3);
  }
  field("AMAG_MIN", w.accelMagnitudeMin, 3);
  field("AMAG_MAX", w.accelMagnitudeMax, 3);
  field("ROLL_START", w.rollStart, 2);
  field("PITCH_START", w.pitchStart, 2);
  field("ROLL_MIN", w.rollMin, 2);
  field("ROLL_MAX", w.rollMax, 2);
  field("PITCH_MIN", w.pitchMin, 2);
  field("PITCH_MAX", w.pitchMax, 2);
  field("YAW_CHANGE", w.yawChange, 2);
  count("LATE", w.late);
  count("ERRORS", w.errors);
  count("STALLS", w.stalls);
  count("ACC_REJECTS", w.accRejects);
  count("GAP_MIN_US", w.gapMinUs);
  count("GAP_MAX_US", w.gapMaxUs);
  count("LAT_MAX_US", w.latencyMaxUs);
  count("FILTER_MAX_US", w.filterMaxUs);
  count("LOOP_MAX_US", w.loopMaxUs);
  output.println();
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
    case Robot::CommandResult::BalanceTripped:
      return "BALANCE_TRIPPED";
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
    case Robot::Mode::Hold:
      mode = "HOLD";
      break;
    case Robot::Mode::Balance:
      mode = "BALANCE";
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
  output.print(" DRIVE_LOCK=");
  output.print(robot_.balanceTripLockout() ? 1 : 0);
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
  Imu::Snapshot imu;
  Imu::snapshot(imu);
  output.print(" IMU=");
  output.print(Imu::stateName(imu.state));
  if (imu.state == Imu::State::Ok) {
    output.print(" IMU_ROLL=");
    output.print(imu.roll, 2);
    output.print(" IMU_PITCH=");
    output.print(imu.pitch, 2);
    output.print(" IMU_YAW=");
    output.print(imu.yaw, 2);
  }
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
