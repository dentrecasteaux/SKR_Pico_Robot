#include <Arduino.h>
#include <cstdlib>
#include <cstring>
#include <cctype>

#include "Hardware.h"
#include "Config.h"
#include "Motor.h"
#include "MotionController.h"
#include "Robot.h"
#include "RobotLink.h"
#include "Stepper.h"
#include "TMC2209.h"

arduino::UART stepperUart(HW::UART_TX, HW::UART_RX);
Stepper leftStepper(HW::LEFT_STEP, HW::LEFT_DIR, HW::LEFT_EN, true);
Stepper rightStepper(HW::RIGHT_STEP, HW::RIGHT_DIR, HW::RIGHT_EN, false);
Motor leftMotor(leftStepper, Config::MAX_ACCELERATION_STEPS_PER_SECOND_SQUARED);
Motor rightMotor(rightStepper, Config::MAX_ACCELERATION_STEPS_PER_SECOND_SQUARED);
MotionController motionController(leftMotor, rightMotor);
TMC2209 leftDriver(stepperUart, 0);
TMC2209 rightDriver(stepperUart, 2);
Robot robot(leftMotor, rightMotor, motionController, leftDriver, rightDriver,
            stepperUart);

namespace
{
bool equalsIgnoreCase(const char* left, const char* right)
{
  while (*left != '\0' && *right != '\0') {
    const unsigned char leftCharacter = static_cast<unsigned char>(*left++);
    const unsigned char rightCharacter = static_cast<unsigned char>(*right++);
    if (std::tolower(leftCharacter) != std::tolower(rightCharacter)) {
      return false;
    }
  }
  return *left == '\0' && *right == '\0';
}

void printStatus()
{
  Serial.print("Left driver: ");
  Serial.println(leftMotor.isEnabled() ? "enabled" : "disabled");
  Serial.print("Left speed: ");
  Serial.println(leftMotor.speed());
  Serial.print("Right driver: ");
  Serial.println(rightMotor.isEnabled() ? "enabled" : "disabled");
  Serial.print("Right speed: ");
  Serial.println(rightMotor.speed());
}

void printHelp()
{
  Serial.println("Commands: left/right on/off/test/forward/reverse/speed N, drive LINEAR TURN, velocity MM_PER_S DEG_PER_S, move DISTANCE_MM, turn ANGLE_DEGREES, drivers, configuration, configure CURRENT_MA MICROSTEPS ACCEL_MM_S2 MODE, off, status, help");
}

void printConfiguration()
{
  const Robot::Status status = robot.status();
  Serial.print("Configuration: current=");
  Serial.print(status.configuredCurrentMa);
  Serial.print(" mA, microsteps=");
  Serial.print(status.configuredMicrosteps);
  Serial.print(", acceleration=");
  Serial.print(status.configuredAccelerationMmPerSecondSquared);
  Serial.print(" mm/s^2, mode=");
  Serial.println(status.configuredChopperMode ==
                         TMC2209::ChopperMode::SpreadCycle
                     ? "spreadCycle"
                     : "stealthChop");
}

bool processConfigureCommand(const char* command)
{
  constexpr const char* prefix = "configure ";
  if (std::strncmp(command, prefix, std::strlen(prefix)) != 0) return false;

  unsigned int currentMa = 0;
  unsigned int microsteps = 0;
  float acceleration = 0.0F;
  char arguments[96];
  std::strncpy(arguments, command + std::strlen(prefix), sizeof(arguments));
  arguments[sizeof(arguments) - 1] = '\0';
  char* savePosition = nullptr;
  char* currentText = ::strtok_r(arguments, " ", &savePosition);
  char* microstepsText = ::strtok_r(nullptr, " ", &savePosition);
  char* accelerationText = ::strtok_r(nullptr, " ", &savePosition);
  char* modeText = ::strtok_r(nullptr, " ", &savePosition);
  char* extraText = ::strtok_r(nullptr, " ", &savePosition);
  if (currentText == nullptr || microstepsText == nullptr ||
      accelerationText == nullptr || modeText == nullptr ||
      extraText != nullptr) {
    Serial.println("Use: configure CURRENT_MA MICROSTEPS ACCEL_MM_S2 stealthchop|spreadcycle");
    return true;
  }

  char* end = nullptr;
  const unsigned long parsedCurrent = std::strtoul(currentText, &end, 10);
  if (end == currentText || *end != '\0') {
    Serial.println("Current and microsteps must be whole numbers.");
    return true;
  }
  currentMa = static_cast<unsigned int>(parsedCurrent);
  const unsigned long parsedMicrosteps =
      std::strtoul(microstepsText, &end, 10);
  if (end == microstepsText || *end != '\0') {
    Serial.println("Current and microsteps must be whole numbers.");
    return true;
  }
  microsteps = static_cast<unsigned int>(parsedMicrosteps);
  acceleration = std::strtof(accelerationText, &end);
  if (end == accelerationText || *end != '\0') {
    Serial.println("Acceleration must be a number.");
    return true;
  }

  TMC2209::ChopperMode mode;
  if (equalsIgnoreCase(modeText, "stealthchop") ||
      equalsIgnoreCase(modeText, "stealth")) {
    mode = TMC2209::ChopperMode::StealthChop;
  } else if (equalsIgnoreCase(modeText, "spreadcycle") ||
             equalsIgnoreCase(modeText, "spread")) {
    mode = TMC2209::ChopperMode::SpreadCycle;
  } else {
    Serial.print("Mode token received: '");
    Serial.print(modeText);
    Serial.println("' (configuration parser v2)");
    Serial.println("Mode must be stealthchop or spreadcycle.");
    return true;
  }

  const Robot::CommandResult result =
      robot.configure(currentMa, microsteps, acceleration, mode);
  if (result == Robot::CommandResult::Accepted) {
    Serial.println("Configuration applied to both drivers.");
    printConfiguration();
  } else if (result == Robot::CommandResult::Busy) {
    Serial.println("Configuration rejected: robot must be idle.");
  } else if (result == Robot::CommandResult::OutOfRange) {
    Serial.println("Configuration rejected: value outside the safe range.");
  } else {
    Serial.println("Configuration rejected: driver write/readback failed.");
  }
  return true;
}

void printDriverHealth(const char* name, const Robot::DriverStatus& driver)
{
  Serial.print(name);
  Serial.print(": ");
  const TMC2209::Status& status = driver.telemetry;
  if (!status.connected) {
    Serial.print(driver.telemetryCached ? "no cached telemetry" :
                                          "not responding");
    Serial.print(", poll=");
    Serial.print(driver.pollDurationUs);
    Serial.println(" us");
    return;
  }

  if (!status.hasFault()) {
    if (!driver.active) {
      Serial.print("OK (idle)");
    } else {
      Serial.print("OK (active)");
    }
    Serial.print(", CS=");
    Serial.print(status.currentScale);
    Serial.print(", mode=");
    Serial.print(status.stealthChop ? "stealthChop" : "spreadCycle");
    Serial.print(", fullstep=");
    Serial.print(status.fullStepActive ? 1 : 0);
    Serial.print(", STEP=");
    Serial.print(driver.stepFrequencyHz);
    Serial.print(" Hz, poll=");
    Serial.print(driver.pollDurationUs);
    Serial.print(" us, telemetry=");
    Serial.print(driver.telemetryCached ? "cached" : "live");
    if (driver.telemetryCached) {
      Serial.print(", age=");
      Serial.print(driver.telemetryAgeMs);
      Serial.print(" ms");
    }
    Serial.println();
    return;
  }

  if (status.overTemperature) Serial.print("over-temperature ");
  if (status.overTemperaturePreWarning) Serial.print("temperature-warning ");
  if (status.shortToGroundA) Serial.print("short-to-ground-A ");
  if (status.shortToGroundB) Serial.print("short-to-ground-B ");
  if (status.shortToSupplyA) Serial.print("low-side-short-A ");
  if (status.shortToSupplyB) Serial.print("low-side-short-B ");
  Serial.print("poll=");
  Serial.print(driver.pollDurationUs);
  Serial.println(" us");
}

void printDriverHealth()
{
  const Robot::Status status = robot.status();
  printDriverHealth("X driver", status.leftDriver);
  printDriverHealth("Y driver", status.rightDriver);
  Serial.print("Both drivers: poll=");
  Serial.print(status.driverPollTotalUs);
  Serial.println(" us total");
}

bool processTurnCommand(const char* command)
{
  constexpr const char* prefix = "turn ";
  constexpr size_t prefixLength = 5;
  if (std::strncmp(command, prefix, prefixLength) != 0) return false;

  char* end = nullptr;
  const float angleDegrees = std::strtof(command + prefixLength, &end);
  if (end == command + prefixLength || *end != '\0' || angleDegrees == 0.0F) {
    Serial.println("Use: turn ANGLE_DEGREES");
    return true;
  }

  if (!robot.startTurn(angleDegrees)) {
    Serial.println("Unable to start turn: motors are busy or angle is invalid.");
    return true;
  }

  Serial.println("Angle turn started.");
  return true;
}

bool processMoveCommand(const char* command)
{
  constexpr const char* prefix = "move ";
  constexpr size_t prefixLength = 5;
  if (std::strncmp(command, prefix, prefixLength) != 0) return false;

  char* end = nullptr;
  const float distanceMm = std::strtof(command + prefixLength, &end);
  if (end == command + prefixLength || *end != '\0' || distanceMm == 0.0F) {
    Serial.println("Use: move DISTANCE_MM");
    return true;
  }

  if (!robot.startMove(distanceMm)) {
    Serial.println("Unable to start move: motors are busy or distance is invalid.");
    return true;
  }

  Serial.println("Distance move started.");
  return true;
}

bool processVelocityCommand(const char* command)
{
  constexpr const char* prefix = "velocity ";
  constexpr size_t prefixLength = 9;
  if (std::strncmp(command, prefix, prefixLength) != 0) return false;

  const char* linearText = command + prefixLength;
  char* end = nullptr;
  const float linearMmPerSecond = std::strtof(linearText, &end);
  if (end == linearText || *end != ' ') {
    Serial.println("Use: velocity MM_PER_S DEG_PER_S");
    return true;
  }

  while (*end == ' ') ++end;
  const char* turnText = end;
  const float turnDegreesPerSecond = std::strtof(turnText, &end);
  if (end == turnText || *end != '\0') {
    Serial.println("Use: velocity MM_PER_S DEG_PER_S");
    return true;
  }

  if (linearMmPerSecond < -Config::MAX_LINEAR_SPEED_MM_PER_SECOND ||
      linearMmPerSecond > Config::MAX_LINEAR_SPEED_MM_PER_SECOND ||
      turnDegreesPerSecond < -Config::MAX_TURN_SPEED_DEGREES_PER_SECOND ||
      turnDegreesPerSecond > Config::MAX_TURN_SPEED_DEGREES_PER_SECOND) {
    Serial.println("Velocity is outside the current safety limits.");
    return true;
  }

  robot.setVelocity(linearMmPerSecond, turnDegreesPerSecond);
  printStatus();
  return true;
}

bool processDriveCommand(const char* command)
{
  constexpr const char* prefix = "drive ";
  constexpr size_t prefixLength = 6;
  if (std::strncmp(command, prefix, prefixLength) != 0) return false;

  const char* linearText = command + prefixLength;
  char* end = nullptr;
  const long linear = std::strtol(linearText, &end, 10);
  if (end == linearText || *end != ' ') {
    Serial.println("Use: drive LINEAR TURN");
    return true;
  }

  while (*end == ' ') ++end;
  const char* turnText = end;
  const long turn = std::strtol(turnText, &end, 10);
  if (end == turnText || *end != '\0' ||
      linear < -Config::MAX_SPEED_STEPS_PER_SECOND ||
      linear > Config::MAX_SPEED_STEPS_PER_SECOND ||
      turn < -Config::MAX_SPEED_STEPS_PER_SECOND ||
      turn > Config::MAX_SPEED_STEPS_PER_SECOND) {
    Serial.print("Linear and turn must each be between -");
    Serial.print(Config::MAX_SPEED_STEPS_PER_SECOND);
    Serial.print(" and ");
    Serial.print(Config::MAX_SPEED_STEPS_PER_SECOND);
    Serial.println('.');
    return true;
  }

  motionController.setDrive(static_cast<int32_t>(linear),
                            static_cast<int32_t>(turn));
  printStatus();
  return true;
}

bool processSpeedCommand(const char* command, const char* prefix, Motor& motor,
                         const char* label)
{
  const size_t prefixLength = std::strlen(prefix);
  if (std::strncmp(command, prefix, prefixLength) != 0) return false;

  char* end = nullptr;
  const char* value = command + prefixLength;
  const long requestedSpeed = std::strtol(value, &end, 10);
  if (end == value || *end != '\0' ||
      requestedSpeed < -Config::MAX_SPEED_STEPS_PER_SECOND ||
      requestedSpeed > Config::MAX_SPEED_STEPS_PER_SECOND) {
    Serial.print("Speed must be between -");
    Serial.print(Config::MAX_SPEED_STEPS_PER_SECOND);
    Serial.print(" and ");
    Serial.println(Config::MAX_SPEED_STEPS_PER_SECOND);
    return true;
  }

  motor.setSpeed(static_cast<int32_t>(requestedSpeed));
  Serial.print(label);
  Serial.print(" speed set to ");
  Serial.println(requestedSpeed);
  return true;
}

void processCommand(const char* command)
{
  if (processConfigureCommand(command)) {
    return;
  } else if (processTurnCommand(command)) {
    return;
  } else if (processMoveCommand(command)) {
    return;
  } else if (processVelocityCommand(command)) {
    return;
  } else if (processDriveCommand(command)) {
    return;
  } else if (processSpeedCommand(command, "left speed ", leftMotor, "Left")) {
    return;
  } else if (processSpeedCommand(command, "right speed ", rightMotor, "Right")) {
    return;
  } else if (std::strcmp(command, "left on") == 0) {
    leftMotor.enable();
  } else if (std::strcmp(command, "left off") == 0) {
    leftMotor.disable();
  } else if (std::strcmp(command, "left test") == 0) {
    if (!leftMotor.startTest(Config::TEST_STEP_COUNT,
                             Config::TEST_STEP_INTERVAL_US)) {
      Serial.println("Left motor is already moving.");
      return;
    }
    Serial.println("Left 10-pulse test started.");
    return;
  } else if (std::strcmp(command, "left forward") == 0) {
    if (!leftMotor.startMove(Config::MOTION_TEST_STEP_COUNT, true,
                             Config::MOTION_TEST_STEP_INTERVAL_US)) {
      Serial.println("Left motor is already moving.");
      return;
    }
    Serial.println("Left forward test started.");
    return;
  } else if (std::strcmp(command, "left reverse") == 0) {
    if (!leftMotor.startMove(Config::MOTION_TEST_STEP_COUNT, false,
                             Config::MOTION_TEST_STEP_INTERVAL_US)) {
      Serial.println("Left motor is already moving.");
      return;
    }
    Serial.println("Left reverse test started.");
    return;
  } else if (std::strcmp(command, "right on") == 0) {
    rightMotor.enable();
  } else if (std::strcmp(command, "right off") == 0) {
    rightMotor.disable();
  } else if (std::strcmp(command, "right test") == 0) {
    if (!rightMotor.startTest(Config::TEST_STEP_COUNT,
                              Config::TEST_STEP_INTERVAL_US)) {
      Serial.println("Right motor is already moving.");
      return;
    }
    Serial.println("Right 10-pulse test started.");
    return;
  } else if (std::strcmp(command, "right forward") == 0) {
    if (!rightMotor.startMove(Config::MOTION_TEST_STEP_COUNT, true,
                              Config::MOTION_TEST_STEP_INTERVAL_US)) {
      Serial.println("Right motor is already moving.");
      return;
    }
    Serial.println("Right forward test started.");
    return;
  } else if (std::strcmp(command, "right reverse") == 0) {
    if (!rightMotor.startMove(Config::MOTION_TEST_STEP_COUNT, false,
                              Config::MOTION_TEST_STEP_INTERVAL_US)) {
      Serial.println("Right motor is already moving.");
      return;
    }
    Serial.println("Right reverse test started.");
    return;
  } else if (std::strcmp(command, "off") == 0) {
    robot.stop();
  } else if (std::strcmp(command, "status") == 0) {
    printStatus();
    return;
  } else if (std::strcmp(command, "drivers") == 0) {
    printDriverHealth();
    return;
  } else if (std::strcmp(command, "configuration") == 0) {
    printConfiguration();
    return;
  } else if (std::strcmp(command, "help") == 0) {
    printHelp();
    return;
  } else if (command[0] != '\0') {
    Serial.println("Unknown command. Type help.");
    return;
  }

  printStatus();
}

}  // namespace

RobotLink robotLink(Serial, robot, processCommand);

void setup()
{
  Serial.begin(Config::SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }

  robot.begin();

  Serial.println("SKR Pico starting...");
  Serial.println("Control build: configuration-parser-v2");
  Serial.println("STEP engine: RP2040 PIO (one state machine per motor).");
  Serial.println("Stepper drivers initialised and disabled.");
  Serial.print("X TMC2209 UART: ");
  Serial.println(leftDriver.isConnected() ? "connected" : "not responding");
  Serial.print("Y TMC2209 UART: ");
  Serial.println(rightDriver.isConnected() ? "connected" : "not responding");
  printHelp();
}

void loop()
{
  robotLink.update();
  robot.update();
}
