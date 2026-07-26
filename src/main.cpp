#include <Arduino.h>
#include <cstdlib>
#include <cstring>

#include "Hardware.h"
#include "Config.h"
#include "Motor.h"
#include "MotionController.h"
#include "Stepper.h"
#include "TMC2209.h"

arduino::UART stepperUart(HW::UART_TX, HW::UART_RX);
Stepper leftStepper(HW::LEFT_STEP, HW::LEFT_DIR, HW::LEFT_EN, true);
Stepper rightStepper(HW::RIGHT_STEP, HW::RIGHT_DIR, HW::RIGHT_EN, false);
Motor leftMotor(leftStepper);
Motor rightMotor(rightStepper);
MotionController motionController(leftMotor, rightMotor);
TMC2209 leftDriver(stepperUart, 0);
TMC2209 rightDriver(stepperUart, 2);

namespace
{
constexpr size_t COMMAND_BUFFER_SIZE = 32;
char commandBuffer[COMMAND_BUFFER_SIZE];
size_t commandLength = 0;

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
  Serial.println("Commands: left/right on/off/test/forward/reverse/speed N, drive LINEAR TURN, off, status, help");
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
    Serial.println("Linear and turn must each be between -400 and 400.");
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
  if (processDriveCommand(command)) {
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
    motionController.stop();
  } else if (std::strcmp(command, "status") == 0) {
    printStatus();
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

void processSerialCommands()
{
  while (Serial.available() > 0) {
    const char received = static_cast<char>(Serial.read());

    if (received == '\r') {
      continue;
    }

    if (received == '\n') {
      commandBuffer[commandLength] = '\0';
      processCommand(commandBuffer);
      commandLength = 0;
      continue;
    }

    if (commandLength < COMMAND_BUFFER_SIZE - 1) {
      commandBuffer[commandLength++] = received;
    }
  }
}
}  // namespace

void setup()
{
  leftMotor.begin();
  rightMotor.begin();

  stepperUart.begin(Config::TMC_UART_BAUD);
  leftDriver.begin();
  rightDriver.begin();

  Serial.begin(Config::SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }

  Serial.println("SKR Pico starting...");
  Serial.println("Stepper drivers initialised and disabled.");
  Serial.print("X TMC2209 UART: ");
  Serial.println(leftDriver.isConnected() ? "connected" : "not responding");
  Serial.print("Y TMC2209 UART: ");
  Serial.println(rightDriver.isConnected() ? "connected" : "not responding");
  printHelp();
}

void loop()
{
  processSerialCommands();
  const uint32_t nowUs = micros();
  leftMotor.update(nowUs);
  rightMotor.update(nowUs);
}
