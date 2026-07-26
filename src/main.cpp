#include <Arduino.h>
#include <cstring>

#include "Hardware.h"
#include "Config.h"
#include "Stepper.h"
#include "TMC2209.h"

arduino::UART stepperUart(HW::UART_TX, HW::UART_RX);
Stepper leftStepper(HW::LEFT_STEP, HW::LEFT_DIR, HW::LEFT_EN, true);
Stepper rightStepper(HW::RIGHT_STEP, HW::RIGHT_DIR, HW::RIGHT_EN, false);
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
  Serial.println(leftStepper.isEnabled() ? "enabled" : "disabled");
  Serial.print("Right driver: ");
  Serial.println(rightStepper.isEnabled() ? "enabled" : "disabled");
}

void printHelp()
{
  Serial.println("Commands: left or right on/off/test/forward/reverse, off, status, help");
}

void processCommand(const char* command)
{
  if (std::strcmp(command, "left on") == 0) {
    leftStepper.enable();
  } else if (std::strcmp(command, "left off") == 0) {
    leftStepper.disable();
  } else if (std::strcmp(command, "left test") == 0) {
    if (!leftStepper.startTest(Config::TEST_STEP_COUNT,
                               Config::TEST_STEP_INTERVAL_US)) {
      Serial.println("Left motor is already moving.");
      return;
    }
    Serial.println("Left 10-pulse test started.");
    return;
  } else if (std::strcmp(command, "left forward") == 0) {
    if (!leftStepper.startMove(Config::MOTION_TEST_STEP_COUNT, true,
                               Config::MOTION_TEST_STEP_INTERVAL_US)) {
      Serial.println("Left motor is already moving.");
      return;
    }
    Serial.println("Left forward test started.");
    return;
  } else if (std::strcmp(command, "left reverse") == 0) {
    if (!leftStepper.startMove(Config::MOTION_TEST_STEP_COUNT, false,
                               Config::MOTION_TEST_STEP_INTERVAL_US)) {
      Serial.println("Left motor is already moving.");
      return;
    }
    Serial.println("Left reverse test started.");
    return;
  } else if (std::strcmp(command, "right on") == 0) {
    rightStepper.enable();
  } else if (std::strcmp(command, "right off") == 0) {
    rightStepper.disable();
  } else if (std::strcmp(command, "right test") == 0) {
    if (!rightStepper.startTest(Config::TEST_STEP_COUNT,
                                Config::TEST_STEP_INTERVAL_US)) {
      Serial.println("Right motor is already moving.");
      return;
    }
    Serial.println("Right 10-pulse test started.");
    return;
  } else if (std::strcmp(command, "right forward") == 0) {
    if (!rightStepper.startMove(Config::MOTION_TEST_STEP_COUNT, true,
                                Config::MOTION_TEST_STEP_INTERVAL_US)) {
      Serial.println("Right motor is already moving.");
      return;
    }
    Serial.println("Right forward test started.");
    return;
  } else if (std::strcmp(command, "right reverse") == 0) {
    if (!rightStepper.startMove(Config::MOTION_TEST_STEP_COUNT, false,
                                Config::MOTION_TEST_STEP_INTERVAL_US)) {
      Serial.println("Right motor is already moving.");
      return;
    }
    Serial.println("Right reverse test started.");
    return;
  } else if (std::strcmp(command, "off") == 0) {
    leftStepper.disable();
    rightStepper.disable();
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
  leftStepper.begin();
  rightStepper.begin();

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
  leftStepper.update(micros());
  rightStepper.update(micros());
}
