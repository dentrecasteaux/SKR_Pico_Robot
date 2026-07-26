#include <Arduino.h>
#include <cstring>

#include "Hardware.h"
#include "Config.h"
#include "Stepper.h"

Stepper leftStepper(HW::LEFT_EN);
Stepper rightStepper(HW::RIGHT_EN);

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
  Serial.println("Commands: left on, left off, right on, right off, off, status, help");
}

void processCommand(const char* command)
{
  if (std::strcmp(command, "left on") == 0) {
    leftStepper.enable();
  } else if (std::strcmp(command, "left off") == 0) {
    leftStepper.disable();
  } else if (std::strcmp(command, "right on") == 0) {
    rightStepper.enable();
  } else if (std::strcmp(command, "right off") == 0) {
    rightStepper.disable();
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

  Serial.begin(Config::SERIAL_BAUD);
  while (!Serial) {
    delay(10);
  }

  Serial.println("SKR Pico starting...");
  Serial.println("Stepper drivers initialised and disabled.");
  printHelp();
}

void loop()
{
  processSerialCommands();
}
