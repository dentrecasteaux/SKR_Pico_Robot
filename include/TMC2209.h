#pragma once

#include <Arduino.h>
#include <TMCStepper.h>

class TMC2209
{
public:
  TMC2209(Stream& uart, uint8_t address);

  void begin();
  bool isConnected();
  uint32_t status();

private:
  static constexpr float SENSE_RESISTOR_OHMS = 0.11F;

  TMC2209Stepper driver_;
};
