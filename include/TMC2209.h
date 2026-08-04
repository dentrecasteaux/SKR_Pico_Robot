#pragma once

#include <Arduino.h>
#include <TMCStepper.h>

class TMC2209
{
public:
  struct Status
  {
    bool connected = false;
    uint32_t raw = 0;
    uint16_t stallGuardResult = 0;
    uint8_t currentScale = 0;
    bool shortToSupplyA = false;
    bool shortToSupplyB = false;
    bool stealthChop = false;
    bool fullStepActive = false;
    bool stallGuard = false;
    bool overTemperature = false;
    bool overTemperaturePreWarning = false;
    bool shortToGroundA = false;
    bool shortToGroundB = false;
    bool openLoadA = false;
    bool openLoadB = false;
    bool standstill = false;

    bool hasFault() const;
  };

  TMC2209(Stream& uart, uint8_t address);

  void begin();
  bool isConnected();
  Status status();

  static Status decodeStatus(uint32_t raw, bool connected = true);

private:
  static constexpr float SENSE_RESISTOR_OHMS = 0.11F;

  TMC2209Stepper driver_;
};
