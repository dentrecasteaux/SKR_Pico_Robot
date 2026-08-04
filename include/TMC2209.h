#pragma once

#include <Arduino.h>
#include <TMCStepper.h>

class TMC2209
{
public:
  enum class ChopperMode { StealthChop, SpreadCycle };

  struct Settings
  {
    uint16_t currentMa = 0;
    uint16_t microsteps = 0;
    ChopperMode mode = ChopperMode::StealthChop;
  };

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
  bool applySettings(const Settings& settings);
  Settings settings();

  static Status decodeStatus(uint32_t raw, bool connected = true);
  static bool validMicrosteps(uint16_t microsteps);

private:
  static constexpr float SENSE_RESISTOR_OHMS = 0.11F;

  TMC2209Stepper driver_;
};
