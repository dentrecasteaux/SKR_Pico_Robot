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

  struct PwmInfo
  {
    bool connected = false;
    uint32_t pwmConf = 0;      // PWMCONF as last written (driver shadow)
    uint8_t ofs = 0;           // configured PWM_OFS
    uint8_t grad = 0;          // configured PWM_GRAD
    bool autoscale = false;
    bool autograd = false;
    uint8_t ofsAuto = 0;       // PWM_AUTO: learned amplitude offset
    uint8_t gradAuto = 0;      // PWM_AUTO: learned gradient
    uint8_t scaleSum = 0;      // PWM_SCALE: actual PWM amplitude
    int16_t scaleAuto = 0;     // PWM_SCALE: automatic amplitude correction
  };

  TMC2209(Stream& uart, uint8_t address);

  void begin();
  bool isConnected();
  Status status();
  bool applySettings(const Settings& settings);
  Settings settings();
  PwmInfo pwmInfo();
  // Negative arguments leave that field unchanged.
  bool setPwm(int ofs, int grad, int autograd, int autoscale = -1);

  static Status decodeStatus(uint32_t raw, bool connected = true);
  static bool validMicrosteps(uint16_t microsteps);

private:
  static constexpr float SENSE_RESISTOR_OHMS = 0.11F;

  TMC2209Stepper driver_;
};
