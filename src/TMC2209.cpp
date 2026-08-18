#include "TMC2209.h"

#include "Config.h"

TMC2209::TMC2209(Stream& uart, uint8_t address)
    : driver_(&uart, SENSE_RESISTOR_OHMS, address)
{
}

void TMC2209::begin()
{
  driver_.begin();
  driver_.I_scale_analog(false);
  driver_.toff(5);
  driver_.rms_current(Config::TMC_RUN_CURRENT_MA);
  driver_.microsteps(Config::TMC_MICROSTEPS);
  driver_.en_spreadCycle(false);
  driver_.pwm_autoscale(true);
}

bool TMC2209::isConnected()
{
  return driver_.test_connection() == 0;
}

bool TMC2209::applySettings(const Settings& requested)
{
  if (!isConnected() || requested.currentMa == 0 ||
      !validMicrosteps(requested.microsteps)) {
    return false;
  }

  driver_.rms_current(requested.currentMa);
  driver_.microsteps(requested.microsteps == 1 ? 0 : requested.microsteps);
  driver_.en_spreadCycle(requested.mode == ChopperMode::SpreadCycle);

  const Settings actual = settings();
  const uint16_t currentDifference =
      actual.currentMa > requested.currentMa
          ? actual.currentMa - requested.currentMa
          : requested.currentMa - actual.currentMa;
  return currentDifference <= 25 && actual.microsteps == requested.microsteps &&
         actual.mode == requested.mode;
}

TMC2209::Settings TMC2209::settings()
{
  Settings result;
  result.currentMa = driver_.rms_current();
  result.microsteps = driver_.microsteps();
  if (result.microsteps == 0) result.microsteps = 1;
  result.mode = driver_.en_spreadCycle() ? ChopperMode::SpreadCycle
                                         : ChopperMode::StealthChop;
  return result;
}

bool TMC2209::validMicrosteps(uint16_t microsteps)
{
  return microsteps == 1 || microsteps == 2 || microsteps == 4 ||
         microsteps == 8 || microsteps == 16 || microsteps == 32 ||
         microsteps == 64;
}

TMC2209::Status TMC2209::status()
{
  if (!isConnected()) return Status{};
  return decodeStatus(driver_.DRV_STATUS());
}

TMC2209::Status TMC2209::decodeStatus(uint32_t raw, bool connected)
{
  Status status;
  status.connected = connected;
  status.raw = raw;
  status.overTemperaturePreWarning = raw & (1UL << 0);
  status.overTemperature = raw & (1UL << 1);
  status.shortToGroundA = raw & (1UL << 2);
  status.shortToGroundB = raw & (1UL << 3);
  status.shortToSupplyA = raw & (1UL << 4);
  status.shortToSupplyB = raw & (1UL << 5);
  status.openLoadA = raw & (1UL << 6);
  status.openLoadB = raw & (1UL << 7);
  status.currentScale = (raw >> 16) & 0x1FU;
  status.stealthChop = raw & (1UL << 30);
  status.standstill = raw & (1UL << 31);
  return status;
}

bool TMC2209::Status::hasFault() const
{
  return !connected || overTemperature || overTemperaturePreWarning ||
         shortToGroundA || shortToGroundB || shortToSupplyA || shortToSupplyB;
}
