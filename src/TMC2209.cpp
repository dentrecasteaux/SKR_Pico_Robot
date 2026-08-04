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
  status.stallGuardResult = raw & 0x03FFU;
  status.shortToSupplyA = raw & (1UL << 12);
  status.shortToSupplyB = raw & (1UL << 13);
  status.stealthChop = raw & (1UL << 14);
  status.fullStepActive = raw & (1UL << 15);
  status.currentScale = (raw >> 16) & 0x1FU;
  status.stallGuard = raw & (1UL << 24);
  status.overTemperature = raw & (1UL << 25);
  status.overTemperaturePreWarning = raw & (1UL << 26);
  status.shortToGroundA = raw & (1UL << 27);
  status.shortToGroundB = raw & (1UL << 28);
  status.openLoadA = raw & (1UL << 29);
  status.openLoadB = raw & (1UL << 30);
  status.standstill = raw & (1UL << 31);
  return status;
}

bool TMC2209::Status::hasFault() const
{
  return !connected || overTemperature || overTemperaturePreWarning ||
         shortToGroundA || shortToGroundB || shortToSupplyA || shortToSupplyB;
}
