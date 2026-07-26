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
