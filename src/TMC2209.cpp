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
  if (Config::TMC_PWM_FIXED) {
    driver_.pwm_ofs(Config::TMC_PWM_OFS);
    driver_.pwm_grad(Config::TMC_PWM_GRAD);
    driver_.pwm_autograd(false);
  }
}

TMC2209::PwmInfo TMC2209::pwmInfo()
{
  PwmInfo info;
  if (!isConnected()) return info;
  info.connected = true;
  info.pwmConf = driver_.PWMCONF();
  info.ofs = driver_.pwm_ofs();
  info.grad = driver_.pwm_grad();
  info.autoscale = driver_.pwm_autoscale();
  info.autograd = driver_.pwm_autograd();
  const uint32_t pwmAuto = driver_.PWM_AUTO();
  info.ofsAuto = static_cast<uint8_t>(pwmAuto & 0xFF);
  info.gradAuto = static_cast<uint8_t>((pwmAuto >> 16) & 0xFF);
  const uint32_t pwmScale = driver_.PWM_SCALE();
  info.scaleSum = static_cast<uint8_t>(pwmScale & 0xFF);
  // PWM_SCALE_AUTO is a 9-bit signed value in bits 16..24.
  int16_t scaleAuto = static_cast<int16_t>((pwmScale >> 16) & 0x1FF);
  if (scaleAuto & 0x100) scaleAuto -= 0x200;
  info.scaleAuto = scaleAuto;
  return info;
}

bool TMC2209::setPwm(int ofs, int grad, int autograd, int autoscale)
{
  if (!isConnected() || ofs > 255 || grad > 255 || autograd > 1 ||
      autoscale > 1) {
    return false;
  }
  // AUTOSCALE=0 is StealthChop's "forward controlled" mode: the PWM
  // amplitude is simply OFS + GRAD x speed, with no current regulation.
  if (autoscale >= 0) driver_.pwm_autoscale(autoscale == 1);
  if (ofs >= 0) driver_.pwm_ofs(static_cast<uint8_t>(ofs));
  if (grad >= 0) driver_.pwm_grad(static_cast<uint8_t>(grad));
  if (autograd >= 0) driver_.pwm_autograd(autograd == 1);
  return true;
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
  // The driver sets current in 1/32 steps of its full scale (about 50-60 mA
  // here), so the read-back value can differ from the request by up to half
  // a step. A fixed 25 mA tolerance rejected 1300 mA (23 Nov).
  const uint16_t tolerance =
      requested.currentMa / 16 > 25 ? requested.currentMa / 16 : 25;
  return currentDifference <= tolerance &&
         actual.microsteps == requested.microsteps &&
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
