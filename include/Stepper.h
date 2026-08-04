#pragma once

#include <Arduino.h>

// A low-level stepper interface. It owns only the physical STEP, DIR, and
// enable pins. Pulse timing belongs to Motor.
class Stepper
{
public:
  Stepper(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin,
          bool directionInverted);

  void begin();
  void enable();
  void disable();

  void setDirection(bool forward);
  bool startPulses(uint32_t steps, uint32_t stepIntervalUs);
  bool updatePulseInterval(uint32_t stepIntervalUs);
  void stopPulses();
  bool pulsesComplete();

  bool isEnabled() const;

private:
  uint8_t stepPin_;
  uint8_t directionPin_;
  uint8_t enablePin_;
  bool directionInverted_;
  bool enabled_ = false;
  int stateMachine_ = -1;
};
