#include "Stepper.h"

#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <hardware/pio_instructions.h>

namespace
{
// The state machine receives two words: step count minus one, then the delay
// between pulses. It raises STEP for 5 us, counts pulses in its ISR, and sets
// the IRQ corresponding to its state-machine number when a finite move ends.
constexpr uint PROGRAM_LENGTH = 16;
constexpr uint PROGRAM_WRAP_TARGET = 0;
constexpr uint PROGRAM_WRAP = 15;
constexpr uint PULSE_LOOP = 4;
constexpr uint DELAY_LOOP = 9;
constexpr uint CONTINUE_LOOP = 14;
constexpr uint32_t PIO_CYCLES_OUTSIDE_DELAY = 14;
constexpr uint32_t MIN_STEP_INTERVAL_US = PIO_CYCLES_OUTSIDE_DELAY + 1;

uint16_t programInstructions[PROGRAM_LENGTH];
pio_program program = {programInstructions, PROGRAM_LENGTH, -1};
bool programLoaded = false;
uint programOffset = 0;

void loadProgram()
{
  if (programLoaded) return;

  programInstructions[0] = pio_encode_pull(false, true);
  programInstructions[1] = pio_encode_mov(pio_isr, pio_osr);
  programInstructions[2] = pio_encode_pull(false, true);
  programInstructions[3] = pio_encode_mov(pio_x, pio_osr);
  programInstructions[4] = pio_encode_pull(false, false);
  programInstructions[5] = pio_encode_mov(pio_x, pio_osr);
  programInstructions[6] = pio_encode_mov(pio_y, pio_osr);
  programInstructions[7] = pio_encode_set(pio_pins, 1) | pio_encode_delay(4);
  programInstructions[8] = pio_encode_set(pio_pins, 0);
  programInstructions[9] = pio_encode_jmp_y_dec(DELAY_LOOP);
  programInstructions[10] = pio_encode_mov(pio_y, pio_isr);
  programInstructions[11] = pio_encode_jmp_y_dec(CONTINUE_LOOP);
  programInstructions[12] = pio_encode_irq_set(true, 0);
  programInstructions[13] = pio_encode_jmp(PROGRAM_WRAP_TARGET);
  programInstructions[14] = pio_encode_mov(pio_isr, pio_y);
  programInstructions[15] = pio_encode_jmp(PULSE_LOOP);

  programOffset = pio_add_program(pio0, &program);
  programLoaded = true;
}
}  // namespace

Stepper::Stepper(uint8_t stepPin, uint8_t directionPin, uint8_t enablePin,
                 bool directionInverted)
    : stepPin_(stepPin),
      directionPin_(directionPin),
      enablePin_(enablePin),
      directionInverted_(directionInverted)
{
}

void Stepper::begin()
{
  pinMode(directionPin_, OUTPUT);
  pinMode(enablePin_, OUTPUT);

  loadProgram();
  stateMachine_ = pio_claim_unused_sm(pio0, true);
  pio_gpio_init(pio0, stepPin_);
  pio_sm_config config = pio_get_default_sm_config();
  sm_config_set_wrap(&config, programOffset + PROGRAM_WRAP_TARGET,
                     programOffset + PROGRAM_WRAP);
  sm_config_set_set_pins(&config, stepPin_, 1);
  sm_config_set_clkdiv(&config,
                       static_cast<float>(clock_get_hz(clk_sys)) / 1000000.0F);
  pio_sm_init(pio0, stateMachine_, programOffset, &config);
  pio_sm_set_consecutive_pindirs(pio0, stateMachine_, stepPin_, 1, true);
  pio_sm_set_pins_with_mask(pio0, stateMachine_, 0, 1UL << stepPin_);

  setDirection(false);
  disable();
}

void Stepper::enable()
{
  // The TMC2209 enable input on the SKR Pico is active low.
  digitalWrite(enablePin_, LOW);
  enabled_ = true;
}

void Stepper::disable()
{
  stopPulses();
  digitalWrite(enablePin_, HIGH);
  enabled_ = false;
}

void Stepper::setDirection(bool forward)
{
  const bool level = forward ^ directionInverted_;
  digitalWrite(directionPin_, level ? HIGH : LOW);
}

bool Stepper::startPulses(uint32_t steps, uint32_t stepIntervalUs)
{
  if (stateMachine_ < 0 || steps == 0 ||
      stepIntervalUs < MIN_STEP_INTERVAL_US) {
    return false;
  }

  stopPulses();
  pio_interrupt_clear(pio0, stateMachine_);
  pio_sm_clear_fifos(pio0, stateMachine_);
  pio_sm_restart(pio0, stateMachine_);
  pio_sm_exec(pio0, stateMachine_, pio_encode_jmp(programOffset));
  pio_sm_put_blocking(pio0, stateMachine_, steps - 1);
  pio_sm_put_blocking(pio0, stateMachine_,
                      stepIntervalUs - PIO_CYCLES_OUTSIDE_DELAY);
  pio_sm_set_enabled(pio0, stateMachine_, true);
  return true;
}

bool Stepper::updatePulseInterval(uint32_t stepIntervalUs)
{
  if (stateMachine_ < 0 || stepIntervalUs < MIN_STEP_INTERVAL_US ||
      !pio_sm_is_tx_fifo_empty(pio0, stateMachine_)) {
    return false;
  }
  pio_sm_put(pio0, stateMachine_,
             stepIntervalUs - PIO_CYCLES_OUTSIDE_DELAY);
  return true;
}

void Stepper::stopPulses()
{
  if (stateMachine_ < 0) return;
  pio_sm_set_enabled(pio0, stateMachine_, false);
  pio_sm_clear_fifos(pio0, stateMachine_);
  pio_sm_set_pins_with_mask(pio0, stateMachine_, 0, 1UL << stepPin_);
}

bool Stepper::pulsesComplete()
{
  if (stateMachine_ < 0 || !pio_interrupt_get(pio0, stateMachine_)) {
    return false;
  }
  pio_interrupt_clear(pio0, stateMachine_);
  stopPulses();
  return true;
}

bool Stepper::isEnabled() const
{
  return enabled_;
}
