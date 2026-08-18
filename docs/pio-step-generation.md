# RP2040 PIO STEP Generation

## Purpose

The SKR Pico uses the RP2040's Programmable I/O hardware to generate STEP
pulses for both TMC2209 motor drivers. A PIO state machine produces each pulse
at hardware timing, so Linux activity, USB traffic, serial parsing and ordinary
firmware-loop jitter do not stretch the pulse width or the programmed interval.

The division of responsibility is:

```text
Robot / MotionController     physical motion and safety state
            |
            v
Motor                       acceleration and target STEP rate
            |
            v
Stepper                     DIR, ENABLE and PIO commands
            |
            v
RP2040 PIO state machine    timed STEP waveform and pulse counting
```

PIO does not know about millimetres, wheel geometry, turning, leases or motor
current. It receives only a pulse count and an interval in microseconds.

## PIO resources

The implementation is built at runtime in `src/Stepper.cpp` rather than from a
separate `.pio` source file.

- Both motors use `pio0`.
- The 16-instruction program is loaded into PIO instruction memory once.
- Each `Stepper` claims one unused state machine.
- Both state machines run the same program but map their SET output to different
  STEP pins.
- The state-machine clock is divided from the 133 MHz system clock to 1 MHz.
  One PIO instruction cycle therefore equals one microsecond.
- DIR and active-low ENABLE remain ordinary CPU-controlled GPIO outputs.

Loading one shared program saves instruction memory while independent state
machines allow the wheels to run at different rates.

## Command words sent through the FIFO

`Stepper::startPulses()` resets its state machine and writes two blocking words
to the transmit FIFO:

1. pulse count minus one;
2. requested interval minus 14 PIO cycles.

The count uses the usual PIO decrement-loop representation: zero means one
pulse, one means two pulses, and so on. The interval subtracts the 14 cycles
spent outside the delay loop so the complete waveform period matches the
requested interval.

Continuous movement uses `UINT32_MAX` as the requested count. In practice this
is treated as an indefinitely long train and is stopped explicitly by the
velocity lease, `STOP`, `ESTOP`, a replacement command or a fault.

## Instruction sequence

The program occupies offsets 0–15. Its logical sequence is:

| Offset | Operation | Purpose |
|---:|---|---|
| 0 | blocking `pull` | Receive pulse count minus one |
| 1 | `mov isr, osr` | Preserve the remaining-pulse counter |
| 2 | blocking `pull` | Receive initial delay-loop value |
| 3 | `mov x, osr` | Preserve the current interval |
| 4 | non-blocking `pull` | Take a new interval if the CPU supplied one; otherwise reuse X |
| 5 | `mov x, osr` | Make that interval the new fallback value |
| 6 | `mov y, osr` | Load the delay counter |
| 7 | `set pins, 1 [4]` | Raise STEP for five PIO cycles, or 5 µs |
| 8 | `set pins, 0` | Lower STEP |
| 9 | `jmp y--, 9` | Hold the remainder of the programmed period |
| 10 | `mov y, isr` | Load the remaining-pulse counter |
| 11 | `jmp y--, 14` | Continue if more pulses remain |
| 12 | relative `irq set 0` | Signal finite-train completion for this state machine |
| 13 | `jmp 0` | Wait for the next two-word command |
| 14 | `mov isr, y` | Save the decremented pulse count |
| 15 | `jmp 4` | Generate the next pulse |

The relative IRQ selects the IRQ corresponding to the executing state-machine
number. `Stepper::pulsesComplete()` checks and clears that IRQ, stops the state
machine and lets `Motor` disable the driver after a finite move.

## Changing speed during continuous motion

`Motor` maintains a floating-point current speed and an integer target speed in
steps per second. Its `update()` method applies the configured acceleration on
each firmware-loop pass. When the whole-number STEP frequency changes,
`Stepper::updatePulseInterval()` places a replacement interval in the FIFO.

The PIO program checks for one replacement interval at the start of every
pulse. If none is waiting, the non-blocking pull reuses the previous value. The
CPU therefore updates the plan while PIO preserves the waveform timing.

Only one pending interval is admitted at a time. This avoids filling the FIFO
with obsolete ramp values. An interval already in progress completes before a
new value takes effect.

## Fractional-step startup

Starting PIO as soon as a floating-point ramp rounded to 1 step/s caused a
visible twitch followed by nearly a one-second pause. That was not a browser or
lease delay: the first pulse happened immediately and PIO then correctly
completed the initially programmed 1 Hz period.

The current `Motor` implementation integrates fractional step travel during
acceleration from rest using the trapezoidal area between the previous and new
speed:

```text
step progress += (|previous speed| + |new speed|) / 2 × elapsed time
```

PIO starts only when this accumulated progress reaches one complete step. With
1/4 microstepping and the present 61 mm wheel diameter:

- 10 mm/s² acceleration produces the first step after about 218 ms at 9 Hz;
- 150 mm/s² produces it after about 57 ms at 35 Hz;
- 300 mm/s² produces it after about 40 ms at 50 Hz.

This preserves a physical acceleration ramp without an artificial initial
twitch. Stopping or changing direction before startup clears the accumulated
fraction so stale travel cannot create a later pulse in the wrong direction.

## Finite moves and turns

Finite `MOVE` and `TURN` commands are converted to an exact step count by
`MotionController`. Each state machine receives that count and the fixed
calibration interval of 10,000 µs, or 100 Hz. The PIO completion IRQ tells the
firmware when each train has ended. When both motors finish, `Robot` completes
the job and emits its `DONE` event.

Finite actions currently use a fixed STEP rate and do not use the continuous
velocity acceleration ramp. Their position is open loop: completion means that
the requested pulses were generated, not that encoders confirmed movement.

## Timing limits

The program requires 14 cycles outside its programmable delay. The code rejects
intervals shorter than 15 µs. The firmware applies a more conservative maximum
of 50,000 STEP pulses/s, corresponding to a 20 µs period.

At the boot setting of four microsteps, the present geometry uses approximately
4.17 steps/mm. A 1,000 mm/s linear request is therefore about 4.17 kHz before
differential mixing. Combined linear and angular requests are proportionally
limited if either wheel would exceed the configured PIO ceiling.

These electrical timing limits do not guarantee usable mechanical speed.
Available motor torque, supply voltage, current, payload and surface conditions
normally impose a lower practical limit.

## Stop and reset behaviour

Stopping is deterministic and does not depend on draining the FIFO:

1. disable the state machine;
2. clear its transmit FIFO;
3. drive STEP low;
4. clear motor target/current speed and fractional startup progress;
5. drive the TMC2209 ENABLE input high to disable the motor output.

Starting a new train also clears FIFOs, restarts the state machine and jumps to
the program entry point before sending new command words. Old counts and timing
values cannot leak into a later movement.

## Diagnostics and troubleshooting

Status exposes the firmware-commanded STEP frequency independently for the left
and right wheels. During motion, TMC2209 reads are rate-limited and alternate
between drivers so diagnostic UART traffic does not dominate the main loop.

After changing the PIO engine, validate with the wheels raised:

1. finite ten-step tests on each motor;
2. forward and reverse direction on each motor;
3. a bounded straight move and turn;
4. continuous movement at 10 mm/s² to check fractional-step startup;
5. continuous movement at the 150 mm/s² boot default;
6. immediate `STOP` and lease-expiry behavior;
7. different left/right rates from combined linear and angular commands.

Useful symptoms:

- immediate twitch then long pause: startup began at an excessively low
  whole-number frequency;
- pulse train never completes: inspect the count, relative IRQ and IRQ clear;
- speed updates lag: inspect FIFO occupancy and whether an earlier interval is
  still in progress;
- wrong direction with correct pulse rate: inspect CPU-controlled DIR mapping,
  not the PIO SET pin;
- commanded frequency is correct but wheel speed is not: investigate motor
  torque, current, supply, load and missed steps.

## Possible future extensions

DMA could feed longer precomputed acceleration profiles into the state-machine
FIFO and reduce CPU involvement further. That is not currently necessary for
the tested robot. Any DMA design must preserve immediate stop behavior, avoid
stale buffered motion after a stop, and keep the existing `Robot` safety and
lease boundaries authoritative.
