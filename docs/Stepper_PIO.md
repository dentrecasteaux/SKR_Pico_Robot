# How the RP2040 PIO Drives the Stepper Motors

### Why use PIO?
* The RP2040 has two **Programmable I/O (PIO)** blocks, each with four independent state‑machines (SM).  
* A state‑machine runs a tiny assembly‑like program (max 32 instructions) at a deterministic clock rate.  
* Because the program runs in hardware, step pulses can be produced with micro‑second precision **without any CPU‑interrupt jitter**.  
* The Pico can therefore generate step rates up to the `MAX_PIO_STEP_FREQUENCY_HZ` (≈ 50 kHz) while the main loop stays free for communication, safety checks, UI, etc.

---

## 1. The PIO program (see `src/Stepper.cpp`)
```text
// 16‑word program, loaded once into pio0
0  pull   !block, !ifempty   // pull step‑count (steps‑1) from FIFO → OSR
1  mov    ISR, OSR           // copy to ISR (used later for IRQ)
2  pull   !block, !ifempty   // pull delay (µs) from FIFO → OSR
3  mov    X, OSR             // X = delay‑counter (will be used in the inner loop)
4  pull   !block, !ifempty   // pull a dummy word (required by the PIO “pull” pattern)
5  mov    X, OSR             // X = step‑count‑1 (how many pulses to emit)
6  mov    Y, OSR             // Y = delay‑counter (again, for the outer loop)
7  set    pins, 1  [4]       // STEP high for 5 µs (4 extra cycles + the instruction itself)
8  set    pins, 0            // STEP low
9  jmp    Y--, DELAY_LOOP    // inner loop: count down Y (delay between pulses)
10 mov    Y, ISR             // reload Y from ISR (the original delay value)
11 jmp    Y--, CONTINUE_LOOP // outer loop: count down X (step count)
12 irq    set, 0             // raise IRQ0 → tells the CPU the move is finished
13 jmp    PROGRAM_WRAP_TARGET// jump back to start of program (wrap)
14 mov    ISR, Y             // restore ISR (required for next IRQ)
15 jmp    PULSE_LOOP         // go back to the first pulse
```

**Key points**
| Instruction | What it does for the stepper |
|-------------|------------------------------|
| **pull / mov** | Reads two 32‑bit words from the SM’s TX FIFO: <br>1️⃣ *steps‑1* (how many pulses to generate) <br>2️⃣ *step‑interval‑µs* (delay between pulses). |
| **set pins, 1 [4]** | Drives the STEP line high for **5 µs** (the “pulse width”). |
| **jmp Y‑‑** loops | The **inner loop** (`Y`) creates the required low‑time between pulses. The outer loop (`X`) repeats the whole pulse sequence for the requested number of steps. |
| **irq set** | When the outer loop finishes, the SM raises **IRQ0**. The driver code polls this IRQ to know the move is complete. |
| **wrap / jmp** | The program wraps back to the start, allowing the SM to be re‑started instantly for the next command. |

The program is **static** – it never changes at run‑time – and is loaded only once (`loadProgram()`).

---

## 2. Initialisation (`Stepper::begin()`)
```cpp
loadProgram();                                 // load the 16‑word program into pio0
stateMachine_ = pio_claim_unused_sm(pio0, true); // claim a free SM (0‑3)

pio_gpio_init(pio0, stepPin_);                 // make the STEP pin a PIO‑controlled pin
pio_sm_config config = pio_get_default_sm_config();

sm_config_set_wrap(&config,
    programOffset + PROGRAM_WRAP_TARGET,
    programOffset + PROGRAM_WRAP);             // set wrap points (15 → 0)

sm_config_set_set_pins(&config, stepPin_, 1); // the “set” instruction will affect STEP
sm_config_set_clkdiv(&config,
    static_cast<float>(clock_get_hz(clk_sys)) / 1'000'000.0F);
// → one PIO clock tick = 1 µs (the SM runs at 1 MHz)

pio_sm_init(pio0, stateMachine_, programOffset, &config);
pio_sm_set_consecutive_pindirs(pio0, stateMachine_, stepPin_, 1, true);
pio_sm_set_pins_with_mask(pio0, stateMachine_, 0, 1UL << stepPin_);
```
* **Clock divisor**: `clk_sys / 1 MHz` makes each PIO cycle exactly **1 µs**. All timing in the program is expressed in microseconds, which matches the units used by the rest of the firmware.
* **Pin configuration**: The STEP pin is driven directly by the PIO `SET` instruction, while DIR and EN remain ordinary GPIOs controlled by the main CPU.

---

## 3. Starting a move (`Stepper::startPulses()`)
```cpp
bool Stepper::startPulses(uint32_t steps, uint32_t stepIntervalUs)
{
    // sanity checks …
    stopPulses();                     // make sure SM is idle
    pio_interrupt_clear(pio0, stateMachine_);
    pio_sm_clear_fifos(pio0, stateMachine_);
    pio_sm_restart(pio0, stateMachine_);
    pio_sm_exec(pio0, stateMachine_, pio_encode_jmp(programOffset));

    // FIFO order matters: first the step count, then the interval
    pio_sm_put_blocking(pio0, stateMachine_, steps - 1);
    pio_sm_put_blocking(pio0, stateMachine_,
                        stepIntervalUs - PIO_CYCLES_OUTSIDE_DELAY);
    // The “outside delay” accounts for the 5 µs high pulse + a few
    // instruction cycles that the SM itself consumes.
    pio_sm_set_enabled(pio0, stateMachine_, true);
    return true;
}
```
* **FIFO feeding** – The SM reads the two words in the exact order the program expects.
* **`stepIntervalUs - PIO_CYCLES_OUTSIDE_DELAY`** – The interval supplied by the caller is the *total* period between successive STEP rising edges. The program already spends **14 µs** (the constant `PIO_CYCLES_OUTSIDE_DELAY`) on the high pulse and on the instruction overhead, so we subtract that amount before writing the value to the FIFO.
* **Enabling the SM** – Once enabled, the SM runs completely autonomously, generating the required number of pulses at the requested spacing.

---

## 4. Updating the interval on‑the‑fly (`Stepper::updatePulseInterval()`)
```cpp
bool Stepper::updatePulseInterval(uint32_t stepIntervalUs)
{
    if (stateMachine_ < 0 || stepIntervalUs < MIN_STEP_INTERVAL_US ||
        !pio_sm_is_tx_fifo_empty(pio0, stateMachine_)) {
        return false;               // cannot change while a word is still pending
    }
    pio_sm_put(pio0, stateMachine_,
                stepIntervalUs - PIO_CYCLES_OUTSIDE_DELAY);
    return true;
}
```
* The function is used by the **velocity‑lease** logic: while a continuous‑velocity command is active the host can tweak the speed without stopping the motor.
* It only works when the TX FIFO is empty (i.e., the SM has already consumed the previous interval word).

---

## 5. Detecting completion (`Stepper::pulsesComplete()`)
```cpp
bool Stepper::pulsesComplete()
{
    if (stateMachine_ < 0 || !pio_interrupt_get(pio0, stateMachine_))
        return false;               // no IRQ yet → still moving

    pio_interrupt_clear(pio0, stateMachine_);
    stopPulses();                   // disables SM, clears FIFOs, forces STEP low
    return true;
}
```
* The **IRQ0** set by instruction `12` in the PIO program is the only signal the SM sends back to the CPU.
* When the driver sees the IRQ, it knows the finite move (or test pulse train) has finished, clears the flag, and disables the SM.

---

## 6. Interaction with the rest of the firmware
| Component | How it uses the Stepper API |
|-----------|-----------------------------|
| **Motor** | Calls `startPulses()`, `updatePulseInterval()`, `pulsesComplete()`, and `enable()/disable()` to implement acceleration profiles, test bursts, and normal motion. |
| **MotionController** | Computes the desired step‑frequency for each wheel (linear + turn mixing) and feeds those values to the two `Stepper` instances. |
| **RobotLink** | Receives high‑level commands from the Pi (e.g., `SET_VELOCITY`, `MOVE`, `TURN`) and forwards them to `Robot`, which in turn uses the `Motor`/`Stepper` stack. |
| **Safety / Lease logic** | Periodically checks `pulsesComplete()` to know when a finite move has ended, and uses `updatePulseInterval()` to enforce the lease timeout for continuous velocity commands. |

Because the PIO handles the exact timing of each STEP edge, the CPU never needs to busy‑wait or use `delayMicroseconds()`. This gives:
* **Deterministic pulse timing** – essential for reliable stepper operation at high speeds.
* **Low CPU load** – the main loop can stay responsive to serial traffic, safety checks, and the web UI.
* **Scalability** – each motor gets its own SM, so both wheels can run at independent speeds without interfering with each other.

---

## 7. Summary (plain language)
1. **Program load** – A 16‑instruction PIO program is compiled into a binary array and uploaded to the RP2040’s PIO block once.
2. **State‑machine claim** – The firmware claims one of the four SMs on `pio0` and configures it to run at a 1 µs clock tick.
3. **GPIO wiring** – The STEP pin is handed over to the PIO; DIR and EN stay under normal GPIO control.
4. **Start a move** – The CPU writes two numbers into the SM’s FIFO: (steps‑1) and (desired period‑µs). The SM then autonomously toggles the STEP line high for 5 µs, waits the requested low‑time, repeats until the step count reaches zero, and finally raises an IRQ.
5. **Completion** – The CPU polls the IRQ; when it sees it, it disables the SM and clears the FIFO.
6. **Dynamic speed changes** – While a continuous‑velocity command is active, the CPU can push a new period value into the FIFO (provided the FIFO is empty), letting the motor accelerate or decelerate without stopping.

All of this is encapsulated in the `Stepper` class, which presents a clean C++ interface (`startPulses`, `updatePulseInterval`, `pulsesComplete`, `enable`, `disable`, `setDirection`) to the higher‑level motion controller and robot logic. The result is a **tight, low‑jitter step‑pulse generator** that coexists peacefully with the rest of the robot’s safety‑critical firmware.
