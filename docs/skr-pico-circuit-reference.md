# BTT SKR Pico V1.0: circuit reference and SPI IMU (SEN0443) hookup

Traced from the official BIGTREETECH schematic (`Hardware/BTT SKR Pico V1.0-SCH.pdf`), the pin diagram (`-PIN.pdf`) and the Klipper reference config in github.com/bigtreetech/skr-pico, 16 Nov 2026.

## 1. Short version

- All 30 RP2040 GPIOs are used by the board. Only **five come out to a header as plain, two-way GPIO**: **GPIO0, GPIO1, GPIO22, GPIO29**, and GPIO20 (through 100 Ω, with side effects).
- Four more reach a header **as inputs only**, through a 74LVC2G34 buffer with an RC filter: GPIO3, 4, 16, 25 (the endstops). These are fine for the IMU's INT line. They are not usable as SPI MOSI or SCK, and MISO only works if you remove the filter capacitor.
- **No complete hardware SPI port is reachable without changing the board.** SPI0's SCK (GPIO22), MISO (GPIO0) and CSn (GPIO1) are all on headers, but every SPI0 MOSI pin (GPIO3/7/19/23) is either buffered as an input or wired only to on-board parts.
- **Recommended, no board changes:** PIO-based SPI (arduino-pico `SoftwareSPI`) with SCK on GPIO22, MOSI on GPIO29, MISO on GPIO0, CS on GPIO1 and INT on GPIO16. These are the same pins as hardware SPI0 apart from MOSI.
- **Upgrade path to true hardware SPI0:** solder one wire to GPIO23 (pin 12 of U7, or the pad of its 10 kΩ resistor R69). Move MOSI from GPIO29 to that wire. Nothing else changes.

## 2. Board architecture

| Block | Parts | Notes |
|---|---|---|
| MCU | RP2040 (U2), 12 MHz crystal X1, W25Q16 2 MB QSPI flash (U1) | BOOT jumper JP1 pulls QSPI_SS low; RESET button K2 on RUN (10 kΩ pull-up, 100 nF). |
| Debug | SWD header P9: RUN, SWCLK, GND, SWDIO, **3.3 V** | SWCLK/SWDIO are dedicated debug pins, not GPIOs. Pin 5 is the only header pin carrying the board's 3.3 V rail. |
| USB | USB-C, D+/D− via 27 Ω, CC 5.1 kΩ to GND, USBLC6 ESD | VBUS feeds 5 V only with the USB-power jumper JP3 fitted (0.75 A fuse F1, diode D16). |
| Power | VIN 12/24 V (20 A fuse F2, TVS D14) → SS34 → TPS5450 buck → 5 V. 5 V → 0.35 A fuse F3 → AMS1117 → 3.3 V | Header 5 V pins come from the buck output. Headers 5V_Power and endstop 5V go through or alongside a 3.5 A fuse F4 with TVS D17. |
| Stepper drivers | 4 × TMC2209 soldered on: X (U3), Y (U4), Z (U5, two parallel outputs Z1/Z2), E0 (U8) | STEP/DIR/EN go through 100 Ω resistor arrays RN1–RN4. Sense resistors 0.11 Ω. Shared single-wire UART: TX = GPIO8 (100 Ω + 1 kΩ), RX = GPIO9 (100 Ω), 20 kΩ into each PDN_UART. Addresses: X 0, Z 1, Y 2, E0 3. |
| Driver DIAG | Jumpers P2 (X), P10 (Y), P18 (Z), P26 (E0) | A fitted jumper connects that driver's DIAG output to the matching endstop input net (for sensorless homing). **Remove it on any endstop you reuse.** |
| Endstops | P1 X, P4 Y, P6 Z, P8 E0 (pins: signal, GND, 5 V) | Signal pin: 10 kΩ pull-up to 3.3 V and **100 nF to GND at the connector**, then 100 Ω into a 74LVC2G34 buffer (U11/U12, 3.3 V), then to the GPIO. One-way, into the MCU only. |
| Fans / heaters / RGB | 74HCT365 buffer U7 at **5 V**, 10 kΩ pull-down on each input | FAN1/2/3 → AO3400-class MOSFETs (switching the low side of VCC). HB/HE → larger MOSFETs. RGB → RGB_1 (5 V logic) → header P24 and on-board WS2812-type LED. |
| Thermistors | P3 THB (GPIO26), P7 TH0 (GPIO27) | 4.7 kΩ pull-up to ADC_VREF, then 2 × 2.37 kΩ in series, BAV99 clamp, **100 nF on the GPIO**. Analogue only. |
| Laser | P12: pin 1 Laser_PWM (GPIO20 via 100 Ω), pin 2 GND, **pin 3 VCC (12/24 V)** | Shares GPIO20 with FAN3. Unfitted footprints (R7/R9/C22/Q1/R1) could link GPIO20 to GPIO29. All are marked NC, so there is no link now. |
| Probe / servo | P13 PROBE (signal, GND); P21 3-pin jumper (P_S / GPIO22 / PROBE); P11 SERVOS (GPIO29, 5 V, GND) | The P21 jumper connects GPIO22 either to the WD-DET optocoupler output (P_S) or straight to the PROBE pin. |
| Proximity input | P14 WD-DET → EL357N optocoupler → P_S (10 kΩ pull-down) | Input for a 12/24 V inductive sensor. Optocouplers are slow, so unusable for SPI. |
| UART0 / Pi header | P5 "5V_Power": 5 V, 5 V, GND, RXD0 (GPIO1), TXD0 (GPIO0) | **Direct to the RP2040, with no resistors or protection.** Free on your robot because the Pi talks over USB. |

## 3. Every RP2040 GPIO, what it's connected to, and whether you can reach it

Key: **Raw** = header pin wired straight to the GPIO (both directions). **Buffered-in** = reaches the GPIO through the 74LVC2G34, inputs only. **Buffered-out (5 V)** = GPIO drives a 74HCT365 input; the header carries a 5 V copy. **Internal** = no header.

| GPIO | Board net | Circuit path | Header access | SPI function (F1) | Usable for the IMU? |
|---|---|---|---|---|---|
| 0 | TXD0 | Direct | **P5 pin 5 (Raw)** | SPI0 RX (MISO) | **Yes: MISO** (hardware SPI0-compatible) |
| 1 | RXD0 | Direct | **P5 pin 4 (Raw)** | SPI0 CSn | **Yes: CS** |
| 2 | Z-EN | RN3 100 Ω → TMC Z ENN | Internal | SPI0 SCK | No |
| 3 | Y-STOP | P4 → 10 k PU + 100 nF → 100 Ω → U11 → GPIO | P4 pin 1 (Buffered-in) | SPI0 TX | INT only; can't be MOSI (input buffer) |
| 4 | X-STOP | P1 → … → U11 → GPIO | P1 pin 1 (Buffered-in) | SPI0 RX | INT; MISO only with C23 removed |
| 5 | Y-DIR | RN2 → TMC Y | Internal | SPI0 CSn | No |
| 6 | Y-STEP | RN2 → TMC Y | Internal | SPI0 SCK | No |
| 7 | Y-EN | RN2 → TMC Y | Internal | SPI0 TX | No |
| 8 | TX4 (TMC UART TX) | 100 Ω + 1 kΩ → all PDN_UART | Internal | SPI1 RX | No |
| 9 | RX4 (TMC UART RX) | 100 Ω | Internal | SPI1 CSn | No |
| 10 | X-DIR | RN1 → TMC X | Internal | SPI1 SCK | No |
| 11 | X-STEP | RN1 → TMC X | Internal | SPI1 TX | No |
| 12 | X-EN | RN1 → TMC X | Internal | SPI1 RX | No |
| 13 | E0-DIR | RN4 → TMC E0 | Internal | SPI1 CSn | No |
| 14 | E0-STEP | RN4 → TMC E0 | Internal | SPI1 SCK | No |
| 15 | E0-EN | RN4 → TMC E0 | Internal | SPI1 TX | No |
| 16 | E0-STOP | P8 → … → U12 → GPIO | **P8 pin 1 (Buffered-in)** | SPI0 RX | **Yes: INT** (remove E0-DIAG jumper) |
| 17 | FAN1_PWM | U7 → MOSFET → FAN1 | FAN1 is a switched VCC output | SPI0 CSn | No |
| 18 | FAN2_PWM | U7 → MOSFET → FAN2 | FAN2 is a switched VCC output | SPI0 SCK | No |
| 19 | Z-STEP | RN3 → TMC Z | Internal | SPI0 TX | No (would need a wire on RN3) |
| 20 | FAN3_PWM + Laser_PWM | U7 input (10 k PD) → FAN3 MOSFET; also 100 Ω → P12 pin 1 | P12 pin 1 (Raw via 100 Ω) | SPI0 RX | Possible spare. Toggling it also switches FAN3. **P12 pin 3 is 12/24 V.** |
| 21 | HB_PWM | U7 → bed MOSFET | HB is a switched VCC output | SPI0 CSn | No |
| 22 | P_2 | P21 middle pin; jumper to PROBE (P13) or P_S (opto) | **P21 middle pin or P13 pin 2 (Raw)** | **SPI0 SCK** | **Yes: SCK** (jumper on P_2–PROBE or removed, never on P_S) |
| 23 | HE_PWM | U7 pin 12 (10 k PD R69) → hotend MOSFET | Internal (solder point: U7 pin 12 / R69) | **SPI0 TX** | Only with one bodge wire. Enables full hardware SPI0. |
| 24 | RGB | U7 → RGB_1 (5 V) → P24 + on-board LED | P24 (Buffered-out, 5 V) | SPI1 RX | **No: 5 V output would damage the 3.3 V IMU** |
| 25 | Z-STOP | P6 → … → U12 → GPIO | P6 pin 1 (Buffered-in) | SPI1 CSn | INT alternative |
| 26 | THB / ADC0 | 4.7 k PU, 4.74 k series, 100 nF on pin | P3 (analogue) | SPI1 SCK | No (RC far too slow) |
| 27 | TH0 / ADC1 | Same as GPIO26 | P7 (analogue) | SPI1 TX | No |
| 28 | Z-DIR / ADC2 | RN3 → TMC Z | Internal | SPI1 RX | No |
| 29 | SERVOS / ADC3 | Direct (link footprints to GPIO20 unfitted) | **P11 pin 3 (Raw)** | SPI1 CSn | **Yes: MOSI** (PIO SPI) |

Header power you can use for the IMU: 5 V on P5, P11, the endstop headers and P24; **3.3 V only on SWD pin 5**; GND on almost every header.

## 4. Why hardware SPI doesn't quite fit

RP2040 SPI pins are fixed in groups of four (RX, CSn, SCK, TX repeating from GPIO0). The SPI peripheral can't be moved to arbitrary pins.

- **SPI0:** SCK = GPIO22 ✔ (raw), RX = GPIO0 ✔ (raw), CSn = GPIO1 ✔ (raw), TX = GPIO3 ✘ (driven by the endstop buffer), 7 ✘, 19 ✘, 23 ✘ (internal only).
- **SPI1:** RX would have to be 8/12/28 (internal) or 24 (5 V-buffered output). SCK/TX would be 10/14/11/15 (internal) or 26/27 (thermistor RC networks). Not viable.

So SPI0 is three-quarters there. Either let the PIO generate SPI on any pins (option A), or get at GPIO23 with one wire (option B).

## 5. Option A (recommended): PIO SPI, no board changes

| SEN0443 pin | Connect to | SKR Pico GPIO / header |
|---|---|---|
| VCC (3.3–5.5 V in, on-board LDO) | 5 V | E0-STOP P8 "5V" (or P5 5 V) |
| GND | GND | E0-STOP P8 "GND" (also run a GND alongside SCK) |
| SCK | SCK | **GPIO22**: PROBE P13 signal pin with the P21 jumper on P_2–PROBE, or P21 middle pin with the jumper removed |
| SDI (MOSI) | MOSI | **GPIO29**: SERVOS P11 signal pin |
| SDO (MISO) | MISO | **GPIO0**: P5 "IO0" (TXD0) |
| CS | CS (software) | **GPIO1**: P5 "IO1" (RXD0) |
| INT | Data-ready | **GPIO16**: E0-STOP P8 signal pin. Remove the E0-DIAG jumper. |
| FSY | GND | Tie to GND unless the DFRobot board already pulls it down |
| SCL/SDA, 3V3 | Leave open | SCL/SDA are the same pads as SCK/SDI in SPI mode |

Firmware (arduino-pico core 4.4.3 or later):

```cpp
#include <SoftwareSPI.h>
constexpr int PIN_SCK = 22, PIN_MISO = 0, PIN_MOSI = 29, PIN_CS = 1, PIN_INT = 16;
SoftwareSPI imuSPI(PIN_SCK, PIN_MISO, PIN_MOSI);   // PIO-driven; CS handled manually
// pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
// imuSPI.begin(); imuSPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3)); ...
```

- Hardware CS in `SoftwareSPI` must sit on SCK+1 (GPIO23, not reachable). Use a normal GPIO for CS, which the DFRobot driver does anyway.
- Initialise and use `imuSPI` only from core 1, as in test-plan Stage 3. It uses one PIO state machine. Check it doesn't clash with anything else of yours that uses PIO.
- Keep the 4 MHz you validated on the bench. Drop to 1–2 MHz if long jumper leads give errors.

## 6. Option B: true hardware SPI0 (one bodge wire)

Same as option A, except MOSI moves from GPIO29 to **GPIO23**. Solder a thin wire to **U7 (74HCT365) pin 12** or to the GPIO side of **R69 (10 kΩ)**.

- Then GPIO22 = SCK, GPIO23 = TX, GPIO0 = RX and GPIO1 = CSn are all native SPI0 pins: `SPI.setSCK(22); SPI.setTX(23); SPI.setRX(0); SPI.setCS(1);`.
- Side effects: GPIO23 also drives the hotend-heater MOSFET through U7, so the HE output and its LED will flicker with SPI data. The board isn't being used as a printer, so this doesn't matter: just leave the HE terminal empty. The 10 kΩ pull-down is a trivial extra load.
- This frees GPIO29 for another use.
- Since the printer circuits aren't needed, B costs only one solder joint. It gives the native SPI0 peripheral, which supports DMA and doesn't use a PIO state machine. It's the better long-term choice for the balancer; A is the quick way to start.

## 6a. The board isn't driving a printer, so these circuits are spare

This applies as of Nov 2026: the board is only driving the robot.

| Circuit | GPIO | Robot use |
|---|---|---|
| HE / HB heater outputs (large MOSFETs, switch VIN low side) | 23 / 21 | Lights, buzzer, solenoid, or a load switch for accessories. Same 10 A-class switching as a heater. |
| FAN1/2/3 outputs (small MOSFETs, VIN, flyback diodes fitted) | 17 / 18 / 20 | Indicator LEDs, a small fan for the drivers, a horn. |
| Endstops X/Y/Z/E0 (10 k pull-up, 100 nF filter) | 4 / 3 / 25 / 16 | Bumper switches, wheel-lift switch, IMU INT. The filter is useful here: it debounces switches. |
| Thermistor inputs THB/TH0 (ADC0/1) | 26 / 27 | Slow analogue sensing. The 4.7 kΩ pull-up to ADC_VREF is part of the circuit, so a battery-voltage divider must account for it. Easier: remove R11/R25 first. |
| PROBE / WD-DET | 22 | GPIO22 is the IMU clock in this plan. WD-DET (optocoupled 12/24 V input) is free if SCK moves elsewhere. |
| Spare drivers (E0, Z) | — | Extra steppers, e.g. a pan head or a lifting arm. |

**Option C (more raw pins, more soldering):** remove U7 (74HCT365). Every MOSFET gate keeps its own 20 kΩ pull-down, so all fan and heater outputs stay safely off. The six U7 input pads become raw GPIO pads, each with a 10 kΩ pull-down to GND: GPIO17, 18, 20, 21, 23 and 24. GPIO20 is then also clean at the laser header (pin 1, via 100 Ω; pin 3 is still VIN). This gives a full set of native SPI0 pins on 18/20/21/23, plus spares. The costs: you lose the fan/heater outputs and the on-board RGB LED, and the pull-downs load each pin (0.33 mA at 3.3 V, harmless). Only worth doing if you run out of pins.

## 7. Pin-specific cautions

- **E0-STOP as INT:** 100 nF sits right on the connector pin and the IMU's INT output has to charge it. Expect a small, constant edge delay of a few µs. Set INT to push-pull, active-high, latched until read. Then the RC filter can't swallow a short pulse, and the 10 kΩ pull-up only matters while the IMU is unpowered. Don't use open-drain: the 10 kΩ × 100 nF release takes about 1 ms. If Stage 2 timing shows a problem, remove C36 (E0-STOP filter cap).
- **GPIO22 jumper:** with P21 on P_S, the optocoupler output fights SCK. Use P_2–PROBE, or no jumper and the middle pin.
- **GPIO0/1** have no series resistor or ESD protection. Keep leads short and connect the IMU only with the board powered off.
- **GPIO20 / laser header:** usable as a spare line, but pin 3 is the 12/24 V supply. One wrong pin kills the IMU. Avoid it unless you really need a sixth line.
- **GPIO24 / RGB header** is a 5 V output. Never connect it to the IMU.
- **Board definition:** if the firmware is built as "Raspberry Pi Pico", the core treats GPIO25 as LED_BUILTIN (here Z-STOP, driven by a buffer), GPIO24 as VBUS sense and GPIO29 as VSYS sense. Build for "Generic RP2040", or make sure nothing touches those pins.
- **SPI-mode lock:** the ICG-20660L driver sets I2C_IF_DIS in SPI mode, and it stays set until power-off.

## 8. Fallback: I²C, no changes

I²C0 runs on GPIO0 (SDA) and GPIO1 (SCL) at P5. The SEN0443 has its own pull-ups. The bench test measured 455 µs per full read at 400 kHz, against 44 µs for SPI at 4 MHz. That is workable for the three-wheeler on core 1, but SPI is better for the balancer.

## Sources

- [bigtreetech/skr-pico (schematic, pin PDF, Klipper config, manual)](https://github.com/bigtreetech/skr-pico)
- [DFRobot SEN0443 wiki (pinout, VCC 3.3–5.5 V)](https://wiki.dfrobot.com/sen0443/)
- [arduino-pico 4.4.3 release: PIO SoftwareSPI](https://newreleases.io/project/github/earlephilhower/arduino-pico/release/4.4.3)
- RP2040 datasheet, GPIO function-select table (SPI pin groups)
