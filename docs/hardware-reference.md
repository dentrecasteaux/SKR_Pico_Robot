# Hardware Reference

## Current hardware

| Item | Current configuration |
|---|---|
| Motor controller | BigTreeTech SKR Pico V1.0, RP2040 |
| Executive controller | Raspberry Pi Zero 2 W |
| Drive motors | Two Rtelligent (SZruitech) **42A08** NEMA17 steppers, direct drive (see below) |
| Motor drivers | On-board TMC2209 (heatsinks fitted) |
| Chassis | **Two-wheel balancing chassis** (Nov 2026) with 3D-printed "training wheel" tilt stops. The electronics moved here from the three-wheeler (two driven wheels and a castor). |
| Battery | 4S LiPo, 14.8 V nominal and 16.8 V fully charged |
| Pi-to-Pico link | USB CDC serial |
| Wheel diameter | 62.33 mm effective (roll test: 1958 mm / 10 rev; `move 500` measured 500 mm) |
| Wheel track | 223 mm (tyre centrelines, measured; not yet turn-calibrated) |
| Motor resolution | 200 full steps/revolution |
| Microstepping | 1/16 (driver interpolation to 1/256) |
| Run current | 1000 mA RMS (runtime ceiling 1300 mA) |
| Mass | 1880 g total |

The effective wheel geometry is a software calibration, not merely a ruler
measurement. It compensates approximately for tyre deformation and open-loop
turning behaviour. It will be refined when encoders are fitted.

## Confirmed Pico pin assignment

| Function | Left / X | Right / Y |
|---|---:|---:|
| STEP | GPIO 11 | GPIO 6 |
| DIR | GPIO 10 | GPIO 5 |
| ENABLE | GPIO 12 | GPIO 7 |
| TMC UART address | 0 | 2 |
| DIAG route, unused | GPIO 4 | GPIO 3 |

Both enable signals are active-low. Direction inversion is handled at the
`Stepper` construction boundary to account for the mirrored physical mounting.
On the balancing chassis **left is not inverted and right is inverted**. The
three-wheeler was the opposite. Checked with `move 200` and `turn 90` on
22 Nov 2026.

## Drive motors: Rtelligent 42A08

| Parameter | Value |
|---|---|
| Type | Hybrid open-loop bipolar, 1.8° (200 steps/rev), 4 wires |
| Rated current | 1.8 A/phase (listings don't say RMS or peak) |
| Holding torque | 0.68–0.71 N·m (published) |
| Phase resistance | 2.0 Ω ±10% published; about 2.4–2.7 Ω measured with a meter whose leads read 0.8–1.2 Ω |
| Phase inductance | 4.8 mH published |
| Rotor inertia | 114 g·cm² |
| Body / shaft | 42 × 42 × 60 mm; 5 mm × 24 mm shaft |
| **Coil pairs** | **Red–Blue** and **Green–Black**. Both motors are crimped identically. |

**Heat run** (10 min at 2 rev/s, 1.0 A RMS, StealthChop, about 28 °C room):
motors 33 °C, TMC2209 heatsinks 50 °C, no driver warnings, CS_ACTUAL 17 on
both drivers. The drivers, not the motors, limit the current. A balancer runs
at full run current the whole time, so 1.0 A is the working value. Go higher
only with airflow.

**Resonance:** in SpreadCycle the motors are audibly rougher and shake the IMU
more at 1–2 rev/s (mid-band resonance), and quieter again by 3 rev/s.
StealthChop damps it. See `vibe_test.py` in `pi/README.md`.

The shared TMC2209 UART uses:

| Signal | GPIO |
|---|---:|
| TX | 8 |
| RX | 9 |

UART configures and inspects the TMC2209 drivers. Motion itself uses STEP and
DIR. In the arduino-pico firmware this UART (UART1) is `Serial2`.

## IMU (MPU6050, GY-521)

The IMU runs on the RP2040's second core (see
[firmware architecture](firmware-architecture.md)).

| GY-521 pin | SKR Pico connection | GPIO | Notes |
|---|---|---:|---|
| VCC | P5 pin 1 or 2 (5 V) | — | The GY-521 has its own 3.3 V regulator |
| GND | P5 pin 3 | — | |
| SCL | P5 pin 4 (IO1) | 1 | I²C0 SCL, 400 kHz |
| SDA | P5 pin 5 (IO0) | 0 | I²C0 SDA |
| INT | E0-STOP signal pin | 16 | Data-ready, 500 Hz; **remove the E0-DIAG jumper** |
| AD0 | open | — | Address 0x68 |

The SDA, SCL and INT signals are 3.3 V logic and must never be tied to 5 V. The
GY-521's own 4.7 kΩ pull-ups to its 3.3 V are used. Secure the INT lead: if it
comes off, GPIO16 sits high and the firmware falls back to about 166 reads per
second (reported as stalls) instead of 500 Hz.

The sensor's mounting orientation is set in `Config.h` (`IMU_AXIS_FROM`,
`IMU_AXIS_SIGN`), mapping sensor axes to the robot frame: X forward, Y left,
Z up. It is the identity mapping until the board is mounted rigidly and its
orientation measured.

## USB connection

The Pi Zero must use its USB data/OTG port, not its power-only port. The tested
cable provides VBUS, ground, D+ and D− and puts the Pi controller into USB host
mode. A USB-C configuration-channel conductor is not needed in this
micro-USB-to-USB-C arrangement.

Normal Pico enumeration looks similar to:

```text
ID 2e8a:000a Arduino RaspberryPi Pico
/dev/ttyACM0
/dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_<serial>-if00
```

The firmware uses the arduino-pico core (USB ID `2e8a:000a`). Its USB
manufacturer and product strings are set in `platformio.ini` to match the
earlier mbed build (`2e8a:00c0`), so the `/dev/serial/by-id` name the Pi looks
for is unchanged.

If it appears as `2e8a:0003 Raspberry Pi RP2 Boot`, the Pico is in its
bootloader and will not provide the serial protocol. Power-cycle or reset it
without holding BOOTSEL after installing valid firmware.

## Power notes

- Use a suitable regulated supply for the Pi; do not assume a motor-controller
  rail is an appropriate Pi supply.
- The Pi and Pico need a common signal reference when using a GPIO UART.
  USB already supplies the connection reference.
- Avoid creating conflicting power paths through USB and board power inputs.

**Verify before use:** the exact isolation between USB VBUS, the SKR Pico power
input and any external Pi regulator has not been documented by this project.
Check the exact board schematic and planned wiring before permanent untethered
power is installed.

**Verify before use:** an earlier design discussion considered a future 6S
battery. This project currently uses 4S. Do not connect 6S unless every affected
board, regulator, capacitor and protection component has been checked against
its maximum voltage with suitable margin.

## TMC2209 DIAG and StallGuard

The board can route X and Y DIAG signals through its end-stop circuitry to
GPIO 4 and GPIO 3. The required jumpers are not part of the current control
system.

DIAG/StallGuard may later help detect a stalled stepper under suitable motion
conditions. It is not wheel position feedback, cannot measure odometry and must
not be treated as a physical emergency stop.

## Hardware not yet fitted or integrated

- wheel encoders;
- rigid IMU mounting (on the balancing chassis the MPU6050 is taped to the
  battery layer, about 10° pitched relative to the platforms; pitch and yaw
  signs are verified, and the offset is absorbed by the measured balance
  angle);
- battery voltage or current sensing;
- physical emergency-stop switch and independent motor-power interruption;
- autonomous-navigation sensors.

