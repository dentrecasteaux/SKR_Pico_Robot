# Hardware Reference

## Current hardware

| Item | Current configuration |
|---|---|
| Motor controller | BigTreeTech SKR Pico V1.0, RP2040 |
| Executive controller | Raspberry Pi Zero 2 W |
| Drive motors | Two NEMA17 stepper motors, direct drive |
| Motor drivers | On-board TMC2209 |
| Chassis | Two driven wheels and one passive castor |
| Battery | 4S LiPo, 14.8 V nominal and 16.8 V fully charged |
| Pi-to-Pico link | USB CDC serial |
| Wheel diameter | 61 mm effective, provisional |
| Wheel track | 188 mm effective, provisional |
| Motor resolution | 200 full steps/revolution |
| Microstepping | 1/4 |
| Run current | 400 mA RMS |

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
`Stepper` construction boundary to account for the mirrored physical mounting:
left is inverted and right is not.

The shared TMC2209 UART uses:

| Signal | GPIO |
|---|---:|
| TX | 8 |
| RX | 9 |

UART configures and inspects the TMC2209 drivers. Motion itself uses STEP and
DIR.

## USB connection

The Pi Zero must use its USB data/OTG port, not its power-only port. The tested
cable provides VBUS, ground, D+ and D− and puts the Pi controller into USB host
mode. A USB-C configuration-channel conductor is not needed in this
micro-USB-to-USB-C arrangement.

Normal Pico enumeration looks similar to:

```text
ID 2e8a:00c0 Arduino RaspberryPi Pico
/dev/ttyACM0
/dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_71355045E8FA1029-if00
```

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
- IMU;
- battery voltage or current sensing;
- physical emergency-stop switch and independent motor-power interruption;
- autonomous-navigation sensors.

