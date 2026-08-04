# R2W Robot Controller

R2W is a modular three-wheeled differential-drive robot. Two directly driven
NEMA17 wheels provide propulsion and a passive castor supports the chassis.

The control system is deliberately split:

- a BigTreeTech SKR Pico V1.0 performs time-sensitive motor control and safety;
- a Raspberry Pi Zero 2 W provides networking, the web interface and future
  high-level behaviour.

The system is working over USB serial. It can be controlled from the Pi command
line or from a phone-friendly web page on the local network.

## Documentation

Start with the [documentation guide](docs/README.md). It links the reference
manual in a suggested reading order.

The most useful operational pages are:

- [Operating the robot](docs/operations.md)
- [Safety model](docs/safety.md)
- [Testing and troubleshooting](docs/testing-troubleshooting.md)
- [Backup, deployment and recovery](docs/backup-recovery.md)

## Current capabilities

- non-blocking control of left and right stepper motors;
- differential-drive velocity mixing;
- finite distance and in-place turn commands;
- acceleration limiting;
- structured TMC2209 fault and operating telemetry;
- idle-only runtime current, microstep, acceleration and chopper-mode tuning;
- RP2040 PIO-generated STEP pulses, one state machine per wheel;
- versioned `R2W/1` Pi-to-Pico protocol;
- leased continuous velocity commands;
- finite jobs with completion events;
- normal stop and latched software emergency stop;
- duplicate-safe command retries;
- persistent Pi controller service;
- local-network mobile web interface.

## Important limitations

- Motion is open loop: there are no wheel encoders yet.
- Distances and angles use effective geometry and will vary with wheel slip,
  load, surface and battery condition.
- The software `ESTOP` is useful but is not a physical emergency-stop circuit.
- Battery voltage is not yet monitored.
- Motor tuning is held in RAM and returns to safe defaults after a Pico reboot.
- Manual web driving can occasionally respond more slowly than expected; this
  is recorded for timing and lease-path investigation.
- The web interface has no authentication or TLS. It must remain on a trusted
  local network and must not be exposed to the internet.

## Repository layout

```text
include/       Pico C++ interfaces and configuration
src/           Pico C++ implementation
pi/            Pi client, service, web server and web assets
docs/          System reference and learning notes
platformio.ini PlatformIO build configuration
CALIBRATION.md Historical calibration notes
TODO.md        Short development backlog
```

The Git repository on the development computer is the source of truth. The
copy under `~/r2w` on the Pi is a deployed runtime copy.
