# R2W Robot Controller

R2W is a modular robot built on two directly driven NEMA 17 stepper wheels. It began as a three-wheeled differential-drive robot with a castor. Since November 2026 its electronics run a **two-wheel self-balancing chassis**.

The control system is deliberately split:

- a BigTreeTech SKR Pico V1.0 (RP2040) performs the time-critical work: motor control, the IMU and the balance controller (500 Hz), and safety;
- a Raspberry Pi Zero 2 W provides networking, the phone web page, run logging and tuning persistence.

The two talk over USB serial with the versioned `R2W/1` protocol. The robot is controlled from a phone web page or from the Pi command line.

## Documentation

- **[Engineering reference](docs/engineering-reference.md):** the single technical reference. Architecture, firmware, parameters, the measured plant, results, known issues, and a detailed guide to bringing a similar balancing device online.
- [Documentation guide](docs/README.md): every document, in reading order.

The most useful operational pages are:

- [Operating the robot](docs/operations.md)
- [Safety model](docs/safety.md)
- [Testing and troubleshooting](docs/testing-troubleshooting.md)
- [Backup, deployment and recovery](docs/backup-recovery.md)

## Current capabilities

- **Two-wheel balancing:** full-state controller (KP 51.4, KD 4.37, KV 6.89, KX 6.48 by default) on core 1, straight after each 500 Hz IMU sample.
- **Attitude estimate:** complementary filter, τ 0.5 s. The accelerometer is compensated for the IMU's own acceleration (`ACCEL_COMP=2`), and the turn coupling is learned.
- **A 20.5 Hz notch** on the controller output, against the drive-train resonance.
- **Driving and turning while balancing,** with an S-curve, drive feed-forward and automatic trim.
- **Balance safety:** arming window, soft start, tilt, runaway and IMU-stale trips, a black box, and a drive lockout after any fall.
- **Telemetry:** 500 Hz rings, 250 Hz runs logged on the Pi, identification kicks.
- **Phone balance page:** status, runs with kicks, hold-to-drive, live tuning with "save as start-up", run plots. The three-wheeler bench console is at `/bench`.
- **The original motion stack:** leased velocity, finite moves and turns, acceleration limiting, PIO step generation, TMC2209 configuration and health, stop and latched software e-stop, duplicate-safe retries.

## Important limitations

- Motion is open loop: there are no wheel encoders. Speed and position in the controller are the commanded values.
- The software `ESTOP` is useful but is not a physical emergency-stop circuit.
- Battery voltage is not monitored.
- Live tuning is held in RAM. Only the web page's saved tuning is re-applied after an SKR restart.
- The web interface has no authentication or TLS. It must stay on a trusted local network and must never be exposed to the internet.
- The balancing chassis is a prototype. Its joint damping varies, so compare test runs within one session.

## Repository layout

```text
include/, src/   SKR Pico firmware (C++, PlatformIO, arduino-pico core)
pi/              Pi service (robotd), web server and pages, CLI, test scripts
analysis/        Offline analysis (Phase 11 model fitting, scratch scripts)
tools/           deploy-pi.sh (Pi software), flash-skr.sh (SKR firmware over Wi-Fi, via the Pi)
docs/            Reference manual, commissioning plan and results
platformio.ini   Build configuration
CALIBRATION.md   Historical three-wheeler calibration notes
TODO.md          Short backlog
```

The Git repository on the development computer is the source of truth. The copy under `~/r2w` on the Pi is a deployed runtime copy. Run logs live in `~/runs` on the Pi and are synced to `../runs` on the Mac.
