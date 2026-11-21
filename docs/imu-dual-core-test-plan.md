# IMU and Dual-Core Test Plan (three-wheeler)

Goal: prove IMU input and a core 0 / core 1 split on the existing three-wheel R2W robot before attempting a balancing robot. Each stage has a pass check; don't move on until it passes.

## Stage 0: Preparation
- Choose the IMU: ICM-42688-P or BMI088 breakout with a data-ready (INT) pin, 3.3 V logic and onboard pull-ups.
- Confirm pins: check that GPIO 0/1 (Pi UART header, unused because the Pi uses USB) are free for I²C0 on the SKR Pico, and pick a pin for INT. Check the board schematic for anything already on those pins. Endstop inputs (GPIO 3/4) have filter components: fine for INT, poor for I²C.
- Mounting: rigid, near the wheel axle line, axes aligned with the robot (X forward, Z up). Write the orientation down.
- Baseline: start a new git branch. Record current behaviour, including loop timing and stop response, for comparison later.

**Pass:** wiring done, pins confirmed, branch created.

## Stage 1: IMU bring-up, single core
Everything on core 0. Just prove the sensor talks.
- I²C scan finds the device; its ID register reads correctly.
- Debug serial command prints raw gyro and accelerometer values.
- Tilt and rotate by hand to confirm each axis's direction and sign.
- Record noise at rest with motors off, then with motors running at several speeds (stepper vibration).

**Pass:** sensible values on every axis, signs match orientation notes, noise level with motors running is known.

## Stage 2: Data-ready timing
- Set the IMU output rate (start at 500 Hz) and enable its INT pin.
- Count interrupts per second; measure the time per I²C read.

**Pass:** interrupt rate matches the setting; read time fits comfortably inside one period (e.g. < 0.5 ms at 500 Hz).

## Stage 3: Core 1 introduced, IMU only
Motors stay on core 0. Core 1 only reads the sensor.
- Move IMU reads into `setup1()` / `loop1()`. Only core 1 ever touches the I²C bus.
- Core 1 publishes the latest readings and a sample counter to a shared, mutex-protected struct; core 0 adds them to STATUS.
- Measure core 1 timing jitter (min/max gap between samples, missed samples) while core 0 is busy with USB traffic, TMC polling and driving.

**Pass:** no missed samples and steady timing under full core 0 load. This proves the split works.

## Stage 4: Turn raw data into angles
- At startup, while stationary, measure the gyro bias.
- Complementary filter for pitch and roll; integrate gyro Z for heading (yaw).
- Tests: tilt to known angles and compare; leave still for 5 minutes and record heading drift; drive at speed and check vibration doesn't corrupt the angles.

**Pass:** tilt accurate to about 1°, yaw drift known and acceptable, angles stable while driving.

## Stage 5: Move motor ownership to core 1
Riskiest step. Wheels lifted.
- Core 1 runs `Motor` / `Stepper` updates at a fixed rate. Core 0 only posts setpoints (velocity, turn, MOVE/TURN jobs, stop, fault) into the shared struct.
- Safety decisions (leases, ESTOP latch, driver faults) stay on core 0; their results reach core 1 as flags it must obey.
- Retest everything: velocity driving, MOVE/TURN, DONE events, lease expiry, STOP, ESTOP.
- Measure stop latency (STOP received → pulses end) and compare with the Stage 0 baseline.

**Pass:** behaviour identical to before; stop latency the same or better.

## Stage 6: Make the IMU useful on the three-wheeler
- Gyro-assisted TURN: compare step-count turn angle with gyro-measured angle; optionally finish turns based on the gyro.
- Heading hold: keep straight-line driving straight despite slip or wheel mismatch.
- Tip or lift detection: pitch or roll past a limit stops the motors (first version of the balancer's "fallen" check).

**Pass:** turns more accurate than step counting alone; tip detection stops the motors reliably.

## Stage 7: Protocol, UI and docs
- Add tilt, heading and IMU health to the R2W STATUS reply; bump the protocol version if the format changes.
- Show tilt and heading on the web page; optionally log samples on the Pi for plotting.
- Update `firmware-architecture.md`, `safety.md` and `hardware-reference.md` with new pins, the core split and the setpoint struct.

## Rules throughout
- Wheels off the floor for Stages 3 and 5.
- Keep a fallback: a compile-time switch back to the single-core, no-IMU build.
- Core 1 never prints to serial; send anything to report back to core 0.
- Avoid flash writes while core 1 runs: on Arduino-pico, flash writes briefly pause the other core.

## What carries over to the balancer
Known loop rate and jitter, IMU noise under stepper vibration, a tuned tilt filter, a working core-to-core setpoint hand-off, and a tested fall detector.
