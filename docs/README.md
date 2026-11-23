# R2W Documentation Guide

This manual describes how the robot works now and how it is intended to evolve. Statements labelled **current** describe implemented or physically tested behaviour. Statements labelled **future** or **verify before use** are design ideas, not completed features. Dated logs (the commissioning results and the Phase 11 log) record what was true on the day. The [engineering reference](engineering-reference.md) gives the current state.

## Start here

**[Engineering reference](engineering-reference.md):** architecture, firmware, parameters, plant model, results, known issues, and the bring-up guide for a similar device.

## Suggested reading order

1. [System overview](system-overview.md): purpose, architecture and data flow.
2. [Hardware reference](hardware-reference.md): wiring, pin assignments, motors, IMU and power.
3. [Firmware architecture](firmware-architecture.md): the Pico classes, the two cores and why their responsibilities are separated.
4. [RP2040 PIO STEP generation](pio-step-generation.md): the state-machine program, FIFO protocol and timing ([Stepper_PIO.md](Stepper_PIO.md) is a gentler tutorial on the same code).
5. [Protocol reference](protocol.md): the complete `R2W/1` wire protocol, including the balance, IMU and telemetry commands.
6. [Pi executive controller](pi-controller.md): client, service and web interface. Command-line details and test scripts are in [../pi/README.md](../pi/README.md).
7. [Safety model](safety.md): leases, stops, balancing safety, faults and residual risks.
8. [Operating the robot](operations.md): everyday commands, balancing and service use.
9. [Testing and troubleshooting](testing-troubleshooting.md): test procedures and fault diagnosis.
10. [Backup, deployment and recovery](backup-recovery.md): source control, Pi deployment and rebuilding.
11. [Roadmap](roadmap.md): encoders, sensing, estimators and autonomy.

## Balancing robot

- [Balancing commissioning plan](balancing-commissioning-plan.md): the phased bring-up used for the two-wheel chassis.
- [Balancing results](balancing-results.md): dated measurements and test results for phases 1–10 and 11.0.
- [Phase 11 log](phase11-model.md): the model, the steps to LQR, the estimator and notch fixes, and the drive and turn tests (23–24 Nov).
- [Random questions](RandomQuestions.md): background questions and answers asked during commissioning.

## Reference and history

- [SKR Pico circuit reference](skr-pico-circuit-reference.md): board circuit notes and an SPI IMU option.
- [The Robot API and architecture boundary](step-3-robot-api.md): the refactor that made `Robot` the single public interface to physical motion and status (three-wheeler era).
- [IMU and dual-core test plan](imu-dual-core-test-plan.md): the original IMU bring-up tests (three-wheeler).

The documents incorporate relevant ideas from the earlier "Controlling 3-Wheeled Robot" design discussion. Older suggestions have not been treated as facts: uncertain power limits, unused pins and future features are marked for verification.

## Conventions

- Robot frame: X forward, Y left, Z up. **Pitch is positive when tipped forward,** and positive yaw or turn is to the left.
- Protocol and normal motion: millimetres, mm/s, degrees, deg/s.
- Balance controller internally: SI units (m, m/s, m/s², rad); `BAL_SET` angles are in degrees.
- Positive distance: forwards.
- X motor channel: left wheel. Y motor channel: right wheel.
