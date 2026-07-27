# R2W Documentation Guide

This manual describes both how the robot works now and how it is intended to
evolve. Statements labelled **current** describe implemented or physically
tested behaviour. Statements labelled **future** or **verify before use** are
design ideas, not completed features.

## Suggested reading order

1. [System overview](system-overview.md) — purpose, architecture and data flow.
2. [Hardware reference](hardware-reference.md) — wiring, pin assignments and
   present electrical assumptions.
3. [Firmware architecture](firmware-architecture.md) — the Pico classes and
   why their responsibilities are separated.
4. [Protocol reference](protocol.md) — complete `R2W/1` wire protocol.
5. [Pi executive controller](pi-controller.md) — client, service and web
   interface.
6. [Safety model](safety.md) — leases, stops, faults and residual risks.
7. [Operating the robot](operations.md) — everyday commands and service use.
8. [Testing and troubleshooting](testing-troubleshooting.md) — test procedures
   and fault diagnosis.
9. [Backup, deployment and recovery](backup-recovery.md) — source control,
   Pi deployment and rebuilding.
10. [Roadmap](roadmap.md) — encoders, IMU, telemetry and autonomy.

## Design history

[The Robot API and architecture boundary](step-3-robot-api.md) explains the
important refactor that made `Robot` the single public interface to physical
motion and status.

The documents incorporate relevant ideas from the earlier “Controlling
3-Wheeled Robot” design discussion. Older suggestions have not been treated as
facts: uncertain power limits, unused pins and future features are explicitly
marked for verification.

## Conventions

- Linear velocity: millimetres per second (`mm/s`)
- Angular velocity: degrees per second (`deg/s`)
- Distance: millimetres (`mm`)
- Angle: degrees (`deg`)
- Positive distance: forwards
- Positive angular velocity or angle: left turn
- X motor channel: left wheel
- Y motor channel: right wheel

