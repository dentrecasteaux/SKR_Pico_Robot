# Roadmap and Design Ideas

The current Pi/Pico boundary should remain stable. New features should extend
status and high-level behaviour without moving STEP generation to the Pi.

## Near-term documentation and reliability

- keep hardware wiring and calibration records current;
- add automated host-side parser tests;
- add Pico unit or integration tests where practical;
- record release tags for physically tested milestones;
- remove or clearly gate legacy commands before less-supervised operation;
- design and fit a physical emergency-stop circuit.

## Battery monitoring

Future goals:

- battery voltage;
- optional current and consumed-charge estimate;
- low-voltage warning;
- safe motor disable threshold;
- web telemetry and event logging.

The sensing divider, ADC protection, calibration and behaviour under undervoltage
must be designed before implementation. The controller must not infer safe LiPo
operation from nominal voltage alone.

## Wheel encoders

Encoders enable:

- measured wheel speed;
- closed-loop speed control;
- travelled distance;
- odometry;
- detection of disagreement between commanded and measured motion.

Encoder sampling and low-level wheel control belong on the Pico. Pose estimation
and navigation can run on the Pi.

## IMU

**Current:** an MPU6050 runs on the SKR's second core at 500 Hz and provides
the pitch estimate for balancing. Future: fusing it with encoders for heading
and odometry, and a Kalman or LQG estimator for the controller comparison. On
its own it does not measure ground-relative position.

The Pi is a suitable place for higher-level sensor fusion. Time-critical sample
capture may remain on the Pico if required.

## Odometry and autonomy

A likely evolution is:

```text
encoders + IMU
       |
       v
state estimation / odometry
       |
       v
manual-autonomous command arbitration
       |
       v
R2W motion requests
       |
       v
Pico motor control
```

The Pi service should become the sole arbiter between web/manual commands and
autonomous commands. Two controllers must never independently command the Pico.

ROS 2 may later connect to the Pi service or replace part of its high-level
API. The `R2W/1` Pico interface can remain small and deterministic.

## Obstacle and navigation sensors

Possible additions include range sensors, lidar or a camera. Spare stepper
channels might operate a sensor mast, lidar rotation, gripper or lift, but exact
SKR Pico channel labels, pins, current capacity and mechanical suitability must
be verified against the board schematic before assignment.

## Balancing robot concept

The earlier design discussion considered eventual two-wheel balancing. That is
a substantially different control problem:

- high-rate IMU acquisition;
- fast, deterministic feedback control;
- encoder feedback;
- careful motor torque and latency characterisation;
- a safe fall/disable strategy;
- new mechanical centre-of-mass design.

The castor-based robot was a useful platform for learning the software and
sensors. Balancing was treated as a dedicated milestone with its own plan and
safety layer.

**Status, Nov 2026:** the electronics now run a dedicated two-wheel balancing
chassis. Commissioning follows a phased plan (mechanical measurements, motors,
power, directions, IMU, logging, plant identification, safety layer, PD, outer
loops, then a PID vs LQR comparison). See
[balancing-commissioning-plan.md](balancing-commissioning-plan.md) and the
running results in [balancing-results.md](balancing-results.md). **Phases
1–10 are complete, and Phase 11 is under way** (model, LQR trial, estimator
and notch fixes: [phase11-model.md](phase11-model.md)). The current state and
open items are in the [engineering reference](engineering-reference.md).
Remaining balancing work: a turn-lean feed-forward, the comparison protocol,
encoders, battery sensing and a physical e-stop.

## RP2040 PIO and DMA

PIO STEP generation is complete and tested with one state machine per motor.
Possible future optimisation:

- DMA feeding PIO command buffers;
- UART DMA for sustained communications;
- ADC DMA for regular sensor sampling;
- SPI DMA for high-rate sensors.

These techniques are valuable when measurements demonstrate that the present
loop scheduler is limiting performance. They add complexity and should preserve
the existing `Robot` and `MotionController` interfaces.

## Protocol evolution

Version 1 can gain optional status fields without breaking clients that ignore
unknown reply fields. Likely future fields include:

- battery voltage and current;
- encoder counts;
- measured wheel velocities;
- pose or odometry timestamp;
- IMU or estimator health;
- physical-estop state.

A breaking change to field meaning or framing requires a new major version,
such as `R2W/2`.
