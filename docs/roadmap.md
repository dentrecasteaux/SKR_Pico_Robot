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

An IMU can provide angular rate and acceleration. On a three-wheel platform it
can improve heading estimation when fused with encoders. It does not by itself
measure ground-relative position.

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

The present castor-based differential-drive robot is a useful platform for
learning the software and sensors, but adding an IMU does not turn the existing
controller into a balancing robot. Treat balancing as a later dedicated
milestone with its own safety analysis.

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
