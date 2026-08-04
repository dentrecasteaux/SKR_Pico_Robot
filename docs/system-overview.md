# System Overview

## Purpose

R2W is a learning and development platform for a modular mobile robot. Its
architecture is intended to remain stable as sensors, odometry and autonomous
behaviour are added.

The robot currently has two driven wheels and one passive castor. Differential
drive is produced by commanding the left and right wheel at different speeds.

## Responsibility boundary

```text
Phone or computer
        |
        | HTTP on trusted Wi-Fi
        v
Pi web interface
        |
        | local Unix socket
        v
Pi robotd service
        |
        | R2W/1 over USB serial
        v
SKR Pico Robot API
        |
        +--> MotionController --> Motor --> Stepper --> RP2040 PIO --> STEP
        |
        +--> TMC2209 configuration and health
```

The Pi requests an outcome. The Pico performs the motion.

### Raspberry Pi Zero 2 W

The Pi is the executive controller. It owns:

- Wi-Fi and the user interface;
- one persistent connection to the Pico;
- validation and arbitration of external requests;
- a second control lease for manual driving;
- future planning, sensor fusion, logging and autonomy.

It does not generate STEP pulses and does not need real-time scheduling.

### SKR Pico

The Pico is the real-time motor controller. It owns:

- STEP, DIR and ENABLE signals;
- motor enable state;
- hardware-timed PIO pulse generation, one state machine per wheel;
- acceleration limiting;
- conversion from robot units to wheel movement;
- finite-job completion;
- TMC2209 setup and fault inspection;
- the final communications-loss lease;
- immediate software stop behaviour.

This division keeps motor safety available even if Linux, Wi-Fi, the browser or
the Pi application stops working.

## Motion types

### Continuous velocity

`SET_VELOCITY` asks the robot to maintain linear and angular velocity. It is
open-ended, so it must be renewed. If renewal stops, the Pico lease expires and
the motors stop.

### Finite job

`MOVE` and `TURN` describe bounded work. After accepting a job, the Pico owns
its completion. Loss of ordinary Pi communication does not cancel it because
the requested distance or angle is already bounded. `STOP`, `ESTOP` or a motor
fault can still cancel it.

## Present state

The following path has been tested:

```text
mobile browser
  -> robot_web.py
  -> robotd.py
  -> USB CDC serial
  -> RobotLink
  -> Robot
  -> MotionController
  -> motors
```

The Pi is a Raspberry Pi Zero 2 W running 64-bit Debian 13. The Pico appears as
an Arduino Raspberry Pi Pico USB ACM device and has a stable `/dev/serial/by-id`
identity.

## Architectural principles

- Non-blocking code: motor updates continue while commands are received.
- One owner per resource: only `robotd` opens the serial port in normal use.
- Physical units at interfaces: external code uses mm, mm/s, degrees and
  degrees/s.
- Layered safety: browser, Pi service and Pico each have a defined role.
- Human-readable protocol: bring-up and diagnosis can be performed in a serial
  terminal.
- Extensibility: sensors and autonomy should be added above or beside existing
  boundaries, not by bypassing them.
