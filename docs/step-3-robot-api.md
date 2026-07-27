# Step 3: The Robot API and Architecture Boundary

## Purpose of this step

This step did not add new Pi motion commands. It reorganised the Pico firmware
so those commands can be added safely in the next step.

The important change is that `Robot` is now the single application-level
interface to the physical robot. Code that receives a request does not need to
know which motor, driver, pin, or UART is involved.

The resulting responsibility split is:

```text
Pi or serial terminal
        |
        v
    RobotLink             Parses and formats the R2W/1 protocol
        |
        v
      Robot               Owns robot actions, state and status
        |
        +----> MotionController ----> Motor ----> Stepper ----> GPIO
        |
        +----> TMC2209 drivers
```

This preserves the central design rule: the Pi requests motion, while the Pico
owns real-time motor control.

## Before this step

`RobotLink` received references to both motors and both TMC2209 drivers:

```text
RobotLink
  ├── left Motor
  ├── right Motor
  ├── left TMC2209
  └── right TMC2209
```

It used those objects directly to construct a status reply. This worked for
`PING` and `STATUS`, but it created an undesirable dependency:

- the protocol layer knew how the robot hardware was assembled;
- adding motion commands would tempt the protocol parser to control motors
  directly;
- safety rules could become split between `RobotLink`, `MotionController`, and
  `main.cpp`;
- a future hardware change could require changes to the communications code.

The existing `Robot` class also had only `begin()` and `update()`, so it was not
yet a useful boundary for external commands.

## The public Robot API

The public methods declared in `Robot.h` describe what an external controller
may ask the robot to do:

```cpp
void begin();
void update();

bool setVelocity(float linearMmPerSecond,
                 float turnDegreesPerSecond);
bool startMove(float distanceMm);
bool startTurn(float angleDegrees);
void stop();
Status status();
```

These methods use physical robot units rather than motor implementation units:

- linear velocity is in millimetres per second;
- angular velocity is in degrees per second;
- distance is in millimetres;
- turn angle is in degrees.

The API does not expose STEP pins, pulse intervals, motor directions, wheel
mixing, or TMC2209 UART registers. Those remain internal Pico concerns.

### `begin()`

Initialises the two motors, starts the TMC2209 UART, and configures both
drivers. Hardware initialisation therefore belongs to the robot application
object rather than the protocol layer.

### `update()`

Runs the non-blocking motor updates on every pass through the Arduino loop. It
also observes finite motion completion and returns the application mode to
`IDLE` when both motors have finished.

Nothing in `update()` waits for a move to finish. Serial processing can continue
while the motors are running.

### `setVelocity()`

Accepts a linear and angular velocity request, checks it against the configured
safety limits, and passes it to `MotionController` for differential-drive
mixing.

If accepted, it records:

- `MODE=VELOCITY`;
- the requested linear setpoint;
- the requested angular setpoint.

It returns `false` when a request is outside the configured limits. This gives
the future protocol dispatcher one clear result from which to produce either an
`ACK` or an `ERR`.

The lease timer described in `protocol.md` is not implemented in this step. It
will be added as part of the safety behaviour.

### `startMove()`

Requests a bounded straight-line move from `MotionController`. If the motion
layer accepts the request, `Robot` records `MODE=MOVE`.

The Pico remains responsible for generating all pulses and detecting when the
move has finished. The caller does not loop over steps or wait synchronously.

Job IDs and asynchronous `DONE` replies are not implemented in this step.

### `startTurn()`

Works like `startMove()`, but requests a bounded in-place rotation and records
`MODE=TURN`.

The left/right wheel directions, geometry conversion, and pulse count remain
inside the Pico motion layers.

### `stop()`

Calls the existing immediate motor stop, clears the stored velocity setpoints,
and returns the application state to `IDLE`.

This method is not latched. Latched emergency-stop behaviour will be a separate
operation in the safety step.

### `status()`

Builds and returns a snapshot of the robot's application state and driver
health. It does not print protocol text.

That distinction matters:

```text
Robot::status()             provides structured robot data
RobotLink::sendStatus()     formats that data as an R2W/1 STAT line
```

Other consumers can therefore use the same robot status later without parsing
serial text.

## Robot state tracking

`Robot` now tracks one of four application modes:

```text
IDLE
VELOCITY
MOVE
TURN
```

It also records the requested linear and angular velocity setpoints.

For example:

```text
velocity 20 0
```

is routed through `Robot::setVelocity(20, 0)`. A subsequent protocol status
request can therefore report:

```text
MODE=VELOCITY V_SET=20.00 W_SET=0.00
```

After `Robot::stop()`, the same status fields become:

```text
MODE=IDLE V_SET=0.00 W_SET=0.00
```

This is application state, not odometry or measured wheel speed. Encoders may
provide measured values later without changing the meaning of these setpoint
fields.

The status structure already contains placeholders for emergency-stop and job
state. They remain at their safe default values until those features are
implemented:

```text
ESTOP=0
JOB=0
```

## RobotLink now depends only on Robot

The old constructor was conceptually:

```cpp
RobotLink(stream, leftMotor, rightMotor, leftDriver, rightDriver, ...);
```

The new constructor is:

```cpp
RobotLink(stream, robot, ...);
```

`RobotLink` no longer stores motor or TMC2209 references. When it receives:

```text
R2W/1 CMD 11 STATUS
```

the flow is:

```text
1. SerialTransport produces one complete line.
2. Protocol validates and identifies the STATUS command.
3. RobotLink calls robot.status().
4. Robot returns a Robot::Status snapshot.
5. RobotLink formats that snapshot as R2W/1 STAT.
```

The output observed during the hardware test was:

```text
R2W/1 STAT 11 MODE=IDLE ESTOP=0 FAULTS=NONE JOB=0 V_SET=0.00 W_SET=0.00 X_DRIVER=OK_IDLE Y_DRIVER=OK_IDLE UPTIME_MS=43117 RX_AGE_MS=17
```

The driver register reads occur behind the `Robot` interface. `RobotLink` sees
only two `DriverStatus` values and converts them to protocol tokens such as
`OK_IDLE`, `OK_ACTIVE`, or `NO_REPLY`.

## Lifecycle changes

The Arduino entry points are now small coordinators:

```cpp
void setup()
{
    // Start USB serial.
    robot.begin();
    // Print existing start-up diagnostics.
}

void loop()
{
    robotLink.update();
    robot.update();
}
```

The implementations of `Robot::begin()`, `Robot::update()`, and the public
control methods now live in `Robot.cpp`. This keeps `main.cpp` focused on
assembling objects, maintaining the legacy terminal commands during migration,
and calling the two top-level update functions.

The order in `loop()` is not a transfer of timing responsibility to
`RobotLink`. Communications processing is bounded and non-blocking, then
`Robot::update()` continues deterministic pulse scheduling on the Pico.

## Why this helps Step 4

The next protocol commands can be dispatched through one narrow interface:

```text
SET_VELOCITY  -> Robot::setVelocity()
MOVE          -> Robot::startMove()
TURN          -> Robot::startTurn()
STOP          -> Robot::stop()
```

Lease expiry, job allocation, cancellation, and latched emergency-stop
behaviour can be added to `Robot` as application safety rules. `RobotLink` will
remain responsible for parsing requests and formatting replies.

This avoids two particularly dangerous designs:

- a serial parser directly enabling or stepping motors;
- the Pi becoming responsible for real-time pulse generation or motor safety.

## What this step deliberately did not do

To keep the refactor independently testable, it did not yet add:

- protocol motion-command parsing;
- velocity leases;
- job IDs or `DONE` events;
- latched `ESTOP`;
- `CLEAR_ESTOP`;
- command replay protection;
- encoder or measured-velocity status.

Those features can now be implemented without breaking the hardware boundary
established here.

## Hardware verification

The completed tests demonstrated that:

- `PING` still works after the refactor;
- idle status is reported through `Robot::Status`;
- the existing `velocity 20 0` command still drives both wheels;
- protocol status reports `MODE=VELOCITY` and the correct setpoints;
- the existing `off` command stops and disables both motors;
- protocol status returns to `MODE=IDLE` with zero setpoints and idle drivers.

This confirms both the new architecture boundary and preservation of the
existing motor behaviour.
