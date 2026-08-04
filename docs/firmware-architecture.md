# Pico Firmware Architecture

## Main loop

The Arduino main loop repeatedly updates the robot and communications:

```text
robot.update()
robotLink.update()
```

Neither operation waits for a complete movement. Each pass performs a small
amount of work and returns, allowing serial handling and motor pulse scheduling
to coexist.

## Layers

```text
RobotLink
   |
 Robot
   |
 MotionController
   |
 Motor (left and right)
   |
 Stepper
   |
 GPIO
```

TMC2209 driver objects sit beside the motion chain. They configure current,
microstepping and operating mode, and report driver health.

### `Stepper`

`Stepper` owns the STEP, DIR and ENABLE pins. It understands electrical details:

- active-low enable;
- direction inversion for physical motor mounting;
- step pulse edges.

Each `Stepper` owns an RP2040 PIO state machine. PIO generates 5 microsecond
STEP pulses, maintains pulse timing independently of the main loop, counts
finite moves, and signals completion. It does not understand millimetres,
turning or acceleration.

### `Motor`

`Motor` owns non-blocking movement for one wheel:

- current and target step rate;
- acceleration limiting;
- continuous motion;
- finite step counts;
- busy and enabled state.

Its `update()` method applies acceleration and sends changed pulse periods to
PIO through the state-machine FIFO. Slow serial or UART work can delay motion
planning updates, but it no longer stretches individual STEP periods.

### `MotionController`

`MotionController` coordinates the two motors. It:

- mixes linear and angular commands into left and right wheel speeds;
- converts physical geometry into steps;
- starts straight distance moves;
- starts in-place turns;
- stops both motors.

The current geometry is 61 mm effective wheel diameter, 188 mm effective wheel
track, 200 full steps per revolution and 1/4 microstepping.

### `Robot`

`Robot` is the application boundary. External controllers request actions only
through its public API:

```cpp
setVelocity(...)
startMove(...)
startTurn(...)
stop()
estop()
clearEstop()
status()
```

It owns:

- application mode;
- velocity lease timing;
- emergency-stop latch;
- active and most recently completed job;
- job completion events;
- driver permission checks;
- the structured status snapshot.

This is explained historically in
[The Robot API and architecture boundary](step-3-robot-api.md).

### `Protocol`

`Protocol` parses one complete `R2W/1` command into structured fields. It
validates framing, command names, required fields, duplicate fields, numeric
format and configured ranges.

It does not move hardware.

### `SerialTransport`

`SerialTransport` accumulates serial bytes into newline-terminated messages
without blocking. It detects an overlong input line and discards it safely.

### `RobotLink`

`RobotLink` connects protocol commands to the `Robot` API and formats:

- `ACK` and `ERR` direct responses;
- `PONG` and `STAT` responses;
- asynchronous `DONE` job events.

It depends on `Robot`, not directly on motors, GPIO or TMC2209 objects.

It also caches the most recent motion-command response. Repeating the identical
sequence and command replays the response; reusing that sequence for different
text produces `SEQUENCE_CONFLICT`. This prevents a lost acknowledgement from
starting a finite move twice.

## State model

| Mode | Meaning |
|---|---|
| `IDLE` | No commanded motion |
| `VELOCITY` | Open-ended velocity with an active Pico lease |
| `MOVE` | Finite straight-line job |
| `TURN` | Finite in-place turn job |
| `ESTOP` | Software emergency-stop latch is set |

`V_SET` and `W_SET` are requested setpoints, not measured velocity. The present
robot has no encoder feedback.

## Current configured limits

| Setting | Value |
|---|---:|
| Maximum motor speed | 2,000 steps/s |
| Acceleration | 200 steps/s² |
| Maximum linear request | 500 mm/s |
| Maximum angular request | 180 deg/s |
| Maximum finite distance | 10,000 mm |
| Maximum finite angle | 3,600 deg |
| Velocity lease range | 100–2,000 ms |
| TMC run current | 400 mA RMS |
| Microstepping | 1/4 |

These are firmware safety limits, not proof that every permitted request is
safe for every payload and surface.

## Legacy serial commands

The older human-readable commands remain in `main.cpp` for bench diagnosis.
Lines beginning with `R2W` go to `RobotLink`; other lines go to the legacy
handler.

Normal Pi control should use `R2W/1`. Legacy motor-level commands bypass parts
of the application protocol and should not become a second production control
API.

## Future real-time improvements

The RP2040 supports PIO and DMA. Possible future uses include PIO/DMA step
generation and DMA-assisted sensor sampling. DMA transfers data between memory
and peripherals; it does not directly “drive a pin” without a suitable
peripheral such as PIO.

The present software scheduler is adequate for the tested speeds. Optimisation
should follow measurement of timing limits, not precede it.
