# Pico Firmware Architecture

## Core and main loops

The firmware uses the Earle Philhower **arduino-pico** core (selected in
`platformio.ini`), which runs `setup()`/`loop()` on core 0 and
`setup1()`/`loop1()` on core 1.

Core 0, the main loop, repeatedly updates communications and the robot:

```text
robotLink.update()
robot.update()
Imu::setMotorsStopped(!robot.motorsBusy())
```

Neither operation waits for a complete movement. Each pass performs a small
amount of work and returns, allowing serial handling and motor pulse scheduling
to coexist.

Core 1 runs only the IMU (`Imu::beginCore1()` / `Imu::updateCore1()`). It owns
I²C0 and the GPIO16 data-ready interrupt, never prints, and never touches the
motors, the TMC UART or USB. Building with `-DR2W_IMU=0` leaves core 1 unused
and makes every IMU command return `UNSUPPORTED_COMMAND`.

## IMU module (`Imu`)

Per 500 Hz sample, core 1:

- reads the 14-byte sample with a non-blocking I²C FIFO burst (about 430 µs of
  bus time, polled rather than waited for);
- maps sensor axes into the robot frame (`IMU_AXIS_FROM`, `IMU_AXIS_SIGN` in
  `Config.h`);
- calibrates gyro bias at start-up or on request: 2 s average with a stillness
  check, retried up to five times;
- runs a complementary filter for roll and pitch (default time constant 0.5 s,
  with the accelerometer correction skipped when |a| is more than 0.15 g away
  from 1 g) and integrates yaw;
- optionally tracks gyro bias while still (auto-bias), only after the motors
  have been stopped for 1 s, so a real slow turn is never absorbed;
- accumulates test-window statistics (noise, ranges, heading change, timing).

Core 0 reads results through a mutex-protected snapshot and sends requests
(calibrate, zero heading, filter time constant, auto-bias, start window)
through flags. These calls return in microseconds.

Robustness: stall recovery reads anyway if INT stays high with no interrupt
for 6 ms (also timed from the last read, so a disconnected INT gives about
166 reads/s rather than a flood); 50 consecutive read errors return the IMU to
`NO_SENSOR`, which retries initialisation every second; core 0 reports `STALE`
if no sample has arrived for 100 ms.

Measured on the bench (MPU6050 test project): 500 Hz with no missed samples
under heavy core 0 load, filter maths about 93 µs per sample, core 1 longest
loop pass about 210 µs, yaw drift about 0.5 °/min with calibration only and
none at rest with auto-bias.

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
 RP2040 PIO -> STEP/DIR/ENABLE
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

The instruction-level implementation is documented in
[RP2040 PIO STEP generation](pio-step-generation.md).

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

During motion, `Robot` polls one TMC2209 every 250 milliseconds and alternates
drivers when both are active. Status commands return the latest cached
snapshots immediately rather than adding UART transactions. These background
readings are diagnostic only and do not alter motion or driver settings.

### `MotionController`

`MotionController` coordinates the two motors. It:

- mixes linear and angular commands into left and right wheel speeds;
- converts physical geometry into steps;
- starts straight distance moves;
- starts in-place turns;
- stops both motors.

The current geometry is 61 mm effective wheel diameter, 188 mm effective wheel
track and 200 full steps per revolution. The safe boot default is 1/4
microstepping; an accepted idle-only configuration updates both the TMC setting
and the motion conversion together.

When continuous motion accelerates from rest, the motor ramp accumulates
fractional step travel and starts PIO output only when the first complete step
is due. This avoids an immediate 1 Hz pulse followed by a long low-speed pause.

### `Robot`

`Robot` is the application boundary. External controllers request actions only
through its public API:

```cpp
setVelocity(...)
startMove(...)
startTurn(...)
configure(...)
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

IMU commands (`IMU`, `IMU_CAL`, `IMU_ZERO_YAW`, `IMU_SET`, `IMU_WINDOW_START`,
`IMU_WINDOW`) never move the motors. They are accepted in every mode, including
`ESTOP`, and are not part of the motion-reply replay cache.

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
| Maximum PIO STEP frequency | 50,000 Hz |
| Boot acceleration | 150 mm/s² |
| Runtime acceleration range | 10–500 mm/s² |
| Maximum linear request | 1,000 mm/s |
| Maximum angular request | 360 deg/s |
| Maximum finite distance | 10,000 mm |
| Maximum finite angle | 3,600 deg |
| Velocity lease range | 100–2,000 ms |
| Boot TMC run current | 400 mA RMS |
| Runtime current range | 100–400 mA RMS |
| Boot microstepping | 1/4 |
| Runtime microsteps | 1, 2, 4, 8, 16, 32 or 64 |
| Chopper modes | StealthChop or SpreadCycle |

These are firmware safety limits, not proof that every permitted request is
safe for every payload and surface.

## Legacy serial commands

The older human-readable commands remain in `main.cpp` for bench diagnosis.
Lines beginning with `R2W` go to `RobotLink`; other lines go to the legacy
handler.

Normal Pi control should use `R2W/1`. Legacy motor-level commands bypass parts
of the application protocol and should not become a second production control
API.

## Runtime motor configuration

Motor configuration is accepted only while the robot is idle, the emergency
stop is clear and both motors are disabled. Current, microsteps and chopper mode
are applied to both TMC2209s, read back and rolled back on failure. Acceleration
is applied to both motion channels. Settings are intentionally held in RAM and
return to the safe boot defaults after reset.

The 400 mA ceiling matches the installed Casun 42SHD0001-24B motors. Replacing
the motors requires a reviewed motor profile before increasing this limit.

## Future real-time improvements

RP2040 PIO STEP generation is now in use. Possible future improvements include
DMA-fed PIO command buffers and DMA-assisted sensor sampling. DMA transfers data
between memory and peripherals; it does not directly “drive a pin” without a
suitable peripheral such as PIO.

The present software scheduler is adequate for the tested speeds. Optimisation
should follow measurement of timing limits, not precede it.
