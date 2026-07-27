# R2W Pi-to-Pico Protocol

## Purpose

R2W is the command and status protocol between the Raspberry Pi Zero 2 W
executive controller and the SKR Pico motor controller.

The Pi sends motion requests and consumes status. The Pico owns real-time step
generation, acceleration limiting, motor enable state, finite-motion completion,
TMC2209 health, and immediate safety stopping. The Pi must never generate STEP
pulses.

## Transport

The initial transport is USB CDC serial. A future GPIO UART transport may carry
the same protocol unchanged.

- Encoding: printable ASCII
- Framing: one message per line, terminated by `\n`
- Input may use `\r\n`; the receiver ignores the preceding `\r`
- Field separator: one or more ASCII spaces
- Keys and command names are upper case and case-sensitive
- Receivers must process input without blocking motor updates
- Maximum line length, baud-rate settings for UART, and numeric limits are
  implementation constants and must be reported as errors rather than silently
  truncated

Only one host may own the command transport at a time.

## Protocol versioning

Every message begins with:

```text
R2W/1
```

`1` is the major protocol version. A receiver must reject an unsupported major
version with `BAD_VERSION`. New optional fields and new commands may be added
within version 1. Receivers must ignore unknown fields in replies, but commands
with unknown or duplicate parameters are rejected.

## Command syntax

```text
R2W/1 CMD <sequence> <command> [KEY=VALUE ...]
```

- `<sequence>` is an unsigned decimal integer chosen by the Pi.
- The Pi uses a new sequence number for every command, including lease renewals.
- The Pico echoes the sequence number in the direct reply.
- Parameters are unordered `KEY=VALUE` fields.
- Numbers use decimal notation and a full stop as the decimal separator.
- Units are part of this specification, not the field value.

The Pico sends exactly one direct reply (`ACK`, `ERR`, `PONG`, or `STAT`) for
each syntactically complete command. Commands may arrive while motion is active,
but their mode-specific rules still apply.

## Reply syntax

```text
R2W/1 ACK <sequence> OK [KEY=VALUE ...]
R2W/1 ERR <sequence> CODE=<code> [FIELD=<key>] [MSG=<token>]
R2W/1 PONG <sequence> UPTIME_MS=<milliseconds>
R2W/1 STAT <sequence> <status-fields>
R2W/1 DONE JOB=<job-id> ORIGIN_SEQ=<sequence> RESULT=<result>
```

`ACK` confirms acceptance, not completion. `ERR` means the command was not
accepted. `PONG` and `STAT` are successful direct replies and therefore do not
also receive an `ACK`.

`DONE` is an asynchronous event for a previously accepted finite job. It may be
sent at any time after that job's `ACK`. The Pico should retain the most recent
completion in status so a disconnected Pi can recover it after reconnecting.

## Command set

### `PING`

Checks protocol and transport availability without changing robot state.

```text
R2W/1 CMD 1 PING
R2W/1 PONG 1 UPTIME_MS=542331
```

### `STATUS`

Requests a current status snapshot.

```text
R2W/1 CMD 2 STATUS
R2W/1 STAT 2 MODE=IDLE ESTOP=0 FAULTS=NONE JOB=0 V_SET=0 W_SET=0 X_DRIVER=OK_IDLE Y_DRIVER=OK_IDLE UPTIME_MS=542410 RX_AGE_MS=12
```

### `SET_VELOCITY`

Starts or updates open-ended differential-drive motion.

```text
R2W/1 CMD 3 SET_VELOCITY V=50 W=0 LEASE=500
```

- `V`: linear velocity in millimetres per second
- `W`: angular velocity in degrees per second; positive turns left
- `LEASE`: required validity period in milliseconds

The command cancels any active finite job. Values outside configured speed or
lease limits are rejected. `V=0 W=0` is valid, but `STOP` is preferred when an
immediate, unambiguous stop is required.

### `MOVE`

Starts a finite straight-line move.

```text
R2W/1 CMD 4 MOVE DIST=500
R2W/1 ACK 4 OK JOB=17
```

`DIST` is in millimetres. Positive is forwards and negative is backwards. Zero
and values outside configured limits are rejected.

### `TURN`

Starts a finite in-place turn.

```text
R2W/1 CMD 5 TURN ANGLE=720
R2W/1 ACK 5 OK JOB=18
```

`ANGLE` is in degrees. Positive turns left and negative turns right. Zero and
values outside configured limits are rejected.

### `STOP`

Cancels streaming motion or an active finite job, commands an immediate stop,
and disables both motors. It is not latched; a later valid motion command may
re-enable the motors.

```text
R2W/1 CMD 6 STOP
R2W/1 ACK 6 OK
```

`STOP` is idempotent and succeeds when already idle. If it cancels a finite job,
the Pico also emits:

```text
R2W/1 DONE JOB=18 ORIGIN_SEQ=5 RESULT=STOPPED
```

### `ESTOP`

Immediately stops motion, disables both motors, cancels any active job, and
latches the emergency-stop state.

```text
R2W/1 CMD 7 ESTOP
R2W/1 ACK 7 OK
```

`ESTOP` is idempotent and must be accepted in every software motion state.
While latched, all motion commands are rejected with `ESTOP_LATCHED`; `PING`,
`STATUS`, `STOP`, and `ESTOP` remain available.

### `CLEAR_ESTOP` (optional)

An implementation may provide explicit software recovery:

```text
R2W/1 CMD 8 CLEAR_ESTOP
R2W/1 ACK 8 OK
```

It may succeed only when the robot is stationary and no blocking hardware or
driver fault remains. It clears the latch but does not start motion or enable
the motors. If not implemented, the Pico returns `UNSUPPORTED_COMMAND` and
recovery requires reset. Physical emergency-stop hardware, when added, must not
be clearable by this command while asserted.

## Lease rules

Leases apply only to `SET_VELOCITY`.

- The lease starts when the Pico accepts the command.
- Each accepted `SET_VELOCITY` replaces the setpoint and renews the lease.
- `PING`, `STATUS`, malformed input, and unrelated commands do not renew it.
- The Pi should renew well before expiry; a starting policy is every 100 ms for
  a 500 ms lease.
- On expiry, the Pico cancels velocity mode, stops, disables both motors, sets
  `LEASE_EXPIRED`, and enters `IDLE`.
- Lease expiry is handled from the Pico's monotonic clock and must not depend on
  Pi time.

The supported lease range is a firmware safety setting. A suggested initial
range is 100–2000 ms.

## Finite-job rules

`MOVE` and `TURN` are bounded jobs owned and completed by the Pico.

- Only one finite job may be active.
- An accepted job receives a non-zero job ID.
- A finite job does not require heartbeats or lease renewal.
- Loss of Pi communication does not interrupt an accepted finite job.
- Normal completion emits `RESULT=OK` and returns the robot to `IDLE`.
- `STOP`, `ESTOP`, a new accepted `SET_VELOCITY`, or a safety fault cancels the
  job and emits a corresponding `DONE` result.
- A new `MOVE` or `TURN` while a job is active is rejected with `BUSY`.
- Job IDs are unsigned and may wrap; zero means no active job.

Example completion:

```text
R2W/1 DONE JOB=17 ORIGIN_SEQ=4 RESULT=OK
```

## Acknowledgements and duplicate commands

Sequence numbers correlate commands and direct replies. They are not job IDs.

The Pico should cache a small number of recent sequence numbers and their direct
replies. If the Pi retries an identical command with the same sequence number,
the Pico resends the cached reply without executing it again. Reuse of a cached
sequence number with different command text is rejected with
`SEQUENCE_CONFLICT`.

This is especially important for `MOVE` and `TURN`, where a lost `ACK` must not
start a second job.

The current Pico implementation caches the single most recent motion-command
reply. That is sufficient for the present client, which retries immediately
with the same sequence. A future pipelined client would require a larger cache.

## Error handling

Errors use stable machine-readable codes. `MSG`, when present, is a short
diagnostic token and must not be parsed as control data.

Initial error codes:

| Code | Meaning |
|---|---|
| `BAD_FRAME` | Invalid message structure |
| `BAD_VERSION` | Unsupported major version |
| `BAD_SEQUENCE` | Missing or invalid sequence number |
| `UNKNOWN_COMMAND` | Command name is not recognised |
| `UNSUPPORTED_COMMAND` | Recognised optional command is not implemented |
| `MISSING_FIELD` | Required parameter is absent |
| `UNKNOWN_FIELD` | Parameter is not valid for this command |
| `DUPLICATE_FIELD` | Parameter occurs more than once |
| `INVALID_VALUE` | Parameter is not a valid number or allowed enum |
| `OUT_OF_RANGE` | Parameter exceeds configured safety limits |
| `BUSY` | A conflicting finite job is active |
| `ESTOP_LATCHED` | Motion is forbidden until emergency stop is cleared |
| `DRIVER_FAULT` | Driver state prevents motion |
| `SEQUENCE_CONFLICT` | A recent sequence number was reused differently |
| `LINE_TOO_LONG` | Input exceeded the parser buffer |

Example:

```text
R2W/1 ERR 9 CODE=OUT_OF_RANGE FIELD=V MSG=ABOVE_LIMIT
```

Malformed input without a recoverable sequence number uses sequence `0`.
The Pico must never start or alter motion from a command that produces `ERR`.

## Status fields

Version 1 status contains:

| Field | Values and units |
|---|---|
| `MODE` | `IDLE`, `VELOCITY`, `MOVE`, `TURN`, or `ESTOP` |
| `ESTOP` | `0` or `1` |
| `FAULTS` | `NONE` or comma-separated fault tokens |
| `JOB` | Active job ID, or `0` |
| `V_SET` | Requested linear velocity in mm/s |
| `W_SET` | Requested angular velocity in deg/s |
| `X_DRIVER` | `OK_IDLE`, `OK_ACTIVE`, `NO_REPLY`, `OT`, `OTPW`, `S2GA`, `S2GB`, or comma-separated faults |
| `Y_DRIVER` | Same values as `X_DRIVER` |
| `UPTIME_MS` | Pico uptime from a monotonic clock |
| `RX_AGE_MS` | Time since the last valid command |

When available, status should also include:

| Field | Values and units |
|---|---|
| `LEASE_LEFT_MS` | Remaining velocity lease, otherwise `0` |
| `LAST_JOB` | Most recently completed job ID, otherwise `0` |
| `LAST_RESULT` | `NONE`, `OK`, `STOPPED`, `ESTOPPED`, `FAULT`, or `REPLACED` |

Future battery, encoder, IMU, and odometry fields may be added without changing
the major protocol version.

## Safety behaviour

Safety decisions are enforced on the Pico and do not depend on the Pi remaining
alive.

| State or event | Required Pico behaviour |
|---|---|
| Velocity lease expires | Stop, disable motors, enter `IDLE`, record `LEASE_EXPIRED` |
| Pi disconnects during finite job | Complete the bounded job, then stop |
| `STOP` received | Stop immediately, disable motors, do not latch |
| `ESTOP` received | Stop immediately, disable motors, latch `ESTOP` |
| Driver or hardware safety fault | Stop or estop as appropriate and report the fault |
| Invalid or partial command | Reject it and preserve the current safe state |
| Pico reset | Start with motors disabled and no active motion |

Protocol heartbeats diagnose link health; they never replace the velocity lease.
No Pi command may bypass the Pico's configured speed, acceleration, driver, or
emergency-stop limits.

## Example session

```text
# Link check and status
R2W/1 CMD 100 PING
R2W/1 PONG 100 UPTIME_MS=1203
R2W/1 CMD 101 STATUS
R2W/1 STAT 101 MODE=IDLE ESTOP=0 FAULTS=NONE JOB=0 V_SET=0 W_SET=0 X_DRIVER=OK_IDLE Y_DRIVER=OK_IDLE UPTIME_MS=1210 RX_AGE_MS=0 LEASE_LEFT_MS=0 LAST_JOB=0 LAST_RESULT=NONE

# Stream forwards; each command renews the lease
R2W/1 CMD 102 SET_VELOCITY V=50 W=0 LEASE=500
R2W/1 ACK 102 OK
R2W/1 CMD 103 SET_VELOCITY V=50 W=0 LEASE=500
R2W/1 ACK 103 OK
R2W/1 CMD 104 STOP
R2W/1 ACK 104 OK

# Run a bounded move
R2W/1 CMD 105 MOVE DIST=500
R2W/1 ACK 105 OK JOB=23
R2W/1 DONE JOB=23 ORIGIN_SEQ=105 RESULT=OK

# Emergency stop and optional recovery
R2W/1 CMD 106 ESTOP
R2W/1 ACK 106 OK
R2W/1 CMD 107 TURN ANGLE=90
R2W/1 ERR 107 CODE=ESTOP_LATCHED
R2W/1 CMD 108 CLEAR_ESTOP
R2W/1 ACK 108 OK
```
