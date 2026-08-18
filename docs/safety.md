# Safety Model

## Scope

The current system has useful software safety layers. It is not a
safety-certified controller and it does not yet have an independent physical
emergency-stop circuit.

When testing, lift the driven wheels clear of the floor unless actual movement
is required. Keep clear of wheels and mechanisms, limit speed, and keep battery
disconnection or another physical power-removal method immediately available.

## Defence in depth

| Layer | Main protection |
|---|---|
| Browser | Motion only while a control is held; stops refreshing on release or loss of focus |
| `robotd` | External control lease expires and sends `STOP` |
| Pico | Independent velocity lease expires and stops motors |
| Robot API | Range, state, estop and driver checks |
| TMC2209 | Hardware driver diagnostics and protection |

No single browser or Wi-Fi event is the sole stop mechanism.

## Continuous velocity

Continuous motion is inherently unbounded, so every `SET_VELOCITY` contains a
lease between 100 and 2,000 ms.

On Pico lease expiry:

1. both motors are stopped and disabled;
2. velocity setpoints are cleared;
3. mode returns to `IDLE`;
4. `FAULTS=LEASE_EXPIRED` is reported.

A later accepted motion command clears this diagnostic. It is a record of why
motion stopped, not a permanent latch.

## Finite jobs

An accepted `MOVE` or `TURN` is bounded and does not require communication
renewal. This permits a legitimate long move to finish if ordinary host traffic
is interrupted.

Finite motion remains open loop. A wheel obstruction or loss of traction does
not change the commanded step count. Therefore “bounded” does not mean
collision-safe or position-accurate.

## `STOP`

`STOP`:

- stops and disables the motors;
- cancels an active finite job with `RESULT=STOPPED`;
- is idempotent;
- is not latched.

A later valid motion command may move again.

## `ESTOP`

The protocol `ESTOP`:

- stops and disables the motors;
- cancels an active finite job with `RESULT=ESTOPPED`;
- latches `MODE=ESTOP` and `ESTOP=1`;
- rejects later motion with `ESTOP_LATCHED`.

`CLEAR_ESTOP` clears the software latch only if the robot is stationary and the
driver check allows motion. It does not itself enable or move the motors.

This is a software emergency stop. A crashed processor, failed output stage or
electrical fault may defeat it. A future physical emergency stop should remove
motor energy independently and should not be clearable by software while
asserted.

## Driver health

Before accepting protocol motion, `Robot` checks that both TMC2209 drivers
reply and that selected serious driver flags are clear:

- over-temperature shutdown (`OT`);
- over-temperature pre-warning (`OTPW`);
- short to ground on phase A (`S2GA`);
- short to ground on phase B (`S2GB`);
- low-side short to supply on phase A (`S2VSA`);
- low-side short to supply on phase B (`S2VSB`).

Status reports `NO_REPLY`, `OK_IDLE`, `OK_ACTIVE` or fault tokens. Driver
protection supplements but does not replace correct current setting, cooling,
wiring and mechanical design.

## Duplicate commands

Network or serial clients sometimes retry after a lost reply. Re-executing
`MOVE 500` would be unsafe. The Pico therefore remembers the latest accepted
motion command:

- same sequence and same command: replay the cached reply;
- same sequence and different command: `SEQUENCE_CONFLICT`.

The Pi client retries with the same sequence specifically to use this property.

## Known residual risks

- no physical emergency stop;
- no obstacle detection;
- no encoder verification of wheel movement;
- no low-battery cut-off implemented by this software;
- no web authentication;
- finite jobs continue after ordinary communication loss;
- legacy serial commands remain available for bench testing;
- motion is open loop and maximum reliable STEP rate has not been characterised
  under every payload, supply-voltage and surface condition.

These limitations should be reconsidered before increasing speed, payload,
voltage or operating near people.
