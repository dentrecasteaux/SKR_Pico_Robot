# Testing and Troubleshooting

## Safe incremental test

Perform initial tests with the driven wheels raised.

### PIO STEP engine bring-up

After changing the STEP engine, first use the direct USB serial commands:

1. Run `left test` and `right test`; each motor must produce exactly ten steps.
2. Run `left forward`, `left reverse`, `right forward`, and `right reverse`.
3. Run `move 500` and `turn 90`; confirm both finite moves stop normally.
4. Start a low continuous speed, issue `drivers`, and confirm STEP remains
   non-zero while driver polling remains zero during motion.
5. Issue `off` during continuous motion and confirm both motors stop
   immediately.

Do not place the wheels on the ground until direction, stopping, and finite
step counts pass with the chassis safely supported.

### 1. USB enumeration

```text
lsusb
ls -l /dev/ttyACM*
ls -l /dev/serial/by-id/
```

Expected: Arduino RaspberryPi Pico, USB ID `2e8a:00c0`, with `ttyACM0` and a
stable by-id link.

### 2. Protocol

With `robotd` running:

```text
python3 robotdctl.py ping
python3 robotdctl.py status
```

Expected: `PONG`, then `STAT` with `connected: true`,
`MODE=IDLE`, `ESTOP=0` and both drivers `OK_IDLE`.

### 3. Leased velocity

```text
python3 robotdctl.py velocity 20 0 --hold 3000
```

Expected: both wheels run forwards for about three seconds and stop. Status
returns to idle.

### 4. Finite job

```text
python3 robotdctl.py move 100
```

Expected: acknowledgement contains a non-zero job ID. After completion,
`service-status` contains a matching `DONE ... RESULT=OK` event and status shows
the job as the latest completed job.

### 5. Estop

```text
python3 robotdctl.py estop
python3 robotdctl.py status
```

Expected: `MODE=ESTOP ESTOP=1`. Motion commands are rejected. Then:

```text
python3 robotdctl.py clear-estop
python3 robotdctl.py status
```

Expected: idle with `ESTOP=0`.

## Common problems

### Pico appears as `RP2 Boot`

Symptom:

```text
ID 2e8a:0003 Raspberry Pi RP2 Boot
```

The Pico is in its UF2 bootloader, so no `ttyACM` serial device exists. Install
the firmware, then reset or power-cycle without holding BOOTSEL. Restarting the
Pi is normally unnecessary.

### No `/dev/ttyACM0`

Check:

- the cable is connected to the Pi Zero’s USB data/OTG port;
- the cable carries D+ and D−, not power only;
- the Pico is not in RP2 Boot;
- the latest kernel messages after reconnecting;
- firmware is present and running.

### Permission denied on serial

The Pi user should belong to `dialout`:

```text
groups
```

After adding group membership, log out and back in or reboot so the session
receives the new group.

### Port is busy or replies are confused

Only one serial owner is allowed. In normal use this is `robotd`. Close
miniterm and do not run `robotctl.py` at the same time.

```text
systemctl --user status robotd
```

Stop it temporarily only for direct serial diagnosis.

### Command times out after some acknowledgements

Possible causes:

- another programme is reading the same serial stream;
- output from a previous direct terminal session remains in the input buffer;
- the Pico disconnected or rebooted;
- the client and service were accidentally used together.

Return to one serial owner, issue `STOP`, obtain a fresh `STATUS`, and repeat one
small test.

### `SEQUENCE_CONFLICT`

The same recent sequence number was reused with different command text. This is
intentional replay protection. Use a new sequence number. An identical retry
with the same sequence should instead replay the original acknowledgement.

### `LEASE_EXPIRED`

This normally means a velocity request was not renewed. It demonstrates that
communications-loss protection worked. A new valid motion command clears the
flag.

### `BUSY`

A finite `MOVE` or `TURN` is already active. Wait for its `DONE` event, inspect
status, or issue `STOP` before starting another finite job.

### `ESTOP_LATCHED`

Motion is deliberately locked. Inspect the robot and driver status before
using `CLEAR_ESTOP`.

### Driver `NO_REPLY` or fault token

Do not keep retrying motion. Check power, motor wiring, UART wiring/address,
driver temperature and configuration. Remove power before changing motor
wiring.

### Web page unavailable

Check:

```text
systemctl --user status robot-web
systemctl --user status robotd
hostname
```

Try `http://r2w-pi.local:8080`. If `.local` discovery is unavailable on the
client, use the Pi’s trusted-LAN IP address. Do not expose the port externally.

## Calibration

Current configured effective geometry is:

```text
wheel diameter = 61 mm
wheel track    = 188 mm
```

For a repeatable calibration:

1. mark the starting pose precisely;
2. use a long, straight, non-slip test surface;
3. repeat each move in both directions;
4. average multiple runs;
5. change one parameter at a time;
6. commit the result with the measurements.

Distance primarily adjusts effective wheel diameter. In-place angle primarily
adjusts effective wheel track after distance is satisfactory.

The older `CALIBRATION.md` records an intermediate 194 mm track estimate.
The firmware’s current tested project baseline is 188 mm. Keep historical data,
but use `Config.h` as the authoritative configured value.
