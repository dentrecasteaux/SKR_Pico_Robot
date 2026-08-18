# Operating the Robot

## Normal start

1. Place the robot safely, preferably with its wheels raised for the first test.
2. Power the motor controller and the Pi using the approved present wiring.
3. Confirm the Pico is connected to the Pi’s USB data port.
4. Wait for the Pi to boot.
5. Open `http://r2w-pi.local:8080` from a device on the same trusted Wi-Fi.
6. Confirm the page shows the controller as connected and the robot as idle.
7. Test `STOP` before commanding motion.

Manual direction controls are press-and-hold. Releasing, leaving the page or
losing browser focus stops their renewals.

## Command-line operation through `robotd`

On the Pi:

```text
cd ~/r2w
python3 robotdctl.py service-status
python3 robotdctl.py ping
python3 robotdctl.py status
python3 robotdctl.py velocity 20 0 --hold 3000
python3 robotdctl.py move 100
python3 robotdctl.py turn 90
python3 robotdctl.py stop
python3 robotdctl.py estop
python3 robotdctl.py clear-estop
python3 robotdctl.py configure 400 4 150 STEALTHCHOP
```

`velocity 20 0 --hold 3000` asks for approximately 20 mm/s forwards for three
seconds. `move 100` starts a 100 mm finite job. These values are commands, not
encoder-confirmed measurements.

Motor configuration is accepted only while idle, with the emergency stop clear
and both motors disabled. The arguments are run current in mA, microsteps,
acceleration in mm/s² and either `STEALTHCHOP` or `SPREADCYCLE`. Settings apply
to both drivers and reset to 400 mA, 1/4 microsteps, 150 mm/s² and StealthChop
when the Pico reboots.

## Checking service state

```text
systemctl --user is-active robotd
systemctl --user is-active robot-web
systemctl --user status robotd
systemctl --user status robot-web
```

View recent logs:

```text
journalctl --user -u robotd -n 100
journalctl --user -u robot-web -n 100
```

Restart:

```text
systemctl --user restart robotd
systemctl --user restart robot-web
```

## Direct Pico diagnosis

Stop the persistent owner first:

```text
systemctl --user stop robotd
```

Then use the direct client:

```text
cd ~/r2w
python3 robotctl.py ping
python3 robotctl.py status
python3 robotctl.py velocity 20 0 --lease 500 --duration 3
python3 robotctl.py move 100 --wait
python3 robotctl.py stop
```

Restart normal control afterwards:

```text
systemctl --user start robotd
```

Do not run `robotctl.py` while `robotd` is active.

## Reading status

Example:

```text
MODE=IDLE ESTOP=0 FAULTS=NONE JOB=0
V_SET=0.00 W_SET=0.00 LEASE_LEFT_MS=0
LAST_JOB=2 LAST_RESULT=OK
X_DRIVER=OK_IDLE Y_DRIVER=OK_IDLE
```

| Field | Interpretation |
|---|---|
| `MODE` | Current application state |
| `ESTOP` | `1` when software estop is latched |
| `FAULTS` | Current/remembered fault tokens |
| `JOB` | Active finite job, or zero |
| `V_SET`, `W_SET` | Requested velocity setpoints |
| `LEASE_LEFT_MS` | Remaining Pico velocity lease |
| `LAST_JOB` | Most recently completed/cancelled job |
| `LAST_RESULT` | Outcome of that job |
| `X_DRIVER`, `Y_DRIVER` | Left and right TMC health/activity |

Driver details also expose actual current scale, observed chopper mode,
commanded STEP frequency, telemetry source and snapshot age.
During motion, status uses cached snapshots refreshed by alternating background
polls so a status request does not pause STEP generation.

## Normal shutdown

1. Issue `STOP`.
2. Confirm `MODE=IDLE`, `JOB=0` and both drivers report `OK_IDLE`.
3. Use **Shut down Raspberry Pi** in the webpage safety panel and confirm the
   prompt. Alternatively, from a Pi shell:

```text
sudo poweroff
```

4. Wait until the Pi activity light goes out before removing system power
   according to the robot’s power arrangement.
