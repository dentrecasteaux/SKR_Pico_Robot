# Raspberry Pi Executive Controller

## Role

The Pi provides a stable bridge between user-facing or future autonomous
software and the real-time Pico controller. In normal operation exactly one
program, `robotd.py`, owns the Pico serial port.

```text
robotdctl.py -----+
                  |
web browser -> robot_web.py -> Unix socket -> robotd.py -> Pico
                  |
future autonomy --+
```

## `robotctl.py`

This is the direct serial test client. It:

- locates the Pico by its stable `/dev/serial/by-id` name;
- creates sequence numbers;
- sends `R2W/1` commands;
- matches direct replies;
- records asynchronous `DONE` events;
- retries once with the same sequence if a direct reply is lost.

Use it for initial bring-up or diagnosis only when `robotd` is stopped. Two
programmes must not open the same serial device.

## `robotd.py`

`robotd` is the persistent executive link. It:

- owns and reconnects the USB serial connection;
- polls Pico status every second;
- exposes a local JSON request API through a Unix socket;
- records the latest status, event and error;
- renews Pico velocity leases;
- applies its own lease to external velocity control.
- forwards idle-only motor-configuration requests and refreshes status after a
  successful change.

The default Unix socket is:

```text
/run/user/<uid>/r2w-robotd.sock
```

The socket keeps the controller API local to the Pi. It is not a network
service.

### Two-level velocity lease

For manual driving, a caller asks `robotd` to hold a velocity for a limited
period. While that request remains valid, `robotd` renews the Pico lease.

```text
browser refreshes 600 ms control lease every 200 ms
        |
robotd renews the Pico lease
        |
Pico permits continuous motion
```

If browser updates stop, `robotd` sends `STOP`. If `robotd`, Linux or USB fails,
the independent Pico lease expires.

## `robotdctl.py`

This is a local command-line client for `robotd`. It is useful for:

- checking service connection state;
- testing velocity control;
- issuing finite moves and turns;
- stopping or clearing the software estop;
- viewing the latest asynchronous job event.

It does not open the Pico serial port.

## `robot_web.py`

The web server:

- serves the mobile user interface;
- accepts only a small allow-list of JSON actions;
- proxies those actions to the local `robotd` socket;
- has no direct knowledge of serial framing or motor pins.

The browser uses press-and-hold drive controls. Pointer, touch, page visibility
and window events are handled so that release or loss of page focus stops
renewing motion. The Pi and Pico leases remain the authoritative fallback.

The deployed motor-tuning panel controls current, microsteps, acceleration and
requested TMC mode. Apply is disabled unless status reports `IDLE` with no
emergency stop. The Pico remains authoritative and rejects unsafe state or
range combinations. Active StealthChop or SpreadCycle mode is read independently
from each TMC2209 and displayed in its driver card.

Manual-drive commands are sent immediately on pointer-down and renewed every
200 milliseconds. A previously observed twitch and startup pause at very low
acceleration was traced to the Pico's initial STEP frequency, not browser or
network delay. Fractional-step startup in `Motor` now avoids that artefact.

The safety panel also provides a confirmed Pi shutdown action. The browser
sends a confirmed request, but only `robot_web.py` executes the privileged
`shutdown -h now` command. The narrowly scoped sudo rule is documented in
`pi/README.md`.

## Services

Both programmes run as user-level `systemd` services:

```text
robotd.service
robot-web.service
```

User lingering allows them to start at boot without waiting for an SSH login.
The web service requests that `robotd` start with it.

## Network boundary

The server currently listens on port 8080 on all Pi interfaces. It has:

- no login;
- no TLS encryption;
- no protection against an untrusted local client.

It is suitable only for a trusted private network. Do not forward port 8080,
place the Pi in a router DMZ or otherwise expose it to the internet.

If remote access is needed later, design authentication, encryption, command
authorisation and network isolation before enabling it.
