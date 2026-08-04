# Raspberry Pi command-line client

`robotctl.py` is the first Raspberry Pi executive-controller tool. It opens the
SKR Pico's stable USB serial identity, sends one `R2W/1` command at a time, and
matches direct replies by sequence number.

If a direct reply is lost, the client retries once with the same sequence
number. Pico-side replay protection returns the cached acknowledgement without
executing the command twice.

## Pi dependency

Install pyserial from Raspberry Pi OS:

```text
sudo apt install python3-serial
```

## Usage

Run from the repository's `pi` directory:

```text
python3 robotctl.py ping
python3 robotctl.py status
python3 robotctl.py velocity 20 0
python3 robotctl.py velocity 20 0 --lease 500 --duration 5
python3 robotctl.py move 100 --wait
python3 robotctl.py turn 90 --wait
python3 robotctl.py stop
python3 robotctl.py estop
python3 robotctl.py clear-estop
```

A one-shot `velocity` command is allowed to expire, which tests Pico-side
communications-loss safety. With `--duration`, the client renews the lease at
one-third of its duration and sends `STOP` when finished. Pressing `Ctrl+C`
during a timed velocity command also sends `STOP`.

Finite commands return after `ACK` unless `--wait` is supplied. With `--wait`,
the client remains connected until it receives the matching `DONE` event.

## Device selection

By default, the client finds one matching Arduino Raspberry Pi Pico under:

```text
/dev/serial/by-id/
```

Override this with either:

```text
python3 robotctl.py --port /dev/ttyACM0 status
```

or the `R2W_PORT` environment variable.

## Persistent control service

`robotd.py` is the single long-running owner of the Pico serial port. Other Pi
programs communicate with it through a local Unix socket; they must not open the
Pico directly while the service is running.

Start it manually for initial testing:

```text
cd ~/r2w
python3 robotd.py
```

From a second SSH session:

```text
cd ~/r2w
python3 robotdctl.py ping
python3 robotdctl.py status
python3 robotdctl.py service-status
python3 robotdctl.py velocity 20 0 --hold 3000
python3 robotdctl.py move 100
python3 robotdctl.py turn 90
python3 robotdctl.py stop
python3 robotdctl.py estop
python3 robotdctl.py clear-estop
python3 robotdctl.py configure 400 4 150 SPREADCYCLE
```

The service maintains a second, local control lease for velocity requests. A
caller must refresh its request before `hold_ms` expires. Until then, `robotd`
renews the shorter Pico lease. If the caller disappears, `robotd` sends `STOP`;
if the service or USB link also fails, the Pico lease expires independently.

Motor configuration is accepted only while the robot is idle and the motors
are disabled. The installed Casun 42SHD0001-24B motor profile is capped at
400 mA. Runtime settings reset to firmware defaults when the Pico reboots.

`robotdctl.py` is only a local service test client. The future web interface
will use the same Unix-socket API.

## Start robotd automatically

Install the supplied user service:

```text
mkdir -p ~/.config/systemd/user
cp ~/r2w/systemd/robotd.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now robotd
```

Allow the user's service manager to start at boot before an SSH login:

```text
sudo loginctl enable-linger robot
```

Useful service commands:

```text
systemctl --user status robotd
systemctl --user restart robotd
systemctl --user stop robotd
journalctl --user -u robotd
```

Do not start a second manual copy of `robotd.py` or use `robotctl.py` while the
system service is active.

## Web interface

`robot_web.py` serves a phone-friendly control panel and proxies its API to the
local `robotd` Unix socket. The browser never opens the Pico serial port.

Install and start the supplied user service:

```text
cp ~/r2w/systemd/robot-web.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now robot-web
```

Open this address from a phone or computer on the same Wi-Fi network:

```text
http://r2w-pi.local:8080
```

Manual drive buttons must be held. The page refreshes a 600 ms local control
lease every 200 ms; releasing the button, hiding the page, losing Wi-Fi, or
closing the browser stops those refreshes. `robotd` then sends `STOP`, while the
independent Pico lease remains the final communications-loss safeguard.

The initial web server has no user login or TLS. Use it only on a trusted local
network and do not expose port 8080 to the internet.

Useful commands:

```text
systemctl --user status robot-web
systemctl --user restart robot-web
journalctl --user -u robot-web
```
