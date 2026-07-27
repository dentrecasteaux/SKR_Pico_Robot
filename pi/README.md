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
