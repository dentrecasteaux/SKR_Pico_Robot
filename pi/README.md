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

## Test scripts

### Telemetry runs (Phase 6)

`robotd` records firmware telemetry into one folder per run under `~/runs/`:
`data.csv` (engineering units, plus wheel speeds in mm/s) and `meta.json`
(name, notes, rate, the robot status at the start, firmware counters at the
end, and a summary with rows, missing records and the largest time gap).

```text
python3 robotdctl.py log-start --name my-test --div 2 --notes "what I did"
python3 robotdctl.py log-start --name quick --duration 10   # stops by itself
python3 robotdctl.py log-stop
python3 robotdctl.py log-status
python3 robotdctl.py bb-freeze      # ESTOP also freezes it
python3 robotdctl.py bb-dump        # saves ~/runs/<time>_bb_<reason>/
python3 robotdctl.py bb-arm         # clear and restart recording
python3 robotdctl.py runs
```

The web server lists runs at `/api/runs` and serves
`/runs/<run>/data.csv` and `/runs/<run>/meta.json` for download. From the
Mac, `rsync -av robot@r2w-pi.local:runs/ ~/robot-runs/` copies everything.

### `log_test.py`: Phase 6 pass test

**Robot on the stand.** It records a 30 s run while driving a ±200 mm/s,
0.25 Hz sine on both wheels, then checks there are no missing records or time
gaps and that the logged wheel speed follows the command. It then freezes and
dumps the black box, and checks it holds about 2 s of consecutive 500 Hz
records. It prints PASS/FAIL for each check.

```text
python3 log_test.py
python3 log_test.py --seconds 10 --div 1
```

### `limits_test.py`: speed and acceleration limits (Phase 7.1)

**Robot on the stand.** Interactive (use `ssh -t`). Part A ramps to 400–1000
mm/s; part B does ±300 mm/s reversals at 2–20 m/s². Both run in each chopper
mode. After every step you answer clean or stalled, because there are no
encoders. Each step is logged, and a summary JSON is written.

```text
ssh -t robot@r2w-pi.local 'cd r2w && python3 limits_test.py'
```

### `fall_test.py`: free fall, ω₀ and balance angle (Phase 7.3/7.4)

**Robot on the floor**, wheels locked with `HOLD`. Interactive. For each fall
you hold the robot near balance, keep still, then let go gently. Each fall is
logged at 500 Hz and fitted to θ = θ_eq + A·e^(ωt) + B·e^(−ωt), using the
gyro-integrated pitch. This gives ω₀ (so l_eff = g/ω₀²) and θ_eq, the balance
angle θ_trim. `--analyse RUN ...` re-analyses saved runs.

```text
ssh -t robot@r2w-pi.local 'cd r2w && python3 fall_test.py'
```

### Balancing (Phases 8–9)

```text
python3 robotdctl.py bal-status
python3 robotdctl.py bal-set KP=51.4 KD=4.37 KV=6.89 KX=6.48 TRIM=1.9   # the defaults
python3 robotdctl.py bal-arm [--dry]
python3 robotdctl.py bal-stop          # ends balancing (as does ESTOP); STOP only stops it moving
```

**Driving while balancing (Phase 10).** Once it's balancing, the web page's drive buttons (or `robotdctl velocity`) drive it: the web pad offers 50–300 mm/s and 15–90 °/s, and the firmware clamps the set points to ±300 mm/s and ±90 °/s. Letting go of a drive button sends speed 0, and STOP while balancing also just stops it moving: the robot keeps balancing. `balance_run.py`'s Ctrl-C (`BAL_STOP`) or ESTOP ends balancing. Whenever balancing ends (a trip or Ctrl-C), drive commands are refused (`BALANCE_TRIPPED`) until you press **Clear emergency stop** on the web page (shown while locked) or start another balance run. **Configuration check:** `python3 check_config.py` compares the live settings with the best known configuration and prints OK/DIFF (read-only). `robotdctl tuning` / `tuning-clear` show or delete the saved web tuning. **StealthChop PWM (11.0):** `robotdctl tmc-pwm` reads both drivers' PWM settings and learned values (`OFS_AUTO`, `GRAD_AUTO`); `robotdctl tmc-pwm OFS=36 GRAD=20 AUTOGRAD=0` sets them. **Identification kicks (9.4):** `balance_run.py --kicks 10` fires alternating ±2 m/s², 40 ms pulses (one every 3 s, after 8 s of balancing, only when calm) and prints l_eff at the end; `robotdctl bal-kick 2 --ms 40` fires one by hand. `balance_run.py` logs at 250 Hz by default (`--div 1` for 500 Hz, which drops about 40 % of lines).

**`compare_run.py` (Phase 11 comparison protocol).** One logged run per
controller set, always the same sequence:
- 10 s settle and 60 s quiet standing;
- 10 standard kicks (±2 m/s², 40 ms);
- 4 pushes (±3 m/s², 60 ms);
- speed steps: +V for 5 s, stop for 10 s, −V for 5 s, stop for 10 s (default 150 mm/s; 100 is a good common value, about 0.5 m out and back, so leave 1 m clear in front and behind).

It writes `compare.json` into the run: every setting (BAL_STATUS, IMU, STATUS, TMC PWM, saved tuning) and the time of each segment. `--set KEY=value …` and `--tau` change settings for that run only, and they are put back afterwards. `--reverse-first` runs the reverse step first; `--no-steps` skips the steps; `--push-u` and `--push-ms` change the pushes; `--notes` goes into `compare.json`. Metrics come from `analysis/compare.py RUN …` on the Mac (numpy, scipy, pandas).

```text
ssh -t robot@r2w-pi.local 'cd r2w && python3 compare_run.py --label everyday'
ssh -t robot@r2w-pi.local 'cd r2w && python3 compare_run.py --label pole-placement --set KP=36.7 KD=4.19 KV=4.81 KX=2.94'
```

**`demo_run.py` (film sequence).** A short logged demo with the live settings: settle and a countdown, then straight into forward 400 mm at 200 mm/s, turn 360°, reverse 400 mm back to the start, turn 360° the other way, 2 s still, then it keeps balancing until you press Enter. Options: `--distance`, `--speed`, `--turn` (180 for an about-face, so the return leg drives forwards), `--turn-rate`, `--pause`, `--start-hold`, `--end-hold`, `--no-wait`. Legs stop on the reference position (X_REF) and the return aims for the actual start; turns stop on the IMU yaw. It writes `demo.json` (event times, position and angle turned after each move) and prints where the robot ended up.

```text
ssh -t robot@r2w-pi.local 'cd r2w && python3 demo_run.py'
```

**`dryrun_check.py` (9.1, motors off).** Tilt the robot by hand forward and
back. It checks u = KP·e + KD·θ̇ holds with the right sign (lean forward →
drive forward). Do this before any powered attempt.

**`balance_run.py` (9.2 / 9.3).** A logged attempt (250 Hz by default). It arms, then
you lift the robot towards upright and it engages by itself. It ends on a
trip, Ctrl-C or `--seconds`. Afterwards it saves the black box and prints how
long it stood, the end reason and an l_eff estimate from the run's own data.
`--kp/--kd/--trim` set gains first, and `--scale 0.3` runs at 30% gains for
that run only. `--stand` is the wheels-in-the-air test: it zeroes the error at
the current angle, allows any tilt and caps the speed at 0.2 m/s.

```text
ssh -t robot@r2w-pi.local 'cd r2w && python3 dryrun_check.py'
ssh -t robot@r2w-pi.local 'cd r2w && python3 balance_run.py --stand --scale 0.3'
ssh -t robot@r2w-pi.local 'cd r2w && python3 balance_run.py'
```

### `vibe_test.py`: IMU vibration with the motors running

Commissioning plan Phase 5.4. **Robot on the stand, wheels clear.** For each
chopper mode it runs both wheels at a series of speeds, waits for them to
reach speed, then records a 4 s firmware IMU statistics window
(`IMU_WINDOW_START` / `IMU_WINDOW`, every 500 Hz sample). It prints gyro and
accelerometer standard deviations, the |a| range and the pitch-estimate
peak-to-peak for each point.

```text
python3 vibe_test.py                                # both modes, 0, 0.5, 1, 2, 3 rev/s
python3 vibe_test.py --modes SPREADCYCLE --speeds 0 1 2
```

Results go to `~/runs/<timestamp>_vibe/` (`data.csv`, `meta.json`). At the end
the script restores 1000 mA, 1/16, 150 mm/s² and StealthChop. It drives through
`robotd`, which caps a velocity hold at 5 s, so the motors stop by themselves
if the script dies.

Long fixed-speed runs (e.g. the heat run) can use a shell loop instead,
re-sending `robotdctl.py velocity … --hold 5000` every 3 s.

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
python3 robotdctl.py configure 1000 16 150 SPREADCYCLE
```

The service maintains a second, local control lease for velocity requests. A
caller must refresh its request before `hold_ms` expires. Until then, `robotd`
renews the shorter Pico lease. If the caller disappears, `robotd` sends `STOP`;
if the service or USB link also fails, the Pico lease expires independently.

Motor configuration is accepted only while the robot is idle and the motors
are disabled. Current is 100–1300 mA (Rtelligent 42A08 motors; the default is
1000 mA). Runtime settings reset to firmware defaults when the Pico reboots.

`robotdctl.py` is only a local service test client. The deployed web interface
uses the same Unix-socket API through `robot_web.py`.

## Deploying from the development computer

From the repository root on the Mac:

```text
tools/deploy-pi.sh
```

It copies `pi/` into `~/r2w` on `robot@r2w-pi.local` (or the address given as
an argument or in `R2W_PI`) with `rsync`, falling back to `scp`. Nothing on
the Pi is deleted. It then checks that the Python files compile, installs
changed user service files, restarts `robotd` and `robot-web`, and pings the
Pico. SSH access to the Pi is required.

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

The main page (`/`) is the **balance page**. The earlier three-wheeler console
(motor tuning, precise moves, IMU tests) is at `/bench`; each page links to the
other.

### Balance page

- **Status:** balance state and last end reason, pitch, error, speed,
  position, trim estimate, engaged time and max |err| / |u|, updated every
  0.3 s. A banner with a **Clear** button appears while driving is locked
  after balancing ended (or the e-stop is latched).
- **Balance run:** notes, number of kicks and "Record a log". **Start (arm)**
  does what `balance_run.py` does at its start (ends leftover balancing,
  resets the safety limits to their defaults, starts a 250 Hz log, arms).
  Kicks start 8 s after it engages, one every 3 s while it is still, from the
  page (keep the page open). **End balancing** stops it (the robot drops onto
  a training wheel), saves the log and black box, and writes
  `balance_summary.json` with the engaged time, end reason and, if there were
  kicks, l_eff and the ring frequency and damping. A trip ends the run the
  same way.
- **Drive while balancing:** hold-to-drive pad (speed 50–300 mm/s, turn
  15–90 °/s), STOP (comes to rest, keeps balancing) and EMERGENCY STOP (ends
  balancing).
- **Controller settings:** KP, KD, KV, KX, trim, drive lean, drive
  acceleration, drive feed-forward and trim rate. **Apply** sends changed
  fields live (also while balancing); KP/KD/KV/KX changes over 15 % in one
  step are refused unless you tick the box. **Save as start-up** stores the
  robot's current values on the Pi (`~/.config/r2w/balance_tuning.json`);
  robotd re-applies them whenever the SKR starts. **Load saved** and
  **Firmware defaults** fill the fields (then press Apply).
- **Recent runs:** summaries (from `balance_summary.json`), CSV download and a
  plot of pitch, u and speed with the kicks marked.

API added for the page: `GET /api/balance` (BAL_STATUS),
`POST /api/balance/start` and `/api/balance/stop`, and the robotd actions
`tuning`, `tuning_save`, `tuning_clear`.

Manual drive buttons must be held. The page renews a 1500 ms robotd hold
(Pico lease 1000 ms) every 200 ms, and retries once after a failed request.
Releasing the button, hiding the page, losing Wi-Fi, or closing the browser
stops those renewals. `robotd` then sends `STOP` (in balance mode: zero
drive, keep balancing), while the independent Pico lease remains the final
communications-loss safeguard. If a drive ends other than by releasing the
button, the page shows why (request failed, touch cancelled or capture lost,
page hidden or lost focus, or balancing stopped). Before 24 Nov the hold was
600 ms, which a single slow phone request could exceed.

### IMU panel

When the Pico firmware includes the IMU, the page shows a live IMU panel:

- roll, pitch, heading, rates, sample rate and temperature, with a 30-second
  rolling chart;
- **Calibrate gyro** (idle only; keep the robot still for about 2 s), **Zero
  heading**, the filter time constant and auto-bias;
- **tests**: still drift, straight move and turn check (which compares the
  commanded angle with the gyro-measured heading change);
- **recording while you drive**: from the IMU panel or the **Record IMU**
  button in the drive panel.

Each test uses a Pico-side IMU test window, so the statistics cover every
500 Hz sample, not just the values the page displays. Results are kept in the
browser and can be downloaded as CSV. `robotd` polls the IMU at about 10 Hz
and keeps 60 s of history for the chart. With firmware that has no IMU, the
panel reports it and `robotd` stops polling until the Pico reconnects.

The initial web server has no user login or TLS. Use it only on a trusted local
network and do not expose port 8080 to the internet.

The webpage shutdown button asks for confirmation, then requests a clean Pi
shutdown from the server. Allow the `robot` user to run only that command
without a password by creating `/etc/sudoers.d/r2w-shutdown` with `visudo`:

```text
robot ALL=(root) NOPASSWD: /sbin/shutdown -h now
```

Useful commands:

```text
systemctl --user status robot-web
systemctl --user restart robot-web
journalctl --user -u robot-web
```
