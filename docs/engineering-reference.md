# R2W Engineering Reference

**Status: 24 November 2026, branch `imu-integration`.**

This is the single technical reference for the R2W robot in its current form: a two-wheel self-balancing robot built from the original three-wheeled R2W electronics. It covers:
- **Part A:** what the robot is and how it works.
- **Part B:** the parameters and the measured plant.
- **Part C:** the results and the open items.
- **Part D:** a step-by-step guide for bringing a similar balancing device online, built from what was learnt here.

Where a topic has its own detailed document, this reference summarises it and links to it. If the two disagree, the code is authoritative, then this document, then the dated logs ([balancing-results.md](balancing-results.md), [phase11-model.md](phase11-model.md)), which record what was true *on the day*.

---

## Contents

**Part A: the system**
1. [Summary and key numbers](#1-summary-and-key-numbers)
2. [Architecture](#2-architecture)
3. [Hardware](#3-hardware)
4. [Firmware](#4-firmware)
5. [Raspberry Pi software](#5-raspberry-pi-software)

**Part B: parameters and plant**

6. [Parameter reference](#6-parameter-reference)
7. [Plant model and identification](#7-plant-model-and-identification)

**Part C: results and status**

8. [Controller design history and results](#8-controller-design-history-and-results)
9. [Known issues and open items](#9-known-issues-and-open-items)

**Part D: bringing a similar device online**

10. [Bring-up guide](#10-bring-up-guide)
11. [Standard tests and how to analyse them](#11-standard-tests-and-how-to-analyse-them)
12. [Troubleshooting map](#12-troubleshooting-map)

**Appendices**

13. [Glossary](#13-glossary)
14. [Timeline](#14-timeline)
15. [Document map](#15-document-map)

---

# Part A: the system

## 1. Summary and key numbers

R2W is a two-wheel inverted-pendulum robot.
- **Motor controller:** a BigTreeTech SKR Pico (RP2040) runs the motors, the IMU and the balance controller in real time.
- **Executive:** a Raspberry Pi Zero 2 W provides Wi-Fi, a phone web page, logging and tuning persistence.
- **Drive:** two open-loop NEMA 17 steppers drive the wheels directly. There are no wheel encoders: the controller's wheel speed and position are the *commanded* ones.

| Quantity | Value | Source |
|---|---|---|
| Mass | 1880 g | weighed |
| Wheel diameter | 62.33 mm | 10-revolution roll; `move 500` measured 500 mm |
| Track | 223 mm | tyre centrelines |
| CoM height above the axle | about 30 mm | balance on a rod |
| Effective pendulum length l_eff | about 94–95 mm | swing test (95 mm); fit of the model with the estimator (94 mm) |
| IMU height above the axle | 98 mm | measured 24 Nov |
| Loop delay (command to response) | about 10 ms | kick-test fits (7.8–13 ms by method) |
| Balance point (IMU pitch) | +1.9° | `TRIM`; walked automatically while running |
| Training-wheel stops (IMU pitch) | +12.7° forward, −16.7° back | measured; tilt trips are 2° inside: +10.7° / −14.7° |
| Control rate | 500 Hz | IMU data-ready interrupt |
| Telemetry | 500 Hz into rings; 250 Hz streamed to the Pi | |
| Everyday gains | KP 51.4, KD 4.37, KV 6.89, KX 6.48 | firmware default since 24 Nov |
| Kick wobble with everyday gains | about 2.3–2.5 Hz, ζ 0.27–0.43 | it varies with the state of the joints |
| Clean drive speed | up to 0.38 m/s measured | firmware drive limit 0.3 m/s set point |

---

## 2. Architecture

### 2.1 Blocks

```text
 Phone / computer ──HTTP (trusted Wi-Fi)──► robot_web.py ──Unix socket──► robotd.py ──USB CDC, R2W/1──► SKR Pico (RP2040)
                                              (Pi Zero 2 W)                (Pi)                          │
                                                                                                         ├─ core 0: RobotLink, Robot, MotionController,
                                                                                                         │          Motor/Stepper, TMC2209 UART, Telemetry
                                                                                                         └─ core 1: Imu (MPU6050 @500 Hz) → Balance controller
                                                                                                                    → steppers (while balancing)
```

### 2.2 Responsibilities

| Layer | Owns | Must never |
|---|---|---|
| Browser | Press-and-hold driving, tuning UI, run start/stop, plots | Be the only stop mechanism |
| `robot_web.py` | Static pages, a small allow-list of actions, balance-run sessions (logging, kick scheduling, summaries) | Open the serial port |
| `robotd.py` | The only owner of the serial port; status polling; the second-level drive lease (`hold_ms`); telemetry capture to `~/runs`; re-applying saved tuning when the SKR restarts | Generate pulses or run control |
| SKR core 0 | Protocol, Robot API and modes, the normal motion chain, TMC configuration and health, the velocity lease, telemetry rings and black box | Touch I²C or the IMU while core 1 runs |
| SKR core 1 | IMU sampling, the attitude filter, the balance controller, the steppers while balancing, every balance safety trip | Print, use USB or the TMC UART |

### 2.3 Timing

| Loop | Rate | Notes |
|---|---|---|
| IMU sample, attitude update, balance step | 500 Hz | Each sample is processed straight away on core 1, with no cross-core hop before the control output |
| Step generation | PIO hardware | Up to 50 kHz; one state machine per wheel |
| Telemetry rings | 500 Hz | 1 s live ring, 2 s black box (1024 samples) |
| `LOG` stream | 250 Hz (`DIV=2`) | `DIV=1` (500 Hz) drops lines over USB |
| `robotd` status poll | 1 Hz (IMU at about 10 Hz) | |
| Web drive renewal | every 200 ms | Pico lease 1000 ms; robotd hold 1500 ms |

### 2.4 Modes

`IDLE`, `VELOCITY`, `MOVE`, `TURN`, `HOLD`, `BALANCE`, `ESTOP` (`STATUS MODE=`).

`BALANCE` covers every balance state (`ARMING`, `RUNNING`, `DRY`, `TRIPPED`). In it, core 1 owns the steppers and every other motion command except `SET_VELOCITY` is refused with `BUSY`.

---

## 3. Hardware

Details: [hardware-reference.md](hardware-reference.md), [skr-pico-circuit-reference.md](skr-pico-circuit-reference.md).

| Item | Configuration |
|---|---|
| Chassis | Two aluminium plates (168 × 76 mm, 61 mm apart), motors under the bottom plate, battery on the top plate, Pi and buck converter on the battery. 3D-printed "training wheels" limit tilt. |
| Motors | 2 × Rtelligent 42A08 (1.8°, 1.8 A rated, 2.0 Ω, 4.8 mH, 0.68–0.71 N·m), direct drive. Coils Red–Blue and Green–Black. |
| Drivers | On-board TMC2209, UART-configured. **StealthChop, 1.0 A RMS, 1/16 microstepping** (interpolated to 1/256). Runtime ceiling 1.3 A. |
| IMU | MPU6050 (GY-521) on I²C0 (GPIO 0/1, 400 kHz), data-ready interrupt on GPIO 16. ±4 g, ±500 °/s, on-chip DLPF about 94 Hz, 500 Hz output. **Wedged between the top plate and the battery, 98 mm above the axle.** |
| Power | 4S LiPo (16.8 V full, about 14.8 V nominal). Not monitored by the software. |
| Pi link | USB CDC (Pi data/OTG port), stable `/dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_*` |

**Pin map (SKR):**

| Signal | Left (X) | Right (Y) |
|---|---|---|
| STEP / DIR / EN | GPIO 11 / 10 / 12 | GPIO 6 / 5 / 7 |
| TMC address | 0 | 2 |

TMC UART is GPIO 8/9. Left is not inverted, right is inverted (balancing chassis).

**Thermal:** in a 10 min heat run at 1.0 A, the motors reached 33 °C and the driver heatsinks 50 °C. The drivers, not the motors, limit the current.

---

## 4. Firmware

Source: `src/`, `include/`. Build with PlatformIO (arduino-pico core). The architecture is described in more depth in [firmware-architecture.md](firmware-architecture.md), and the step generator in [pio-step-generation.md](pio-step-generation.md).

### 4.1 IMU pipeline (core 1, `Imu.cpp`)

1. **Sampling:** a data-ready interrupt triggers a 14-byte I²C burst. If INT stays high with no interrupt, stall recovery reads anyway. 50 consecutive errors re-initialise the sensor.
2. **Axis mapping:** sensor axes to the robot frame (X forward, Y left, Z up), set in `Config.h`.
3. **Gyro bias:** a 2 s calibration at start-up with a stillness check. Optional auto-bias while the motors have been stopped for 1 s or more.
4. **Attitude filter** (complementary, per sample, dt ≈ 2 ms):
   - **Turn coupling:** pitch += (q − c·r)·dt. Here c = **GYRO_QR**, the yaw-to-pitch gyro coupling. The IMU axis is about 1° off level, so turning leaks into q.
   - **Accelerometer pitch:** atan2(−a_x', √(a_y² + a_z²)), where a_x' = a_x − (acceleration compensation)/g.
   - **Correction:** if |a| is within 0.15 g of 1 g, pitch += dt/(τ + dt)·(accelerometer pitch − pitch). Otherwise the correction is skipped (flag `ACC_REJECT`).
   - **Time constant:** τ = **0.5 s** (`IMU_SET TAU=` at runtime).
5. **Acceleration compensation, `ACCEL_COMP`:**
   - **0:** none.
   - **1:** subtract u/g. This is wrong for this robot (see 7.4).
   - **2 (default):** subtract what the IMU actually feels, A = u(t − `COMP_DELAY_MS`) + `IMU_H`·dq/dt. Here dq/dt is the gyro pitch rate differentiated and low-passed at 30 Hz.
6. **GYRO_QR learning:**
   - **When:** only while balancing is RUNNING and |r| > 30 °/s.
   - **How:** least squares, c = ⟨q·r⟩/⟨r²⟩, with a 3 s time constant, limited to ±0.06.
   - **Reset:** to −0.0177 whenever balancing isn't running.

### 4.2 Balance controller (core 1, `Balance.cpp`)

**State machine:** `OFF → ARMING → RUNNING (or DRY) → TRIPPED → OFF`.

| Stage | What happens |
|---|---|
| Arming | Drivers enabled, wheels locked. Engages when \|θ − TRIM\| ≤ 2° and \|θ̇\| ≤ 20 °/s, i.e. lifted upright. Times out after 20 s. |
| Soft start | The output is ramped in over 0.3 s. |
| Trips | Pitch > `TILT_FWD` or < `TILT_BACK`; speed pinned at `VMAX` for `RUNAWAY` s; no IMU sample for 20 ms or attitude invalid; arming time-out; `BAL_STOP`; `ESTOP`. On a trip the steps stop at once (wheels locked), and the drivers are released 0.5 s later. The black box freezes with the reason. |
| Drive lockout | Any end of balancing sets `DRIVE_LOCK=1`. Motion commands are refused with `BALANCE_TRIPPED` until `CLEAR_ESTOP` or a new `BAL_ARM`. |

**Control law** (SI units, forward positive, every 2 ms):

```text
e        = θ − trim_est − θ_ref                         (rad)
feedback = KP·e + KD·θ̇ + KV·(v − v_ref) + KX·(x − x_ref)
u        = notch(feedback) + DRIVE_FF·a_ref [+ kick]     then soft-start scale, clamp ±AMAX
v       += u·dt  (clamp ±VMAX);   x += v·dt
wheel speeds: v_left = v − w_ref·track/2,  v_right = v + w_ref·track/2   → PIO step rates
```

- **θ̇** is the corrected gyro pitch rate. v and x are the *commanded* wheel speed and position (open loop).
- **Drive references** (from `SET_VELOCITY` in BALANCE mode):
  - **Limits:** V is clamped to ±0.3 m/s and W to ±90 °/s.
  - **Lease:** 100–2000 ms.
  - **Speed S-curve:** a_ref heads for 4·(V − v_ref), limited to ±`DRIVE_ACCEL` (0.8 m/s²) and changing at most 4 m/s³.
  - **Integration:** v_ref integrates a_ref, and x_ref integrates v_ref.
  - **Turn ramp:** w_ref ramps at 180 °/s².
- **Drive feed-forward:** θ_ref = `DRIVE_FF`·a_ref/g + `DRIVE_LEAN`·tanh(v_ref/0.05 m/s). That's a lean for acceleration plus a 2° lean against rolling drag.
- **Automatic trim:** while calm (|v − v_ref| < 0.05 m/s and |θ̇| < 5 °/s), trim_est −= `TRIM_RATE`·(x − x_ref)·dt. It stays within ±`TRIM_RANGE` of `TRIM` and 2° inside the tilt limits.
- **Notch:** a biquad at 500 Hz, `NOTCH_HZ` 20.5, `NOTCH_Q` 2, on the feedback part only. It costs about 4° of phase at 3 Hz.
- **Kicks:** `BAL_KICK U= MS=` adds U (|U| ≤ 4 m/s², 10–200 ms) to u. The telemetry flag is 0x80 (with 0x20).

### 4.3 Motion chain (core 0)

In the normal modes: `MotionController` mixes v and w into wheel step rates with acceleration limiting. `Motor` and `Stepper` feed the PIO state machines. `Robot` owns the modes, the leases, the e-stop latch, finite jobs and driver health checks.

While balancing, core 1 writes step intervals directly. A slow interval is restarted rather than waited out, and a direction change restarts the state machine.

### 4.4 Telemetry and black box

Core 0 drains every IMU sample from core 1 into a 1 s live ring and a 2 s black box. `LOG_START DIV=n` streams every n-th sample. The black box freezes on any balance trip, on `ESTOP` or on `BB_FREEZE`. `BB_DUMP` sends it.

**Columns:**

```text
seq,t_us,t_s,pitch_deg,roll_deg,yaw_deg,p_dps,q_dps,r_dps,ax_g,ay_g,az_g,
u_mm_s2,v_left_sps,v_right_sps,v_left_mm_s,v_right_mm_s,mode,flags
```

**Flags:**

| Bit | Meaning |
|---|---|
| 0x01 | LATE |
| 0x02 | STALL |
| 0x04 | ACC_REJECT |
| 0x08 | CALIBRATING |
| 0x10 | NO_ATTITUDE |
| 0x20 | BALANCE_ACTIVE |
| 0x40 | DRY |
| 0x80 | MOTORS_BUSY, or KICK when set with 0x20 |

**Not logged:** the settings for ACCEL_COMP, the notch, τ and GYRO_QR. The web runs record the gains in their notes and `balance_summary.json`. Logging every setting in `meta.json` is an open item.

---

## 5. Raspberry Pi software

Details: [pi-controller.md](pi-controller.md), [../pi/README.md](../pi/README.md). Deploy with `bash tools/deploy-pi.sh` from the Mac.

| Program | Role |
|---|---|
| `robotd.py` | Serial owner. JSON actions over `/run/user/<uid>/r2w-robotd.sock`. Holds drive requests for `hold_ms` (200–5000) and renews the Pico lease at lease/3, then sends `STOP`. Writes runs to `~/runs/<time>_<name>/`. Re-applies `~/.config/r2w/balance_tuning.json` whenever the SKR uptime resets. |
| `robot_web.py` | Port 8080. `/` is the balance page, `/bench` the three-wheeler bench console. API: `GET /api/status`, `/api/balance`, `/api/runs`; `POST /api/command`, `/api/balance/start`, `/api/balance/stop`, `/api/shutdown`. |
| `robotdctl.py` | CLI for robotd: `status`, `bal-status`, `bal-set K=V…`, `bal-arm`, `bal-stop`, `bal-kick`, `tmc-pwm`, `velocity`, `configure`, `log-start/stop`, `bb-*`, `tuning`, `tuning-clear`, `runs`. |
| `robotctl.py` | Direct serial client, only with robotd stopped. |
| `balance_run.py` | Command-line balance attempt with logging and kicks (`--kicks N`, `--div`). Prints the stood time, end reason and l_eff. |
| `compare_run.py` | Phase 11 comparison protocol: settle, 60 s stand, 10 kicks, 4 pushes, speed steps; writes `compare.json` (all settings and segment times). `--set`/`--tau` apply for that run only. |
| `check_config.py` | Read-only OK/DIFF check of live settings against the defaults (gains, comp, notch, drivers, PWM, saved tuning). |
| `dryrun_check.py`, `fall_test.py`, `limits_test.py`, `log_test.py`, `vibe_test.py` | Commissioning tests (phases 5–9) |
| `tuning.py`, `balance_tools.py` | Shared defaults, ranges, kick analysis (l_eff, damped-sine ring fit) |

**The balance page:**
- **Status:** state, pitch, error, speed, position, trim estimate, max \|err\| and \|u\|, and the drive-lock banner with **Clear**.
- **Run:** Start (arm) with notes and a kick count. Kicks are sent from the page: the first 8 s after engaging, then one every 3 s when calm. The kick count can be changed mid-run.
- **End balancing:** saves the log, the black box and `balance_summary.json`.
- **Drive pad:** hold to drive, speed 50–300 mm/s, turn 15–90 °/s. STOP keeps it balancing; EMERGENCY STOP ends balancing.
  - Drive requests use lease 1000 ms, hold 1500 ms, renewal every 200 ms, with one retry after a failed request.
  - A drive that ends other than by release shows why.
- **Tuning panel:** KP, KD, KV, KX, TRIM, DRIVE_LEAN, DRIVE_ACCEL, DRIVE_FF, TRIM_RATE.
  - Changes over 15 % in one step need the "allow" box ticked.
  - **Save as start-up** stores the values on the Pi.
- **Recent runs:** summaries, CSV download and a plot with the kicks marked.

**Syncing runs to the Mac:**

```bash
rsync -av robot@r2w-pi.local:runs/ ~/Documents/Projects/SKR-Pico-Robot/runs/
```

Wait for "Saved …" on the page first, or the summary may be missing.

**Analysis** (Mac or any Python with numpy, scipy and pandas): `analysis/phase11/phase11_model.py fit|check|steps`, and the scratch scripts in `analysis/phase11/scratch/` (band RMS by speed, estimator models, pole fits).

---

# Part B: parameters and plant

## 6. Parameter reference

All `BAL_SET` keys are live (also while balancing) and are lost at reboot unless saved from the web page. Only the web tuning keys can be saved. The full syntax is in [protocol.md](protocol.md).

### 6.1 Balance controller (`BAL_SET`; defaults in `include/Balance.h`)

| Key | Default | Unit | Valid range | Purpose |
|---|---|---|---|---|
| `KP` | 51.4 | m/s² per rad | 0–500 | Angle gain |
| `KD` | 4.37 | m/s² per rad/s | 0–50 | Angle-rate gain |
| `KV` | 6.89 | m/s² per m/s | ±100 | Speed gain (positive) |
| `KX` | 6.48 | m/s² per m | ±100 | Position gain (positive; also enables trim walking) |
| `TRIM` | 1.9 | ° | between the tilt limits | Balance point (IMU pitch) |
| `TRIM_RATE` | 0.25 | °/(m·s) | 0–10 | Automatic trim speed (0 = off) |
| `TRIM_RANGE` | 4.0 | ° | 0–10 | Trim walking limit around `TRIM` |
| `AMAX` | 6.0 | m/s² | 0–20 | Output clamp |
| `VMAX` | 0.55 | m/s | 0–1 | Wheel speed clamp (runaway trip if pinned) |
| `SOFT` | 0.3 | s | 0–5 | Soft-start ramp |
| `TILT_FWD` / `TILT_BACK` | 10.7 / −14.7 | ° | ±45 | Absolute IMU pitch trips |
| `ARM_WINDOW` / `ARM_RATE` / `ARM_TIMEOUT` | 2.0 / 20 / 20 | ° / °/s / s | | Arming conditions |
| `RUNAWAY` | 0.3 | s | 0.05–5 | Time pinned at VMAX before a trip |
| `DRIVE_ACCEL` | 0.8 | m/s² | 0–3 | Drive S-curve acceleration |
| `DRIVE_FF` | 1.0 | – | 0–2 | Drive feed-forward weight |
| `DRIVE_LEAN` | 2.0 | ° | 0–5 | Lean while moving (rolling drag) |
| `ACCEL_COMP` | 2 | – | 0, 1, 2 | Accelerometer compensation mode |
| `IMU_H` | 0.098 | m | 0–0.5 | IMU height above the axle (mode 2) |
| `COMP_DELAY_MS` | 10 | ms | 0–30 | Command-to-wheel delay (mode 2) |
| `NOTCH_HZ` | 20.5 | Hz | 0 (off) or 5–100 | Notch on the feedback output |
| `NOTCH_Q` | 2.0 | – | 0.3–10 | Notch quality (width = Hz/Q) |

### 6.2 IMU and drivers

| Setting | Default | How to change |
|---|---|---|
| Filter τ | 0.5 s (0.05–5) | `IMU_SET TAU=` (robotd action `imu_set`, `tau`); `Config::IMU_FILTER_TAU_S` |
| Auto-bias | off | `IMU_SET AUTO_BIAS=1` |
| DLPF, ranges | 94 Hz, ±4 g, ±500 °/s | `Imu.cpp` init table |
| Motor current | 1000 mA (100–1300) | `CONFIGURE` (idle only); `Config::TMC_RUN_CURRENT_MA` |
| Microsteps | 16 | `CONFIGURE` |
| Chopper | StealthChop, automatic PWM (AUTOSCALE=1, AUTOGRAD=1) | `CONFIGURE`, `TMC_PWM` |
| Normal-mode acceleration | 150 mm/s² (10–20000) | `CONFIGURE` |

### 6.3 Pi and web

| Setting | Value | Where |
|---|---|---|
| Drive lease / hold / renew | 1000 / 1500 / 200 ms | `pi/web/balance.js` |
| Web gain step guard | 15 % | `balance.js` |
| Kick schedule | ±2 m/s², 40 ms, first at 8 s, every 3 s when calm | `balance.js`; `balance_run.py --kick-u --kick-ms` |
| Saved tuning | `~/.config/r2w/balance_tuning.json` | robotd re-applies it whenever the SKR restarts |
| Expected configuration | `pi/check_config.py` (from `tuning.TUNING_DEFAULTS`, `balance_tools.DEFAULTS`) | |

---

## 7. Plant model and identification

### 7.1 The rigid model

The wheel acceleration a is the commanded u, delayed by τ_d of about 10 ms:

```text
l_eff · θ̈ = g·θ − a
```

With l_eff ≈ 0.094 m, the unstable pole is √(g/l_eff) ≈ 10 rad/s (1.6 Hz). The controller holds that and the slow position and speed modes.

### 7.2 Joint compliance (the 2.2–2.4 Hz wobble)

After every kick the robot rings at about 2.2–2.9 Hz. The damping changes from session to session: ζ about 0.05 settled, up to 0.4 just after handling. In the 11.1 fits this behaves like a lightly damped pole/zero pair between the body and the IMU reading, about 1.2 Hz pole and 1.8 Hz zero. The closed-loop ring is the controller pole drawn towards that zero.
- **Cause:** friction in the prototype's joints (battery strap, wedged IMU, motor mounts).
- **Ruled out:** the motors, current, StealthChop PWM, battery and temperature.
- **Practical rule:** compare runs within one session; handle the robot the same way before each.

### 7.3 Delay

About 10 ms in total: stepper response, IMU DLPF (about 3 ms at 94 Hz), sample timing and the half-sample hold. This limits how much angle gain the robot can use.

### 7.4 The estimator loop, and why ACCEL_COMP mode 2

The accelerometer feels the acceleration of the IMU's own location: a(t − τ_d) + h·θ̈.
- **Why the IMU barely feels the wheels:** with h = 98 mm ≈ l_eff, the h·θ̈ term largely cancels the wheel term (from the model, h·θ̈ ≈ (h/l)(gθ − a)).
- **What mode 1 did wrong:** it subtracted the full commanded u. That injected an error of about u/g at 2–5 Hz. The filter passes it with gain about 1/(τ·ω), and multiplied by KP that forms a second feedback loop. At KP ≥ 59 it gave an almost undamped 4.4–5.2 Hz mode.
- **The two remedies:**
  - **τ 2.0:** weakens the loop four times, but the estimate then drifts 1–1.6° while driving.
  - **Mode 2, now the default:** subtracts the right thing, so τ can stay 0.5 s.

### 7.5 The drive-train resonance (20.5 Hz)

Above about 150–250 mm/s a 20.5 Hz oscillation can build up. It is fed by the controller output, gets worse with stiffer gains and with estimator drift, and it shows as accelerometer rejects of up to 75 %. The notch at 20.5 Hz, Q 2, removes it: the 16–25 Hz band drops from 17–31 °/s to under 1.4 °/s at speed.

### 7.6 Turning

The wheels follow the turn command. The measured yaw rate matched the 60 °/s set point. Two things make a turn look like "one wheel isn't working":
1. **Forward speed carried into the turn:** the speed loop's time constant is about 1 s.
2. **The balance point moves forward in a spin:** about 0.38°·(r in rad/s)², so +0.4° at 60 °/s. The robot creeps 12–25 cm forward until it reaches the new lean.

A turn-lean feed-forward is the planned fix (section 9).

### 7.7 Identification method (the kick test)

1. **Fire the kicks:** while balancing, fire alternating ±2 m/s² pulses of 40 ms, one every 3 s when calm, 20 per run (web page or `balance_run.py --kicks 20`).
2. **Average them:** take a sign-aligned average of the pitch rate and u over −0.1…1.6 s around each kick (`phase11_model.kick_ensemble`).
3. **Fit the ring:** a damped sine from 0.12 s gives the ring frequency and damping (`balance_tools.ring_after`; an ensemble fit is more robust).
4. **Fit the model:** fit plant parameters by simulating the closed loop with the exact firmware law, matching both q and u (`phase11_model.py fit`).
5. **Check it:** the model must reproduce the ring *frequency and damping* in runs it wasn't fitted to, and predict a known failure (it predicted the instability of the 23 Nov redesign).

For a mode that only appears at speed, also compute the RMS of the pitch rate in frequency bands, sorted by wheel speed (`analysis/phase11/scratch/bands.py`).

---

# Part C: results and status

## 8. Controller design history and results

| Set | KP / KD / KV / KX | Design | Outcome |
|---|---|---|---|
| Phase 9 PD | 39 / 2.9 / 0 / 0 | From the model | First balance; drifted (no outer loop) |
| Pole placement (to 24 Nov) | 36.7 / 4.19 / 4.81 / 2.94 | Poles at 10 rad/s (ζ 0.7) and 1.2 rad/s (ζ 0.9), designed for l 0.20 m | Stood for minutes, drove and turned; wobble ζ 0.05–0.25; little margin in the fitted model |
| Redesign (23 Nov) | 30.55 / 2.19 / … | Wrong model | Unstable in 2 s (the fitted model later predicted ζ −0.21) |
| **Everyday (step 2)** | **51.4 / 4.37 / 6.89 / 6.48** | 2/5 of the way to LQR, checked on the model | **The default.** With comp 2, τ 0.5 and the notch: kick ζ 0.27–0.43, pitch std 0.16°, clean drives to 0.38 m/s |
| LQR | 73.5 / 4.63 / 10.0 / 11.8 | Rigid model, Q = diag(10, 1, 300, 0.3) on [x, v, θ, θ̇], R = 0.1 | Standing best with comp 1, τ 2.0, no notch (ζ 0.43, pitch std 0.13–0.16°). Shakes at 20 Hz from about 200 mm/s and fell in every drive test. On the comp 2 defaults it has a marginal 5–6 Hz mode. **Shelved.** |

**What was learnt:**
- **Where the difficulty really was:** the hard problems were the estimator, the delay and the mechanics, not the control law.
- **The LQR gains:** right in shape (much more KP, KV and KX; KD unchanged), but they need margin that the rigid model can't see.
- **Gain margin:** the everyday set gives up very little standing performance and keeps that margin.

The comparison protocol (Phase 11) uses the everyday set as its baseline.

---

## 9. Known issues and open items

| Item | Status | Next action |
|---|---|---|
| Turn creep and the "one wheel" look | Understood (7.6) | Turn-lean feed-forward θ_ref += 0.38°·(w_ref in rad/s)², with an on/off setting; test turns from standstill |
| iPhone drive dropouts | robotd hold raised from 600 to 1500 ms, retry, end reasons shown (24 Nov) | Confirm on the phone; read the reason if it recurs |
| GYRO_QR learnt while carried | Fixed (24 Nov) | Check learned values in turn runs |
| Joint damping varies | Accepted for the prototype | Tighten the battery strap, IMU mount and motor mounts |
| Settings not logged | Open | Write the full `BAL_STATUS` and `IMU` settings into `meta.json` at the start of every run |
| LQR / stiffer gains | Shelved | Possible routes: reduce KX while driving (Klop-style speed mode), better model, Kalman filter |
| Turn-induced estimator effects | Small with the current fixes | — |
| Battery monitoring | None | LiPo alarm on the balance lead; ADC divider later |
| Physical e-stop | None | Independent motor-power cut-off |
| Encoders | None | Needed for true odometry and slip detection |
| Flashing from the Pi | Not implemented | 1200-baud touch + `picotool load` script (RandomQuestions Q25) |
| Permanent IMU mount | Wedged, not fixed | Screwed bracket at a known height |

---

# Part D: bringing a similar device online

## 10. Bring-up guide

This is the sequence we would follow next time, for any two-wheel (or similar inverted-pendulum) robot with an IMU and a stepper or BLDC drive. It is ordered so that each stage removes a class of error before the next stage can be confused by it.

**Rules that apply throughout:**
- **One change per run.** Write down what changed, and why.
- **Log every run, with every setting.** Gains, estimator, filters, driver mode, current, battery and firmware hash all go in the run metadata.
- **Make every tunable a live setting with an A/B switch,** and give each a safe default in firmware.
- **Predict before you test.** Write the expected result next to each test, so a surprise is visible.
- **Gates are hard.** Don't start the next stage until the current one passes. Most lost time on this project came from building on an unverified stage.
- **Keep the same structure for each test** (the same kicks, the same drive pattern), so runs compare directly.

### Stage 0: preparation

| Do | Why |
|---|---|
| Decide the frames and signs on paper: X forward, pitch positive nose-down/forward, positive turn left, u positive forward | Most balancing failures are sign errors |
| Set up the logging path before anything moves: rings on the MCU, streaming to a host, a run folder per test | Every later stage depends on data |
| Write the safety layer early: arming window, soft start, tilt trips at the mechanical stops, runaway trip, sensor-stale trip, black box, drive lockout after a fall | It makes the first attempts safe and informative |
| Provide tilt stops ("training wheels") about 13–17° each way | The robot can fall without damage, and the stop angles calibrate the IMU |

**Gate:** frames written down; logging, safety trips and the black box tested on the bench.

### Stage 1: mechanics and measurements (no power)

| Task | Method | Record |
|---|---|---|
| Rigidity | Shake the robot. Strap the battery positively. Screw the IMU to a rigid bracket. | Nothing moves |
| Wheel backlash | Rock each wheel against the locked motor | None |
| Mass and inertia | Weigh it; swing test about the axle | m, l_eff = J/(m·l) |
| CoM height | Balance on a rod across the plates | l |
| Wheel diameter | Roll 10 revolutions under load | D (±0.2 mm) |
| Track | Tyre centrelines; refine later with turns | b |
| **IMU position** | Height above the axle and fore/aft offset | h, d (this decides the acceleration compensation) |
| Stop angles | Protractor on each stop | θ_fwd, θ_back |

**Gate:** all values recorded; nothing rattles.

### Stage 2: motors and drivers (wheels up)

| Task | Pass |
|---|---|
| Coil pairs, wiring, current (start at about 55 % of rating), microstepping, chopper mode | Smooth rotation both ways |
| Directions per wheel, steps per metre | `move 500` travels 500 mm; `turn 90` turns 90° |
| Heat run at working current, 10 min | Motor and driver temperatures stable, no warnings |
| Speed and acceleration limits on the stand and the floor, in each chopper mode | Know the stall speed and acceleration (balancers fall when they run out of speed) |
| Vibration of the IMU at speed, per chopper mode | Know the resonance speeds |

**Gate:** directions and units correct; limits known; drivers cool.

### Stage 3: IMU and estimator

| Task | Method | Pass |
|---|---|---|
| Axes and signs | Tilt and turn by hand | Pitch is positive when tipped forward; yaw is positive turning left |
| Noise and bias | 60 s still | Gyro sd within datasheet; bias calibrates |
| On-chip filter | Choose the widest DLPF that keeps the noise acceptable (lower delay) | Noise ok, delay ≤ 3 ms |
| Accuracy | Compare with a protractor at the two stops | Within 0.5° |
| **Acceleration compensation** | Use the measured IMU height: subtract the IMU's own acceleration (delayed command + h·θ̈), never the raw wheel command, unless the IMU sits on the axle | While the robot is pushed back and forth by hand, the estimate tracks the accelerometer within 0.2°, and there is no lag-and-overshoot after a push |
| Time constant | Start at 0.5 s | Estimate within 0.3° of the accelerometer pitch in a straight drive (Stage 8) |
| Turn coupling | Spin on the stand or floor: does q show a fraction of r? | Coupling measured; correction learned only while balancing |

**Gate:** signs, accuracy and compensation verified *before* the first balance.

### Stage 4: model

| Task | Method |
|---|---|
| Balance angle | Free-fall tests from both sides with the wheels locked: average θ_eq |
| l_eff | Swing test, and later the kick test |
| Delay | Kick test (Stage 6) |
| First gains | Full-state pole placement for l_eff (check robustness for ±50 %), or PD on angle for the first attempt |

**Gate:** two independent estimates of l_eff and the balance angle agree within about 20 %.

### Stage 5: first balance

1. A dry run with the motors off, the robot tilted by hand: u has the right sign (lean forward → drive forward).
2. Wheels in the air at reduced gain (`--stand --scale 0.3`): the wheels respond correctly.
3. On the floor, lift it into the arming window and it engages. Hold a tether.

**Gate:** stands 60 s with position hold; every trip direction tested; the black box captures each fall.

### Stage 6: identify

- **Kick test** (20 kicks, section 11.1) at the first stable gains.
- **Fit** the closed-loop model (rigid pendulum plus delay, plus any modes) using the exact control law. Check it against runs it wasn't fitted to.

**Gate:** the model reproduces the kick ring frequency within 10 % and damping within ±0.05, in at least two runs.

### Stage 7: tune or design

1. Design the new gains on the fitted model (pole placement or LQR).
2. Check the margins against variants: delay +4 ms, l ±20 %, looser joints.
3. **Step towards the new gains in 4–5 steps.** Predict the ring frequency and damping for each step and measure it.
4. Stop at the first step where the result departs from the prediction, or a new mode appears. That means the model is missing something. Find it before going on.

**Gate:** each step within its prediction; no new mode; standing pitch std and kick damping documented.

### Stage 8: drive

1. Speed S-curve (acceleration and jerk limits), drive feed-forward (a/g lean + acceleration), steady lean for rolling drag.
2. Drive test at increasing speed with the same pattern each time: out and back at 100, 200 and 300 mm/s.
3. Analyse the band RMS of the pitch rate by wheel speed, and the estimate against the accelerometer pitch while moving (section 11.2).
4. If a narrow-band oscillation appears at speed: find its frequency. Notch it if it's fixed in frequency; question the estimator if the estimate drifts.

**Gate:** no band grows with speed up to the maximum speed; the estimate stays within 0.3° of the accelerometer while driving; no falls in three drives.

### Stage 9: turn

1. Turn-rate ramp; learned gyro turn coupling (only while balancing).
2. Spins on the spot from standstill at increasing rate: the forward speed must stay near zero.
3. If it creeps, measure the balance-point shift against r² and add a turn-lean feed-forward.

**Gate:** in a spin on the spot the forward speed stays under 20 mm/s; no inner-wheel stall.

### Stage 10: robustness

Repeat the Stage 6–9 gates with +200 g on top, a low battery, a different floor, a cold start and after handling.

**Gate:** all gates still pass, or the failures are documented with limits.

### Stage 11: comparison and further work

Only now compare controllers (PID, LQR, LQG and others) with a fixed protocol:
- stand 60 s,
- a repeatable push from a pendulum hammer,
- a speed step,
- the sensitivity tests.

The baseline is the configuration that passed every gate.

### Typical numbers for a robot like this (for sanity checks)

| Quantity | R2W value |
|---|---|
| l_eff | 0.09–0.10 m |
| Unstable pole | about 10 rad/s |
| Usable KP | 35–75 m/s² per rad |
| KD | about 4–5 |
| Loop delay | about 10 ms |
| Kick ring | 2–3.5 Hz |
| Drive-train resonance | about 20 Hz |
| Pitch std standing | 0.13–0.2° |

---

## 11. Standard tests and how to analyse them

### 11.1 Kick test (standing)

- **Procedure:** 20 kicks, ±2 m/s², 40 ms, every 3 s when calm, after an 8 s settle. Use the web page with a kick count, or `balance_run.py --kicks 20`.
- **Outputs:**
  - per-kick ring frequency and damping (the page summary);
  - ensemble ring (more robust);
  - l_eff estimate.
- **Pass:** damping ≥ 0.2, and no kick with damping below 0.05 (which would mean a hidden marginal mode).

### 11.2 Drive band test

- **Procedure:** after the kicks, in the same run, drive out and back at 200, then 300 mm/s, then hold one long press for 10 s or more.
- **Analysis:** for wheel-speed bins (0–100, 100–200, 200–260, > 260 mm/s), compute:
  - the pitch-rate RMS in the 1.5–3, 3.8–5.2, 5.2–8 and 16–25 Hz bands;
  - the u standard deviation;
  - the fraction of accelerometer rejects;
  - the low-passed estimate minus the compensated accelerometer pitch.
- **Pass:**
  - no band rises with speed by more than about 2×;
  - u sd ≤ 0.4 m/s² at speed;
  - rejects < 5 %;
  - estimate error < 0.3°.

### 11.3 Turn test

- **Procedure:** spins on the spot at 30, 45 and 60 °/s from standstill, each 3–5 s. Then turns while moving slowly.
- **Analysis:** per turn episode, the mean and maximum forward speed, the inner and outer wheel speeds, the measured yaw rate against the command, and the late pitch against the pre-turn pitch.
- **Pass:** yaw rate within 10 % of the command; forward speed < 20 mm/s in spins on the spot.

### 11.4 Comparison protocol (Phase 11)

`pi/compare_run.py --label NAME [--set KEY=value …] [--tau S] [--speed MM_S]` runs, in one log:
1. a 10 s settle;
2. 60 s standing;
3. 10 standard kicks (±2 m/s², 40 ms);
4. 4 pushes (±3 m/s², 60 ms);
5. speed steps +V / 0 / −V / 0 (2.5 s on, 4 s off; V default 150 mm/s).

`analysis/compare.py RUN …` prints a side-by-side table:
- **Stand:** pitch, rate and u RMS, drift.
- **Kicks:** ensemble ring.
- **Pushes:** peak pitch, distance, recovery, peak u.
- **Steps:** t90, overshoot, wrong-way speed, peak pitch, stop settle and overshoot.
- **Moving:** band RMS and u sd while |v| > 100 mm/s.

Compare runs from one session, after the same handling, at the same battery state.

### 11.5 Configuration check

Before any comparison run, `python3 check_config.py` must print ALL OK, or show only intended differences.

---

## 12. Troubleshooting map

| Symptom | Likely cause | Check / fix |
|---|---|---|
| Runs away in one direction straight after engaging | Wrong sign of u or of the gyro, or a wrong `TRIM` | Dry run (lean forward → u > 0); check `TRIM` against the free-fall angle |
| Drifts steadily away while standing | Wrong trim; KX = 0 | Check `TRIM_EST` walking; KX > 0 |
| Slow wobble at 2–3 Hz with low damping | Joint compliance; settled state | Compare within a session; tighten the mounts; more KD helps a little |
| Almost undamped 4–6 Hz wobble at high gains | Estimator loop (wrong acceleration compensation), or notch phase at marginal gains | Use `ACCEL_COMP=2` with the correct `IMU_H`; try `NOTCH_HZ=0` to test |
| 20 Hz shake above about 200 mm/s; accelerometer rejects | Drive-train resonance fed by the controller | Notch 20.5 Hz, Q 2; avoid τ 2.0 (estimate drift) |
| Leans more and more while driving; estimate differs from the accelerometer | τ too long, or wrong compensation | τ 0.5 with comp 2 |
| "One wheel not working" in turns | Forward speed in the turn (carried over, or creep from the spin lean) | Wait about 1 s after driving before turning; turn-lean feed-forward |
| Turn makes it lurch forward in one direction and back in the other | Wrong GYRO_QR | Check `GYRO_QR` in `BAL_STATUS`; it resets each arm |
| Drive stops after a few seconds (phone) | Request gaps > hold time | Hold 1500 ms; read the reason message |
| `BALANCE_TRIPPED` on drive | Drive lockout after a fall | **Clear** on the page, or `clear-estop` |
| `UNKNOWN_FIELD` on `BAL_SET` | Firmware older than the key | Build *and* flash; check that `firmware.uf2` is newer than the source |
| Summary missing after a sync | Synced before the page finished | Wait for "Saved …", then sync again |
| Black box shows `ARM_TIMEOUT`, not the fall | Re-armed after the fall | Dump before re-arming (`bb-dump`) |
| `configure` refused with `DRIVER_FAULT` at 1.3 A | Read-back tolerance (fixed: max(25 mA, I/16)) | Update the firmware |

---

# Appendices

## 13. Glossary

| Term | Meaning |
|---|---|
| l_eff | Effective pendulum length: inertia about the axle / (m·l) |
| Kick | A short added acceleration pulse used to excite the closed loop |
| Ring | The damped oscillation after a kick |
| ζ (damping ratio) | 0 = undamped, 1 = critically damped; ≥ 0.3 is comfortable |
| τ (filter) | Complementary-filter time constant: how slowly the accelerometer corrects the gyro |
| ACCEL_COMP | How the IMU's own acceleration is removed from the accelerometer before correction |
| GYRO_QR | Learned fraction of the yaw rate appearing on the pitch gyro |
| Notch | A filter removing a narrow frequency band (20.5 Hz) from the controller output |
| Everyday set | KP 51.4, KD 4.37, KV 6.89, KX 6.48 with comp 2, τ 0.5 and the notch (the default) |
| Drive lockout | Refusal of motion after balancing ends, until cleared |
| Lease / hold | Pico-side and robotd-side expiry times for drive commands |

## 14. Timeline

| Date | Milestone |
|---|---|
| ≤ 21 Nov | Three-wheeled R2W: protocol, PIO steps, robotd, web page, IMU on core 1 |
| 22 Nov | Balancing chassis; phases 1–7 (measurements, motors, power, directions, IMU, logging, limits, free fall) |
| 23 Nov (morning) | Phases 8–10: safety layer, first balance, outer loops, driving and turning, drive feed-forward, GYRO_QR, drive lockout |
| 23 Nov (day) | 9.4 kick tests and the 94 Hz DLPF; 11.0 wobble investigation (joint friction); web balance page; saved tuning |
| 23 Nov (evening) | 11.1 model; 11.2 steps to LQR; 11.3 the 4–5 Hz estimator mode → τ 2.0; drive falls with a 20 Hz shake |
| 24 Nov | Shake traced to τ 2.0 drift and the drive resonance; ACCEL_COMP=2 (IMU at 98 mm); 20.5 Hz notch; everyday set made the default; LQR shelved; GYRO_QR and iPhone fixes; turn analysis |

## 15. Document map

| Document | Contents |
|---|---|
| [README.md](README.md) | Reading order and conventions |
| [system-overview.md](system-overview.md) | Purpose, boundary, motion types |
| [hardware-reference.md](hardware-reference.md) | Wiring, pins, motors, IMU, power |
| [skr-pico-circuit-reference.md](skr-pico-circuit-reference.md) | SKR Pico circuit notes; an SPI IMU option |
| [firmware-architecture.md](firmware-architecture.md) | Classes, cores, modes, limits |
| [pio-step-generation.md](pio-step-generation.md), [Stepper_PIO.md](Stepper_PIO.md) | PIO step generator (detailed, and a tutorial) |
| [protocol.md](protocol.md) | Full `R2W/1` command reference |
| [pi-controller.md](pi-controller.md), [../pi/README.md](../pi/README.md) | Pi programs, web pages, test scripts |
| [safety.md](safety.md) | Safety layers, balancing safety, residual risks |
| [operations.md](operations.md) | Everyday operation, including balancing |
| [testing-troubleshooting.md](testing-troubleshooting.md) | Bench tests and common faults |
| [backup-recovery.md](backup-recovery.md) | Source control, deployment, rebuild |
| [balancing-commissioning-plan.md](balancing-commissioning-plan.md) | The phased plan used for this robot |
| [balancing-results.md](balancing-results.md) | Dated results for phases 1–10 and 11.0 |
| [phase11-model.md](phase11-model.md) | Dated Phase 11 work: model, LQR steps, estimator and filter fixes, drive and turn tests |
| [RandomQuestions.md](RandomQuestions.md) | Background Q&A (29 questions) |
| [roadmap.md](roadmap.md) | Future work |
| [step-3-robot-api.md](step-3-robot-api.md), [imu-dual-core-test-plan.md](imu-dual-core-test-plan.md) | Historical (three-wheeler) |
