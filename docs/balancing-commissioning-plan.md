# Balancing robot: commissioning plan

Started 22 Nov 2026. The SKR Pico robot electronics (14.8 V 4S LiPo, buck converter, Pi Zero 2 W, SKR Pico, MPU6050) have moved into a two-wheel chassis, with 3D-printed "training wheels" that limit tilt. The long-term goal is to compare PID, LQR and other balance controllers. This plan gets the robot from "assembled" to "balancing on a simple controller". It also builds the logging and model that make the later comparison fair.

**How it works:** finish each phase and record its numbers in [balancing-results.md](balancing-results.md) before starting the next. Phases 1–6 need no balancing code. Most past difficulty with balancing came from wrong signs, an unknown balance angle and guessed gains. Phases 5–8 remove all three before the first attempt.

**Status (23 Nov 2026):** Phases 1–9 complete; Phase 10 built (full-state control, automatic trim, driving, accel compensation) and push-tested. **First balance: 64.6 s, pitch sd 0.12°, position within ±7 mm, with the model-derived PD gains. With full-state control and ACCEL_COMP it recovers from 10° pushes and returns home.** Next: web drive test, then Phase 11.

---

## 0. Known data

### 0.1 Motors: Rtelligent (SZruitech) 42A08, open-loop NEMA 17

SZruitech.com is Rtelligent's domain. Published figures for the 42A08:

| Parameter | Value | Note |
|---|---|---|
| Step angle | 1.8° (200 full steps/rev) | Confirmed |
| Rated current | **1.8 A/phase** (some listings say 1.8–2.0 A) | Rtelligent lists don't say RMS or peak. Treat 1.8 A as the ceiling. |
| Holding torque | 0.68–0.71 N·m | Listings vary |
| Phase resistance | 2.0 Ω ±10% | Measured about 2.4–2.7 Ω with a meter whose leads read 0.8–1.2 Ω |
| Phase inductance | 4.8 mH ±20% | High. This limits top speed at 14.8 V (see 0.3). |
| Rotor inertia | 114 g·cm² | Use in the model (reflected inertia) |
| Body | 42 × 42 × 60 mm, 4 wires, 30 cm lead, no connector | Coil pairs: Red–Blue, Green–Black |
| Shaft | 5 mm, 24 mm long | Check that the wheel hubs fit with no play |
| Weight | about 0.45–0.5 kg each | |

A closed-loop "42A08EC" also exists (2.8 A, 8 mm shaft, encoder). If the motor has an encoder cable or an 8 mm shaft, you have that version, and this table doesn't apply.

### 0.2 Chassis geometry

| Item | Value |
|---|---|
| Plates | 168 × 76 mm; top 2 mm, bottom 3 mm aluminium; 61 mm gap |
| Wheel diameter | 62.33 mm measured (nominal 65 mm) |
| Track (tyre centrelines) | 223 mm |
| Overall width | 248 mm |
| Layout | Motors under the bottom plate; SKR Pico on the bottom plate; battery on the top plate; Pi and buck converter on the battery; MPU6050 taped to the battery layer. The wheels aren't mounted symmetrically under the platforms. |
| Supply | 4S LiPo: 16.8 V full, about 14.8 V nominal, about 14 V under load late in a pack |

### 0.3 First estimates (superseded by measurements in the results doc)

- **Distance per step** at 1/16 microstepping: 200 × 16 / (π × 0.06233 m) ≈ **16,343 microsteps/m**. 1 m/s ≈ 16.3 kHz step rate per motor.
- **Speed limit from inductance:** the current can reverse only so fast. The maximum full-step rate is roughly V / (2 L I). At 14.8 V, 4.8 mH and about 1.4 A peak (1.0 A RMS), that gives about 1100 full steps/s ≈ 5.5 rev/s ≈ **1.07 m/s**. Back-EMF lowers it further. **Expect torque to fall off above about 0.6–0.9 m/s.** Phase 7 measures the real limit. In a balancer, running out of speed is the usual way to fall over.
- **Torque needed:** recovering from a tilt θ needs a wheel acceleration of about g·tanθ, which is 1.7 m/s² at 10°. With a robot of about 2 kg, that's about 0.03 N·m per motor at the wheel, plus 0.0006 N·m for rotor inertia. That's far below what the motor gives. **Speed is the limit, not torque.**
- **CoM:** measured about 30 mm above the axle; l_eff ≈ 95 mm; ω₀ ≈ 10 rad/s. A low CoM falls *faster*. The acceleration needed per degree of tilt doesn't depend on height, which makes a higher CoM *easier* to balance. Adding about 200 g on top is a valid trick for the first attempts.

---

## Phase 1: Mechanical checks and physical measurements (no power)

| # | Task | Pass / record |
|---|---|---|
| 1.1 | Wheels on the 5 mm shafts: grub screws on the flats, thread-locked. Rock each wheel by hand against the locked motor. | **No measurable backlash.** Play at the hub looks like noise and delay to the controller. |
| 1.2 | Motor mounts tight; axles parallel and in line (sight along them). | Record |
| 1.3 | Every board, the battery and the Pi secured. Strap the battery positively, not just with tape: it is the biggest mass, and it's on top. | Nothing moves when the robot is shaken |
| 1.4 | **IMU mounting:** tape is a temporary fix. Replace it with screws, standoffs or a printed clip plus foam tape. Mark the IMU's X/Y/Z on the chassis. | Rigid; axes noted |
| 1.5 | Weigh the robot (*m*). Weigh one motor and one wheel. | Record |
| 1.6 | **Wheel diameter:** roll 10 revolutions in a straight line and measure the distance; *D* = distance / (10π). Use the loaded tyre. | Record *D* (±0.2 mm) |
| 1.7 | **CoM height *l* above the axle:** balance the robot sideways on a pencil or rod laid across the plates; mark the balance line. | Record *l* |
| 1.8 | **Inertia *J* (swing test):** hang the robot from a pivot **above** its CoM (a rod across the top plate), with the wheels on. Swing it ±5° and time 20 periods; measure the pivot-to-CoM distance *d*. Then J_cm = m·g·d·T²/(4π²) − m·d² and **J_axle = J_cm + m·l²**. Don't swing it about the motor shafts: stepper detent torque spoils the period. `balance_calc.py` does the arithmetic. | Record *T*, *d*, *J*; **l_eff = J / (m l)** |
| 1.9 | **Training-wheel limits:** put the robot on a flat surface, resting on each training wheel in turn. Measure the tilt with a phone inclinometer on the top plate. | Record θ_fwd and θ_back |
| 1.10 | Make a **wheels-up stand**: a block under the bottom plate, so both wheels spin freely and the robot can't drive off. | Done |
| 1.11 | **Kill switch:** fit a pull-loop or switch in the battery lead within easy reach, and a LiPo low-voltage buzzer on the balance lead. The SKR doesn't measure the battery voltage (see 3.5). | Fitted |

## Phase 2: Motor and driver bench checks (wheels up)

| # | Task | Pass / record |
|---|---|---|
| 2.1 | **Coil pairs:** with the motor unplugged, measure resistance between all wire pairs. One coil reads about 2.0 Ω, and coils are open circuit to each other. Label the pairs with their colours. | 2 pairs at 1.8–2.2 Ω (subtract the meter-lead resistance) |
| 2.2 | **Connector:** crimp to match the SKR Pico motor header. One coil goes on pins 1–2, the other on pins 3–4. Make both motors identically (same colour on the same pin), so the only difference between them is the firmware direction flag. | Both cables the same; photo taken |
| 2.3 | Plug into the **same driver slots as the three-wheeler** (so the TMC UART addresses and firmware mapping carry over). Always plug and unplug with the power **off**. A stepper unplugged under power can kill a TMC2209. | Left = X, right = Y |
| 2.4 | **Run current:** set it through UART (`rms_current`). Start at **1.0 A RMS** and hold current at about 0.5 A. The drivers have small heatsinks. More than about 1.2–1.4 A RMS needs airflow. You could run a small fan from a FAN output. | Driver UART reads back the values |
| 2.5 | **Microstepping:** 1/16 with interpolation to 256 (`intpol`) on. | Record |
| 2.6 | **Chopper:** test both StealthChop and SpreadCycle (vibration in 5.4, step loss in 7.1). StealthChop is quiet but can lose torque or stall under the sudden load changes balancing creates. | Decision recorded |
| 2.7 | **Heat run:** both motors at 2 rev/s for 10 min at 1.0 A RMS. Touch or IR-check the motors and the driver area. | Motors < 70 °C, no driver over-temperature flags (`ot`/`otpw` in DRV_STATUS) |

## Phase 3: Power checks

| # | Task | Pass / record |
|---|---|---|
| 3.1 | Check the polarity of the battery harness before connecting. The drivers rely on the input TVS and fuse for reverse protection, so use a keyed connector. | Correct |
| 3.2 | Run both motors hard on the stand, then check the Pi. | `vcgencmd get_throttled` = 0x0 after the test |
| 3.3 | Repeat 3.2 at a low battery (about 14.0 V resting). | Same |
| 3.4 | Pi Wi-Fi: turn off power saving (`nmcli … 802-11-wireless.powersave 2`). If the 17 Nov Wi-Fi chip fault recurs, do a cold power-off. | Done |
| 3.5 | Optional, for later: battery voltage into the RP2040 through a divider on a thermistor input (TH0/GPIO27). Account for the 4.7 kΩ pull-up, or remove R25. Voltage sag affects stepper top speed, so this is worth logging in the comparisons. | Optional |

## Phase 4: Direction and units

**Robot frame:** X forward, Y left, Z up. **Pitch θ positive = top tilted forward.** **Wheel travel positive = forward.**

| # | Test | Pass |
|---|---|---|
| 4.1/4.2 | `move 200` on the stand (both wheels forward), then `turn 90` (left back, right forward). | Both correct. If not, invert the DIR flag in `main.cpp`. Don't rewire. |
| 4.3/4.5 | `move 500` on the floor beside a tape measure. | 500 mm ±10 mm, straight |
| 4.4 | Turn command (positive = left/CCW seen from above); IMU yaw rises. | Correct way |

## Phase 5: IMU in the new mounting

| # | Test | Pass |
|---|---|---|
| 5.1 | Rest on each training wheel and read pitch. Pitch must be higher on the front stop. | **Sign of θ checked** |
| 5.2 | **The gyro rate agrees with the angle's derivative:** tilt forward slowly; the pitch rate is positive while the angle rises. | Signs agree. A mismatch here is a classic balancer bug. |
| 5.3 | Tilt span between the stops matches the protractor (within about 2°). | |
| 5.4 | **Vibration** (`pi/vibe_test.py`): wheels up, 0–3 rev/s in both chopper modes. | Gyro noise < 0.5 dps RMS; pitch estimate steady |
| 5.5 | **Accelerometer and motion:** when the wheels accelerate at *a*, the accelerometer reads it as a tilt error of about a/g rad (1.7 m/s² ≈ 10°). The complementary filter (τ = 0.5 s) and the 0.15 g gate limit this, but it still biases the angle during sustained acceleration. **Stepper advantage:** firmware knows the commanded wheel acceleration, so it can subtract it from the accelerometer X axis before the correction step. Add this as an option (`IMU_ACCEL_COMP`) and compare it against no compensation in Phase 9. | Implemented as a switch |
| 5.6 | **Filter delay:** the current DLPF setting (CONFIG 0x03, about 44 Hz) adds about 5 ms of group delay. Try 0x02 (about 94 Hz, about 3 ms). Delay matters more than noise in a fast balancer. | Record the chosen setting. **Switched to 0x02 on 23 Nov (firmware `IMU: DLPF 94 Hz`), checked in 9.4.** |
| 5.7 | **IMU height:** the accelerometer also sees θ̈ × (height above the axle) during fast corrections. 5.5 removes the larger cart-acceleration term. Lower the IMU later if logs show the angular-acceleration term matters. | Note |

## Phase 6: Telemetry and logging (before any balancing)

The robot can't stay connected to the phone or the Mac while it runs, so the robot records data itself. The phone only starts and stops runs.

1. **The Pico streams, the Pi records.** During a run, core 0 sends one compact record over USB every 2 ms or 4 ms (500 or 250 Hz). Each record holds: t_us, θ, θ̇, raw ax/az, commanded acceleration *u*, wheel velocity L/R, wheel position, mode/state flags, loop time and error counters. That's about 40 bytes, so 10–20 kB/s, inside the USB rate already proven not to disturb core 1 (Stage 3: 18 kB/s, 0 missed samples).
2. **`robotd` writes one folder per run:** `~/runs/<timestamp>_<test>/` holding `data.csv` (or binary plus a converter), and `meta.json` with the controller type, every gain, the firmware git hash, the battery voltage if available, and the test name and notes.
3. **Pico "black box":** a RAM ring buffer of the last 2 s at 500 Hz (about 40 kB of the RP2040's 264 kB). The buffer freezes when a trip happens (tilt cut-out, stall, watchdog). The Pi then fetches it, so the 2 s before every fall are captured at full rate even if the USB stream hiccups.
4. **Web page:** arm/disarm; a test selector; **gains editable at runtime** (stored to flash on request, so no reflash per tuning step); a run list with CSV download and a quick plot of θ, *u* and v.
5. **Analysis on the Mac:** `rsync` the `runs/` folder, then use a Python notebook. One script plots any run, and later computes the comparison metrics (Phase 11).
6. **Suspend the 10 Hz STATUS polling and the TMC UART reads while balancing.** STATUS takes about 16.5 ms of core-0 time. Read driver flags only when disarmed, or once per second at most.

**Pass:** a 30 s dummy run on the stand (motors following a sine-wave velocity profile) gives a complete file with no gaps in t_us, and the black box can be fetched after a forced trip.

## Phase 7: Stepper limits and plant identification (stand, then floor)

| # | Test | Purpose |
|---|---|---|
| 7.1 | **Speed and acceleration limit**, wheels up: step-change the velocity at increasing sizes and ramp the acceleration up (e.g. 2, 5, 10, 20, 40 m/s²) until steps are lost. Check by commanding out-and-back moves and seeing whether a tyre mark returns to the same place; TMC `stallguard` / DRV_STATUS help. Do it in both chopper modes, at 1.0 and 1.3 A, and at full and low battery. | Gives **v_max and a_max**, the saturation limits for every controller. Use about 70% of them as limits in firmware. |
| 7.2 | The same test on the floor (robot resting on the training wheels, short moves). | Load and tyre slip reduce the limits |
| 7.3 | **Free-fall test:** hold the robot just off its balance point with the motors holding position. Release; it falls onto a training wheel. Log θ(t) and fit θ_eq + A·e^(ωt) + B·e^(−ωt). **Note (23 Nov):** with the wheels locked the robot *rolls* rather than pivoting about the axle, so ω_lock² = m·g·l/(J_axle + m·r·(2l + r)), not g/l_eff. The test gives the balance angle well (average both fall directions) but bounds l_eff only loosely. | θ_trim, and a range for l_eff |
| 7.4 | **Balance angle θ_trim:** find where the robot balances on its wheels with the motors holding. Hand-steady it until it stays in place for about 1 s, and read the logged θ (do several tries, and both directions). Error here makes the robot run away in one direction. | Record θ_trim (the controller uses θ − θ_trim) |

## Phase 8: Safety layer in firmware (before the first balancing attempt)

On core 1 with the balance loop, independent of the Pi:

- **States:** DISARMED → ARMING → BALANCING → TRIPPED. Arming is only allowed when |θ − θ_trim| < 3° and the robot is still. In practice you lift it upright by hand, then press Arm on the phone or a button.
- **Tilt cut-out:** separate forward and back limits, 2° inside each training-wheel stop in the IMU frame (currently +5.0° / −20.6° IMU pitch). Motors stop, then disable after 0.5 s, and the black box freezes.
- **Saturation trip:** commanded speed at v_max for more than 0.3 s → TRIPPED (the robot is running away).
- **IMU health:** a missed sample or stall → TRIPPED.
- **Link watchdog:** no Pi heartbeat for 0.5 s → set the speed and turn targets to 0 and keep balancing. Don't drop the robot because Wi-Fi went away.
- **Soft start:** after arming, ramp the controller output from 0 to 100% over 0.3 s.
- **E-stop** from the web page and the physical kill switch.

**Pass:** each trip, caused on purpose on the stand or by hand, produces the right state, the motors stop, and the black box is saved.

## Phase 9: First balance: PD on angle, with gains from the model

**Structure** (core 1, 500 Hz, triggered by the IMU interrupt):
`u = Kp·(θ − θ_trim) + Kd·θ̇` → wheel acceleration *u* (clamped to ±a_max) → integrate to wheel speed (clamped to ±v_max) → step rate for both wheels. Steps must come from PIO or a hardware timer, and the step generator must apply speed changes immediately, even at low speeds (the current PIO program waits for the current step interval to finish).

**Why acceleration is the command:** a stepper follows the commanded speed exactly (unless it loses steps). So the plant is the classic "cart with an acceleration input":
J·θ̈ = m·g·l·θ − m·l·u  →  θ̈ = (g·θ − u) / l_eff

**Starting gains from that model** (choose the natural frequency ω_n and damping ζ):
- Kp = g + ω_n²·l_eff  (m/s² per rad; must be greater than g)
- Kd = 2·ζ·ω_n·l_eff  (m/s² per rad/s)
- Choose ω_n ≈ 1.5–2 × ω₀ (from 7.3), ζ ≈ 0.7.
- With the measured l_eff = 95 mm (ω₀ ≈ 10.1 rad/s), ω_n = 1.75 ω₀: Kp ≈ 39.9 m/s²/rad (0.70 per degree), Kd ≈ 2.37 m/s² per rad/s.

| # | Step | Pass |
|---|---|---|
| 9.1 | **Dry run, motors disabled:** arm, tilt by hand, read the logged *u*. Tilting forward must give *u* > 0 (wheels accelerate forward, under the CoM). | **Sign of the loop checked** (the most important test in the plan) |
| 9.2 | **Wheels up, motors on, Kp at 30%:** tilt forward by hand; the wheels spin forward. | Right way |
| 9.3 | **On the floor, hand guarding:** hands either side, not touching. Arm upright. Expect it to stand for a few seconds and then drift away in one direction (PD on angle alone has no position hold). That counts as success. | Stands 3–10 s; log every run |
| 9.4b | **l_eff by kicks:** `balance_run.py --kicks 10` adds alternating ±2 m/s² pulses of 40 ms while balancing; l = ∫(g(θ − θ₀) − u)dt / Δq over each kick. Redesign the gains for the measured l_eff. | Consistent l_eff (spread within ±20%) |
| 9.4 | Tune from the logs, not by eye: oscillation of about 3–8 Hz → raise Kd or reduce delay (5.6); slow toppling → raise Kp; drift always the same way → fix θ_trim. | |

## Phase 10: Outer loops (position and speed hold), then driving

- **PID cascade version:** an outer PI on wheel speed sets the target angle, θ_ref = −K_v·(v − v_ref) − K_i∫(v − v_ref). The inner PD tracks θ_ref. Keep the outer loop 5–10× slower than the inner one. The integral term also trims out the residual θ_trim error.
- **Non-minimum-phase behaviour:** to move forward, the robot first has to roll *back* a little, so it leans forward. The outer loop does this automatically. Don't treat it as a bug.
- Then drive and turn commands from the web UI: speed reference, plus a turn term (± a differential speed on the two wheels).

**Pass:** stands for 60 s within ±0.2 m; recovers from a light push; drives slowly forward and back on command.

## Phase 11: Controller comparison: PID, LQR and others

**State:** x = [wheel position, wheel speed, θ, θ̇], input *u* = wheel acceleration (the same plant as Phase 9).

- **LQR:** linearise the model with the measured m, l, J, l_eff and the rotor and wheel inertia. Compute K on the Mac (Python `control` or `scipy`) for a few Q/R choices. The firmware runs u = −K·x, the same loop with a different gain line. Compare it against pole placement at the same closed-loop poles as a sanity check.
- **Other candidates later:** a Kalman filter for the angle (in place of the complementary filter) or LQG; a disturbance observer/ADRC; MPC (offline first; the RP2040 has software floating point, about 93 µs per sample for the current attitude maths).
- **Fair test protocol** (the same for every controller):
  1. Stand still 60 s: RMS θ, position drift, RMS *u*.
  2. **Repeatable push:** a pendulum "hammer" (a fixed mass swung from a fixed height against the top plate). Measure the peak θ, recovery time and distance travelled.
  3. Step in speed reference (0 → 0.3 m/s → 0): overshoot and settling.
  4. Sensitivity: the same tests with +200 g on top (model mismatch), and at low battery.
  - Record the battery voltage, chopper mode, motor current and firmware hash in `meta.json` for every run.

---

## Open checks

- [x] Coil resistance and wire colours: Red–Blue, Green–Black; about 2.4–2.7 Ω measured (published 2.0 Ω).
- [ ] Is the 1.8 A rating RMS or peak? (Ask SZruitech, or stay at ≤ 1.3 A RMS.)
- [x] Driver slots: left = X, right = Y; left DIR not inverted, right inverted.
- [x] Measured *D*, *m*, *l*, *J*, l_eff, training-wheel angles. [ ] ω₀ (7.3), θ_trim (7.4).
- [ ] v_max and a_max for each chopper mode.

## Sources

- Rtelligent 42A08 listings: Amazon US B07TQS9ZL1 (0.68 N·m, 2.0 Ω, 4.8 mH, 114 g·cm², 42 × 42 × 60 mm, 5 mm shaft) and Amazon DE/IT B07VNCB7LD / B07VS66VWV (71 N·cm, 1.8 A). The Rtelligent catalogue lists the szruitech.com contact address.
- Project docs: `skr-pico-circuit-reference.md`, `hardware-reference.md`, `imu-dual-core-test-plan.md`.
