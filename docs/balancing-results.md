# Balancing robot: commissioning results

Results log for [balancing-commissioning-plan.md](balancing-commissioning-plan.md). Calculator: `balance_calc.py` (Python, standard library only; kept with the project notes).

## Method changes from the original plan

**1.8 swing test:** do it about a pivot *above* the CoM (a rod across the top plate), not about the motor shafts. Unpowered steppers have about 0.02 N·m of detent torque, which acts like a spring and spoils the period. Convert to the axle with the parallel-axis formula: J_axle = J_cm + m·l², where J_cm = m·g·d·T²/(4π²) − m·d². Do the test with the wheels on and use the total mass. The wheels sit on the axle, so the gravity moment m·l is unchanged, and their spin inertia (wheels plus rotors, under 1 kg·cm²) is negligible.

**Balance angle vs platform angle:** the wheels aren't mounted symmetrically under the platforms, so the balance point won't be with the platforms level. The controller uses θ − θ_trim, where θ_trim is the IMU pitch at the measured balance point (7.4). An IMU mounting offset in pitch is then absorbed automatically. No level-platform reference is needed.

**Correction:** the TMC2209s on this SKR Pico *do* have heatsinks fitted.

## 2026-11-22: Phase 1, mechanical checks and measurements

| # | Item | Result |
|---|---|---|
| 1.1 | Wheel hubs | Solid, no backlash |
| 1.2 | Motor alignment | As good as practical for the prototype |
| 1.5 | Mass m | **1880 g** total; wheel + hub + screws 45 g each |
| 1.6 | Roll test | 1958 mm in 10 revolutions → **D = 62.33 mm** (not 65) → **16,343 microsteps/m** at 1/16 |
| 1.7 | CoM height above axle, l | **≈ 30 mm** (hard to measure; ±5 mm) |
| 1.8 | Swing about a pivot above the CoM | 13 swings in 8 s → T = 0.615 s; d = 63 mm (pivot ≈ 93 mm above the axle, about the top plate, which checks out) |
| 1.9 | Training wheels | First measured as back 6°, forward 16°. **After trimming: forward 16°, back 12°** (protractor, platform). The back limit is set by the chassis touching the floor. |
| 1.11 | Kill switch | Switch in the LiPo lead. LiPo alarm buzzer still to fit. |

**Model numbers**

| Quantity | Value |
|---|---|
| J about CoM | 36.8 kg·cm² |
| J about axle | 53.8 kg·cm² |
| **l_eff = J/(m·l)** | **95 mm** (91–103 mm for l = 35–25 mm) |
| **Predicted ω₀ = √(g/l_eff)** | **10.1 rad/s** (9.7–10.4); fall time constant about 100 ms |
| Speed limit from inductance (upper bound) | about 1.07 m/s at 14.8 V, 1.0 A RMS (assumes the spec 4.8 mH) |
| Step rate at 1 m/s | 16.3 kHz per motor |

l_eff barely depends on the uncertain l, because J_cm dominates. Phase 7.3 (free-fall fit) is the cross-check.

**Starting PD gains** (u = Kp·θ + Kd·θ̇, u in m/s², ζ = 0.7)

| ω_n/ω₀ | ω_n (rad/s) | Kp (per rad) | Kp (per °) | Kd (per rad/s) | u at 5° (m/s²) |
|---|---|---|---|---|---|
| 1.5 | 15.2 | 31.9 | 0.556 | 2.03 | 2.8 |
| **1.75** | **17.8** | **39.9** | **0.696** | **2.37** | **3.5** |
| 2.0 | 20.3 | 49.1 | 0.856 | 2.71 | 4.3 |

## 2026-11-22: Phase 2, motor and driver checks

| # | Item | Result |
|---|---|---|
| 2.1 | Coil pairs | **Red–Blue** and **Green–Black** |
| 2.1 | Coil resistance | 3.4 Ω on the meter, with 0.8–1.2 Ω from the meter leads alone → **about 2.4 Ω; recorded as 2.7 Ω**. The spec is 2.0 Ω ±10%, and a cheap meter can't measure this accurately. Phase 7.1 measures the speed limit directly. |
| 2.3 | Drivers after deploy | X (left) and Y (right) both `OK_IDLE`, no faults; 1000 mA, 16 microsteps, StealthChop confirmed by status |
| 2.7 | **Heat run** (stand, 390 mm/s = 2 rev/s, 1.0 A RMS, StealthChop, 10 min) | **CS_ACTUAL = 17 on both drivers while running** (17 is the TMCStepper setting for 1000 mA with 0.11 Ω sense resistors; idle/hold shows 8), so the full current was flowing. **Motors 33.1 °C** at 10 min (30.1 °C at 5 min), IR thermometer. **TMC2209 heatsink 50.3 °C.** Room about 28 °C (IMU die temperature 28.9 °C). Both drivers `OK_ACTIVE` every minute; no faults. |

**Heat-run conclusions**
- The motors barely warm up (about 5 °C rise). Motor heat isn't the limit.
- **The drivers are the limit:** about 22 °C rise on the heatsink at 1.0 A. Driver loss scales with I², so 1.3 A would give about 1.7 × the rise: **about 65 °C heatsink** in a 28 °C room. That's still well below the 120 °C pre-warning, but leave margin for summer and an enclosed chassis. A small fan on a FAN output would help if we go above 1.0 A.
- **A balancer is always "moving",** so the drivers sit at run current the whole time, not hold current. This heat run is representative of balancing.
- Decision: **1.0 A stays the working current.** Revisit 1.2–1.3 A only if the Phase 7 speed and acceleration limits are too low.

**Firmware and Pi changes** (branch `imu-integration`)
- `8eba92e Balance Bringup`: `Config.h` wheel diameter 62.33 mm and track 223 mm; 1/16 microstepping; boot current 1000 mA RMS with a runtime ceiling of 1300 mA; raw speed cap 20,000 steps/s; acceleration ceiling 20,000 mm/s² (for the Phase 7 limit tests; the default stays 150). Finite MOVE/TURN run at a constant 50 mm/s, whatever the microstepping. Web tuning panel and `robotd` defaults: 1000 mA, 16 microsteps. Built with PlatformIO: SUCCESS, 0 errors, 0 warnings (RAM 4.6%, flash 4.7%). Deployed with `tools/deploy-pi.sh`.
- `d5b6b55 Invert wheel directions for balance chassis` (see Phase 4).
- `2e7e861`: `pi/vibe_test.py`, plus documentation updates (hardware reference, calibration, Pi README, roadmap, this file and the plan).
- Noted for Phase 9: the PIO step generator only applies a new step interval after the current one finishes, so at low speeds (near zero, which a balancer crosses constantly) speed changes lag by up to one step period. The balance loop will need a step generator that updates immediately.
- Pi `robotd` caps a velocity hold at 5 s. Long runs re-send the command every 3 s from a shell loop on the Pi (they stop within 5 s if the link drops).

## 2026-11-22: Phase 3, power

- After the 10-minute heat run: Pi `vcgencmd get_throttled` = **0x0**. No undervoltage while the motors ran. ✔
- 3.3 (repeat at a low battery): later, when a pack is nearly flat.

## 2026-11-22: Phase 4, directions and distance

- First forward move: both wheels ran backwards. **Both direction flags inverted** in `main.cpp`: left `false`, right `true` (the three-wheeler was left `true`, right `false`).
- **4.1/4.2 Test A** (`move 200`): both wheels forward ✔. **Test B** (`turn 90`): left back, right forward ✔.
- **4.3/4.5** `move 500` on the floor: **500 mm exactly, ran straight** ✔. Confirms D = 62.33 mm and matched tyres.

## 2026-11-22: Phase 5, IMU in the new mounting

Resting on the training wheels (`robotdctl status`):

| Position | IMU pitch | IMU roll |
|---|---|---|
| Front stop (+16° protractor) | **+6.98°** | 0.73° |
| Back stop (−12° protractor) | **−22.59°** | 0.43° |

- **Pitch sign correct:** pitch increases when leaning forward. No change to `IMU_AXIS_SIGN` needed for pitch. The span is 29.6°, against 28° by protractor (close enough).
- **Level platform ≈ −9.8° IMU** (6.98 − 16 = −9.0; −22.59 + 12 = −10.6). The IMU is mounted pitched about 10° relative to the platforms, and roll is about 0.5–3°. Both are absorbed by θ_trim and harmless. Fix the mount when the permanent one goes on.
- **Tilt cut-outs in the IMU frame** (2° inside each stop): **trip if pitch > +5.0° or pitch < −20.6°**. The balance point θ_trim lies somewhere between, and is measured in 7.4.
- **Yaw sign correct:** `turn 90` (left) moved yaw from −1.3° to +93.2° (+94.5°; the excess is scrub on the training wheel).

### 5.4 Vibration (`pi/vibe_test.py`, stand, 1.0 A, 1/16, 4 s windows)

| Mode | rev/s | gx/gy/gz sd (°/s) | ax/ay/az sd (mg) | \|a\| range (g) | pitch pk-pk |
|---|---|---|---|---|---|
| StealthChop | 0 | 0.042 / 0.044 / 0.038 | 2.4 / 3.0 / 2.9 | 0.951–0.971 | 0.07° |
| StealthChop | 0.5 | 0.053 / 0.113 / 0.062 | 5.8 / 2.7 / 3.0 | 0.951–0.973 | 0.04° |
| StealthChop | 1 | 0.104 / 0.153 / 0.115 | 8.0 / 7.5 / 5.7 | 0.941–0.982 | 0.05° |
| StealthChop | 2 | 0.064 / 0.108 / 0.064 | 5.0 / 4.4 / 11.0 | 0.931–0.987 | 0.05° |
| StealthChop | 3 | 0.080 / 0.098 / 0.089 | 5.6 / 4.7 / 7.5 | 0.940–0.980 | 0.05° |
| SpreadCycle | 0 | 0.043 / 0.044 / 0.040 | 2.3 / 2.9 / 3.0 | 0.951–0.970 | 0.05° |
| SpreadCycle | 0.5 | 0.066 / 0.142 / 0.068 | 7.8 / 3.8 / 4.0 | 0.948–0.975 | 0.06° |
| SpreadCycle | 1 | 0.188 / 0.294 / 0.190 | 19.0 / 11.9 / 8.7 | 0.935–0.987 | 0.05° |
| SpreadCycle | 2 | 0.370 / 0.367 / 0.458 | 31.2 / 26.2 / 17.6 | 0.911–1.018 | 0.06° |
| SpreadCycle | 3 | 0.063 / 0.086 / 0.070 | 4.1 / 3.7 / 4.7 | 0.947–0.975 | 0.05° |

No IMU errors, stalls or accelerometer rejects; core 1 loop max 213–299 µs.

**Conclusions**
- **Vibration is not a problem.** The worst gyro noise is 0.46 °/s (SpreadCycle, 2 rev/s), within the 0.5 °/s target. **The pitch estimate stays within 0.04–0.07° peak-to-peak in every case**, so the filter rejects the vibration completely. The taped mount is adequate for now.
- **StealthChop shakes the IMU less** (gyro ≤ 0.15 °/s, accel ≤ 11 mg).
- **SpreadCycle mid-band resonance at about 1–2 rev/s (200–400 mm/s):** up to 31 mg and 0.46 °/s, and audibly rougher. **At 3 rev/s it is past the resonance:** the wheels were confirmed spinning, and it was audibly quieter than at 1–2 rev/s. StealthChop's smoother current waveform damps the resonance.
- The chopper choice for balancing will come from the Phase 7 step-loss tests (torque under sudden load changes), not from vibration. Balancing mostly runs well below 1 rev/s, with short faster bursts. StealthChop is preferred if it holds up under hard acceleration; SpreadCycle is acceptable otherwise.
- **5.6 DLPF:** the noise is low enough to try the wider filter (CONFIG 0x02, about 94 Hz) to cut about 2 ms of delay before balancing.
- **|a| at rest is 0.951–0.971 g** (it was 1.005 g on the bench at a different orientation). That points to an accelerometer zero offset of a few tens of mg on one axis, which is within the MPU6050 spec. The resulting fixed pitch bias is absorbed by θ_trim. Revisit with a 6-position accelerometer calibration only if it causes trouble (e.g. the accelerometer-rejection gate at ±0.15 g).

## 2026-11-22: Phase 6, telemetry and logging

**Built:**
- **Firmware:** core 1 pushes every IMU sample into a lock-free queue. Core 0 adds the wheel speeds and mode, keeps a 1 s live ring and a 2 s black box, and handles `LOG_START/STOP/STATUS` and `BB_FREEZE/ARM/DUMP`. ESTOP freezes the black box. Lines are written only when USB has room (non-blocking). Record `seq` is core 1's sample counter, so lost samples show as gaps. RAM use is 44.5% (from 4.6%).
- **Pi:** `robotd` writes `~/runs/<time>_<name>/data.csv` + `meta.json`. It adds `robotdctl` `log-start/log-stop/log-status/bb-freeze/bb-arm/bb-dump/runs`, auto-stop with `--duration`, and web `/api/runs` plus downloads.
- `pi/log_test.py`: the automated pass test.

**Pass test** (stand, 30 s, ±200 mm/s 0.25 Hz sine, 250 Hz logging):

| Check | Result |
|---|---|
| Rows | 7691 (expected about 7500) ✔ |
| Missing records | seq_gaps 0, stream drops 0, IMU queue drops 0, bad lines 0 ✔ |
| Time gaps | max 4029 µs against a 4000 µs period ✔ |
| Wheel speed follows command | peak 200 mm/s against 200 commanded ✔ |
| Black box | 1024 records, 2.05 s, consecutive ✔ |

**First attempt** reported 7272 IMU queue drops. They had built up at start-up, while core 0 waited for the USB host (the cable was moving from the Mac to the Pi) and nothing drained the queue. Fixed: `IMU_DROPS` now counts from `LOG_START` (with `IMU_DROPS_TOTAL` since boot), and `seq` comes from core 1, so a real drop also shows as a gap.

## 2026-11-23: Phase 7, limits, free fall and balance angle

### 7.1 Speed and acceleration limits (`limits_test.py`, stand, 1.0 A, 1/16)

| Mode | Top speed, clean | Stalled at | ±300 mm/s reversals (unloaded) |
|---|---|---|---|
| **StealthChop** | **800 mm/s** (4.1 rev/s) | 1000 mm/s | clean at 2, 5, 10, 20 m/s² |
| SpreadCycle | 600 mm/s | 800 mm/s | clean at 2, 5, 10, 20 m/s² |

- **Decision: StealthChop for balancing.** It has the higher top speed, and it vibrated less in 5.4.
- **Initial controller limits: v_max ≈ 550 mm/s** (70% of 800). **a_max ≈ 6 m/s²**, conservative because the reversal tests ran without the robot's load (1.88 kg at 6 m/s² needs about 0.18 N·m per motor). Verify both from the Phase 9 logs.
- Without encoders, stalls were judged by eye and ear. Small step losses short of a stall would not have been seen.

### 7.3 / 7.4 Free fall with the wheels held (`fall_test.py`, 6 falls, 500 Hz)

| Fall | Direction | Held angle | ω (rad/s) | θ_eq (°) | Fit rms (°) |
|---|---|---|---|---|---|
| 1 | forward | −1.42 | 8.35 | −1.61 | 0.024 |
| 2 | back | −4.81 | 5.68 | −5.76 | 0.009 |
| 3 | forward | −2.15 | 6.06 | −1.73 | 0.004 |
| 4 | back | −7.51 | 6.41 | −6.77 | 0.001 |
| 5 | forward | −3.32 | 5.55 | −2.29 | 0.023 |
| 6 | back | −4.74 | 5.34 | −5.44 | 0.013 |

**Balance angle: θ_trim ≈ −3.9° IMU pitch.** The apparent equilibrium depends on the fall direction: forward falls give −1.9° on average, back falls −6.0°. That's a band of about ±2.0°. With the wheels locked, the robot tips about the front or back edge of the tyres' flat contact patch, so each direction shows the CoM above a different patch edge. tan(2°) × 61 mm ≈ 2 mm half-patch, which is plausible for these tyres. When the wheels are driven they roll, so the balance point is the centre of the band: **−3.9°** (mean of the direction means). This needs confirming in Phase 9, where a constant drift in one direction means θ_trim is off.

**ω: the plan's 7.3 assumption was wrong.** With the wheels locked to the body, the robot doesn't pivot about the axle: it *rolls* (each radian of tilt rolls the axle r·θ forward). This is a rolling pendulum about the contact point, with

  ω_lock² = m·g·l / (J_axle + m·r·(2l + r))

rather than the driven-wheel model θ̈ = (g·θ − u)/l_eff. The swing-test numbers predict ω_lock ≈ 7.2 rad/s; measured 5.3–8.4 (mean 6.2, sd 1.1). Converting back, l_eff = g/ω_lock² − r(2l + r)/l gives **about 95 mm (swing test) to about 200 mm (falls)**. The locked-wheel test can't pin it down further: hand release, the patch-edge pivoting and the uncertain CoM height all contribute.

**Consequence for Phase 9:** design the first PD gains for the middle of the range, l_eff ≈ 140 mm, ω_n ≈ 14 rad/s, ζ = 0.7: **Kp ≈ 39 m/s²/rad (0.68/°), Kd ≈ 2.9 m/s² per rad/s.** The closed loop is stable for any l_eff when Kp > g:
- actual 95 mm → ω_n ≈ 17.6, ζ ≈ 0.88
- actual 200 mm → ω_n ≈ 12.1, ζ ≈ 0.61

The balancing logs (u, θ, θ̇) will then give l_eff directly by regression of θ̈ against (g·θ − u).

### Firmware / tools added

- `HOLD [LEASE=ms]` command: wheels locked (drivers on, no steps), time-limited (500–120000 ms), ended by STOP/ESTOP/motion. `MODE=HOLD` (telemetry mode 5). `robotdctl hold`.
- `pi/limits_test.py`, `pi/fall_test.py` (pure-Python fit, checked on synthetic falls: ω exact, θ_eq within 0.1°).

## 2026-11-23: Phases 8–9, safety layer and first balance

### Built

- **Firmware `Balance` module on core 1,** run straight after each 500 Hz IMU sample. u = KP·e + KD·θ̇ + KV·v + KX·x → v → x. Steps follow v immediately: a long interval is restarted rather than waited out, and a direction change restarts the state machines.
- **Safety:** an arming window (|e| ≤ 2°, |θ̇| ≤ 20 °/s) with soft start (0.3 s). Trips: tilt (+5.0° / −20.6° IMU), runaway (at VMAX for 0.3 s), IMU stale (20 ms) or invalid, and arming timeout. On a trip the steps stop at once, the drivers are released after 0.5 s, and the black box freezes with the reason. Dry-run mode. `MODE=BALANCE` locks out all other motion; STOP/ESTOP stop it.
- **Commands:** `BAL_SET/BAL_ARM [DRY=1]/BAL_STOP/BAL_STATUS`. Telemetry: mode 6, flags 0x20/0x40, u (mm/s²) and the commanded speed.
- **Pi:** `robotdctl bal-*`, `dryrun_check.py`, `balance_run.py`, `balance_tools.py`.

### 9.1 Dry-run sign check: PASS

12,679 samples over a 9.1° pitch range. The u-vs-error slope is **38.9 m/s²/rad against KP 39**, correlation 0.997. Leaning forward >2° gave mean u +2.61; leaning back gave −2.06.

### 9.2 Wheels in the air, 30% gains: direction correct

Acceleration was gentle, as designed (30% gains, 0.2 m/s cap).

### 9.3 First floor balance: SUCCESS

**First attempt (bug):** it ran with left-over stand-test parameters (KP 3.5, trim −10.9°, VMAX 0.2, ±30° tilt limits, 10° arming window). `balance_run.py` had applied its temporary settings before its restore safety net took effect, so an interrupted stand run left them behind and the next run scaled them again. The robot rocked between the training wheels for 27 s until TILT_FWD tripped at 19°. **Fixed:** the restore now covers every exit; each floor run resets the safety settings to their defaults and refuses KP < 15.

**Second attempt** (KP 39, KD 2.9, TRIM −3.9, defaults; run `2026-11-23_083314_balance`): **balanced for 64.6 s and was stopped by Ctrl-C.** Over 52.8 s of steady running (after the soft start):

| Quantity | Value |
|---|---|
| Pitch | mean **−3.904°** (trim −3.90), sd **0.116°**, peak-to-peak 0.55° |
| Wheel speed | mean 0.1 mm/s, sd 15.3 mm/s, range −45..+35 mm/s |
| Position | within **±7 mm** the whole time |
| u | sd 0.19 m/s², max 0.75 m/s² (of 6) |
| Pitch rate | sd 3.3 °/s |
| Oscillation | no dominant mode: weak spectral peaks at 1.8 Hz (0.03°) and 5.4 Hz (0.014°) |

- **The fall-test balance angle was right to 0.004°.** That's why PD alone held position (the expected drift didn't appear).
- **The gains designed from the model worked first time** (designed for l_eff = 140 mm, ω_n = 14 rad/s).
- The l_eff regression gave no result (r ≈ 0): too little excitation while it stood still. Repeat with deliberate small pushes.
- The visible "small oscillation" is mostly the base rocking by a few mm (±15 mm/s). The controller uses about 3% of its acceleration range.

## 2026-11-23: Push tests, full-state control and Phase 10

### Push tests with PD (KP 39)

| Run | KD | Result |
|---|---|---|
| `084134` | 2.9 | Caught every push (peak lean ≤ 1.8°, max u 2.3/6 m/s², max v 0.28/0.55 m/s, travel ≤ 20 cm, settled 0.3–1.3 s). l_eff regression ≈ 230 mm (r = 0.59). |
| `084515`, `084710` | 3.6 | **Rocking halved:** quiet-period pitch sd 0.04–0.05° (from 0.09–0.12°), wheel speed sd 5–7 mm/s (from 15). Both runs fell after many pushes: with no speed loop the robot wandered at up to ±0.47 m/s and drifted 0.7 m, so a push in the direction of travel hit the speed or acceleration limit. |

### Full-state control (pole placement)

Model θ̈ = (gθ − u)/l, ẍ = u, u = KP·e + KD·θ̇ + KV·v + KX·x gives
l·s⁴ + (KD − KV·l)·s³ + (KP − g − KX·l)·s² + KV·g·s + KX·g = 0 (KV, KX > 0 for stability). Designed for l = 0.20 m, poles at 10 rad/s (ζ 0.7) and 1.2 rad/s (ζ 0.9): **KP 36.7, KD 4.19, KV 4.81, KX 2.94**. Checked stable and reasonably damped for l = 0.12–0.30 m.

Run `085327`: it returned towards home after each push and held quiet-period pitch sd 0.05°, but it **settled further from home after each push, with a growing steady lean** (−0.44° → −0.88° → −1.2° at x = +90 → +208 → +267 mm). This matches KP·e = −KX·x exactly: the balance point had moved. **The IMU was shifting on its tape mount with each push** (confirmed; the prototype mount stays as it is).

### Phase 10 firmware

- **Automatic trim:** trim_est −= TRIM_RATE·(x − x_ref)·dt, within ±4°, only while calm (|v − v_ref| < 0.05 m/s and |e| < 1°).
- **Driving while balancing:** SET_VELOCITY in BALANCE mode sets v_ref / w_ref (lease, ramped); x_ref integrates v_ref; separate step generators per wheel for turning. Web drive buttons capped at 250 mm/s and 60 °/s; releasing sends speed 0 (keeps balancing).
- The full-state gains became the defaults.

### Accelerometer compensation (plan 5.5): decisive

- Run `102354` (no compensation): returned home after every push, but **ran away** after the hardest. In the black box the wheels accelerate forward to VMAX while the *measured* pitch swings back 2°, then the robot topples forward. Cause: at 2 m/s² the accelerometer reads an 11° backward tilt, so the complementary filter (τ 0.5 s) drifts the estimate backwards during sustained accelerations and the controller under-reacts. l_eff regression gave nonsense (2.3 m), consistent with a biased angle.
- Run `102857` (**ACCEL_COMP=1**, TRIM_RATE 0.25): **recovered from backward leans of 9.2° and 9.9°** (it fell at about 3° before). Returned home within about 2 s every time (pushed out to ≤ 250 mm). The user felt the robot "push back". It ended on a forward lean with the wheels pinned at VMAX 0.55 m/s while the robot did *not* tip back, so most likely a sustained push from behind: the speed limit is now the constraint for forward disturbances.
- **ACCEL_COMP now defaults on**, and the trim rate defaults to 0.25 (calm-only).

### Web drive test 1: robot dropped by a STOP (safety fix)

Run `103528`: it balanced calmly for 28 s, then **balancing ended while upright** (−3.9°, wheels −11 mm/s) and the robot fell right over (84° pitch). Cause: `robotd` sends STOP when a web drive hold (600 ms) expires, and STOP ended balancing. Afterwards, web drive commands ran the motors in **normal velocity mode** (mode 1 bursts from 41 s to 60 s) while the robot lay on its side.

**Fixes:**
- STOP while balancing now means "stop moving, keep balancing" (drive set points to zero). `BAL_STOP`/ESTOP end balancing; STOP during arming cancels.
- **Trip lockout:** after a TILT/RUNAWAY/IMU/ARM_TIMEOUT trip, all motion commands are refused with `BALANCE_TRIPPED` until STOP, CLEAR_ESTOP or BAL_ARM. (Tightened after test 2: see below.)
- `balance_run.py` ends any leftover balancing with BAL_STOP before starting.

**Open points:** the VMAX headroom (0.55 m/s against 0.8 m/s stall unloaded; consider 0.65 after loaded tests); the l_eff regression needs a gyro-integrated angle; drive test from the web page still to do.

### Web drive test 2 (run `104319`, black box `104648_bb_command`)

- **Balanced for about 189 s** (best so far). It was ended by the operator's Ctrl-C (`BAL_STOP`, reason COMMAND, black box clean); the robot then dropped onto a training wheel (pitch +29°) as designed.
- **Turns worked:** four turns of about 1–2 s at the 60 °/s web cap (wheel speeds ∓117 mm/s). Yaw changed 108° over the run.
- **Forward/back did nothing:** the controller never saw a linear set point (no 1 s window with mean wheel speed above 15 mm/s; the model predicts about 0.4 m for a 1.5 s press). The same happened in test 1. The only firmware path that drops a linear command but passes a 60 °/s turn is the range check: `SET_VELOCITY V=400` (the web slider's default) is rejected with `OUT_OF_RANGE` unless the page clamps it to 250, which needs the page's status to read `MODE=BALANCE`. **Fix:** in balance mode, `SET_VELOCITY` is now clamped to ±300 mm/s and ±90 °/s instead of rejected, so it doesn't depend on any client.
- **Motors ran after the fall:** web drive commands drove the fallen robot in normal velocity mode (mode 1 bursts, 201–203 s). A COMMAND end didn't set the lockout, and balance_run's STOP or robotd's hold-expiry STOP would have cleared it anyway. **Fix:** *every* end of balancing sets the drive lockout. STOP no longer clears it; only CLEAR_ESTOP or the next BAL_ARM do. `STATUS` reports `DRIVE_LOCK`, and the web page shows its Clear button while locked.
- **Slow oscillation built up:** from about 150 s, a steady ~1 Hz oscillation in the black box: wheel speed ±20 mm/s, pitch ±0.35°, u ±0.5 m/s². Early in the run the wheel-speed sd was 3 mm/s; by the end it was 13 mm/s. The pitch it balanced at moved in steps (−3.8° → −2.0° → −0.5° → −2.4°), which is the IMU shifting on its tape again. To watch in the next runs (IMU mount, battery level).
- **Logging:** at 500 Hz the Pico drops about 40 % of telemetry lines (`STREAM_DROPS`; all balance runs so far, the USB/Pi side can't keep up). The balance itself is unaffected. `balance_run.py` now logs at 250 Hz by default (`--div`).

### Web drive test 3 (run `110119`, 250 Hz, no lost lines)

- **Drove on command:** forward and back reach the 250 mm/s web limit, leaning about 3° to accelerate, and come back to rest. Turns at 60 °/s: the gyro yaw rate matched the commanded rate within 1–2 % in every turn, so **no motor stalled** (a stalled wheel would halve the yaw rate). Balanced 268 s; ended by Ctrl-C.
- **"A motor stopped" during turns was the controller, not a connection:** in every turn the robot's forward speed drifted to about ±115 mm/s, i.e. half the turn's wheel-speed difference, so the inner wheel was commanded to almost zero and the robot pivoted on it. Left turns drifted forward, right turns back. Cause: while turning, the gyro's pitch axis picks up 1.77 % of the yaw rate (q = −0.0177·r, both directions; the gyro axis is about 1° off horizontal). The attitude filter corrected for this with the *estimated* roll, but that estimate swings ±2° during turns (accelerometer offsets and the centripetal pull on the off-centre IMU), so it added up to 3.8 °/s of false pitch rate. The pitch estimate then went about 1° wrong in every turn and the controller drove to "correct" it. **Fix:** the filter and the controller use θ̇ = q − c·r, with c learned while turning (mean q/r, default −0.0177, reported as `GYRO_QR`).
- **Slow to get going:** the speed ramps over 2.5–4 s although v_ref reaches 250 mm/s in 0.5 s. That's the slow outer (speed and position) loop: its poles were placed at 1.2 rad/s, i.e. a time constant of about 1 s. Faster options: outer poles at 1.6 rad/s (KP 39.4, KD 4.70, KV 6.60, KX 5.22; settable live with `BAL_SET`), at the cost of damping if l_eff is really above 0.3 m; or an acceleration feed-forward in the drive set point.
- **Small oscillation at rest:** pitch sd 0.15–0.21°, wheel speed sd 4–5 mm/s, peak at 2.6–2.9 Hz. Acceptable.

### Drive test 4 (run `111831`): turns fixed; forward/back still slow

- **Turns:** after the first turn (which started while the robot was still catching up after a forward drive), the forward speed stayed within ±25 mm/s during turns, against ±115 mm/s before the gyro fix. Balanced 209 s.
- **Why forward/back is slow:** the lean comes quickly (pitch +2° within 0.6 s), but the robot then **holds a 2.5° forward lean (3.5° back) while accelerating at only about 0.13 m/s²**. The gyro-integrated angle confirms the lean is real. A frictionless pendulum would need only 0.8° for that. The extra steady lean (about 1.5–2.5°) is what the motors' reaction torque needs to push against rolling drag. In the full-state controller that lean can only come from speed and position errors, so the robot falls about 0.3 m behind its set point and creeps up to speed over 2.5–4 s. (The same lag explains the first turn: the robot was still catching up when the turn began.)
- **Fix: drive feed-forward.** The drive set point is now an S-curve (acceleration a_ref, jerk-limited), and the controller targets a lean of a_ref/g + DRIVE_LEAN·tanh(v_ref/0.05) and adds a_ref to u. In the model the robot reaches 250 mm/s in about 0.5 s instead of 1.5 s. `DRIVE_FF` (0 = old behaviour) and `DRIVE_LEAN` (default 1.5°) are settable live.
- **The IMU (or the battery) is still moving:** the balance pitch went from −3.6° to +1.1° during this run, and resting on the back training wheel now reads −18.5° (−22.6° when the tilt limits were set). The tilt trips are absolute IMU angles, so forward trips could now come early. `balance_run.py` now warns when the back-stop reading is more than 2° off. Securing the IMU and battery, then re-measuring both stops, is overdue.

### Drive feed-forward tests (runs `113258`, `113637`)

- With the feed-forward, the robot leans almost at once (about 0.2 s) and reaches 200 mm/s in about 1.3 s, against 2.5–4 s before. The user preferred **DRIVE_LEAN 2.0° and DRIVE_ACCEL 0.8 m/s²; these are now the defaults.**
- Run `113258` ended on **TILT_FWD** at the start of a forward drive. The balance pitch had drifted up to about −0.4° (the IMU moving again), and the forward lean of a_ref/g + DRIVE_LEAN (up to about 6.7° at 0.8 m/s²) crossed the absolute 5° limit. With the IMU back at its original position (balance −3.9°) the same lean peaks near +2.8°, inside the limit. Securing the IMU is now needed before driving harder.
- Two quick attempts to get l_eff from these drive logs (θ̈ regression with a fitted offset; short-horizon gyro prediction) gave no usable answer (r² < 0.1): in closed loop, u and θ move together and the rolling-drag lean biases the offset. l_eff needs a dedicated test.

### Phase 9.4 completion: built (23 Nov)

- **IMU filter 94 Hz** (CONFIG 0x02, about 3 ms delay instead of 4.9 ms). To compare with the 44 Hz quiet-standing figures above (pitch sd 0.15–0.21°, wheel speed sd 4–5 mm/s, peak 2.6–2.9 Hz).
- **Kick test for l_eff:** `BAL_KICK U= MS=` adds a short acceleration pulse to the controller output (telemetry flag 0x80 while balancing). `balance_run.py --kicks N` fires alternating ±2 m/s², 40 ms kicks when calm and computes l = ∫(g(θ − θ₀) − u)dt / Δq per kick. Checked on simulated closed-loop data at 250 Hz: l = 0.20 m recovered as 0.207 m (4 ms actuation delay) and 0.35 m as 0.355 m, spread ±4%.
- The user has wedged the IMU between the top platform and the battery. The trim and both tilt limits need re-measuring before the next balance run.

### IMU wedged: new trim and limits; 94 Hz filter result (run `115606`)

- **Stops re-measured:** front +12.72°, back −16.68° (span 29.4°, was 29.6°). The IMU has turned +5.8° relative to the old mounting. **New defaults: TRIM +1.9°** (same offsets from the stops as before: 12.72 − 10.88 = 1.84; −16.68 + 18.69 = 2.01), **TILT_FWD +10.7°, TILT_BACK −14.7°**; balance_run's back-stop reference −16.7°.
- **The wedged IMU holds still:** the balance pitch moved only 0.13 → 0.17° over 100 s (it drifted up to 5° per run on tape).
- **Run `115606` (old trim −3.9°):** it balanced for 119 s but sat 0.89 m behind home with e ≈ +4°: KP·e = −KX·x exactly. The automatic trim never corrected it, because its calm gate also required |e| < 1°. **Fix:** the gate is now |v − v_ref| < 0.05 m/s and |θ̇| < 5 °/s, so a large trim error does get corrected.
- **No kicks fired** for the same reason: balance_run waited for |err| < 1.5°. Its gate is now |θ̇| < 10 °/s and |v| < 0.1 m/s.
- **94 Hz filter (5.6): quieter.** Over ten quiet 10 s windows: pitch sd **0.07–0.12°** (0.15–0.21° at 44 Hz), wheel-speed sd 2.7–5.8 mm/s (4–5), oscillation peak 2.5–3.8 Hz (2.6–2.9). The new mount probably helps too. **5.6 closed: CONFIG 0x02 (94 Hz) kept.**

### 9.4b Kick test: l_eff measured (run `124753`)

New trim and limits: it held its place (42 s, ended by Ctrl-C after the kicks). Ten alternating ±2 m/s², 40 ms kicks (mean u during each kick 1.5–1.8 m/s², because the controller pushes back):

| | Forward kicks | Backward kicks | All |
|---|---|---|---|
| Δq | −33 to −45 °/s | +44 to +54 °/s | |
| l_eff | 74–80 mm | 57–77 mm | **median 74.5 mm (57–80, within ±20 %: pass)** |

- **l_eff ≈ 75 mm** (ω₀ = √(g/l) ≈ 11.4 rad/s), a little below the swing-test value (95 mm) and far below the 200 mm the gains were designed for. The θ̈ regression in the same run gave 1.6 m (r = 0.2), confirming that the regression is useless and the kick is the method to use.
- **What that means for the current gains** (KP 36.7, KD 4.19, KV 4.81, KX 2.94; designed for l = 0.20 m): at l = 75 mm the fast poles are real (−5.6 and −43 rad/s): heavily damped, but with a large KD, so the loop is noisy and sensitive to delay. With 8–12 ms of loop delay a 9–15 Hz mode appears with damping down to 0.34 (if l is really 60 mm). The slow (speed/position) poles sit at 1.1 rad/s.
- **Redesign for l = 75 mm** (fast pair 14 rad/s ζ 0.7, slow pair 2.0 rad/s ζ 0.9): **KP 30.55, KD 2.19, KV 5.99, KX 5.99.** Checked in discrete time (500 Hz) with 8 and 12 ms of delay for l = 60–90 mm: all stable, minimum damping 0.48–0.89, no high-frequency mode. The slow poles are 1.6× faster, so drive and position response should also be quicker.

### Redesigned gains: unstable (run `125339`, black box `125346_bb_tilt_fwd`)

- With KP 30.55, KD 2.19, KV 5.99, KX 5.99 it oscillated at **about 2.8 Hz (17.5 rad/s), growing from ±1° to ±3°** within 2 s, and tripped TILT_FWD. The model said this set was well damped.
- 2.8 Hz is the same frequency as the small oscillation at rest with the old gains (2.5–3.8 Hz). So there is a lightly damped mode there that the model doesn't contain, and the old, large KD (4.19) is what damps it.
- **A better model fit:** fitting each of the ten kicks with a delay, θ̈ = (g(θ − θ₀) − u(t − d))/l, over 120 ms gives **l ≈ 90 mm (75–105) and d ≈ 12 ms** (from logged u to measured q), residual 4–11 °/s. That model still predicts the new gains are stable (damping 0.48 at 12 ms), so the missing piece isn't pure delay. Candidates: compliance in the IMU wedge, the tyres or the stepper load angle; the step generator's interval-update lag at low speed.
- **Decision:** keep the original fast-loop gains (KP 36.7, KD 4.19). l_eff is recorded as 75–90 mm. Any later gain change keeps KD at about 4 unless a test shows otherwise. A safer way to get a quicker speed/position response is to change only the outer gains: KV 6.0, KX 5.0 puts the slow poles at 1.8 rad/s (from 1.25), and the model shows the fast-loop damping almost unchanged.

### Outer gains KV 6.0, KX 5.0 (run `130117`) and 9.4 closed

- Run `125814` (old gains) tripped TILT_FWD at +5.0° when driving forward: the Pi still had the old balance_run defaults and had reset the cut-outs to +5.0 / −20.6. Fixed by deploying. (Run `125339`, the unstable redesign, had the same stale cut-out; its 2.8 Hz oscillation was still real.)
- With KV 6.0 and KX 5.0 it balanced 109 s and drove well: 30 → 200 mm/s in 1.1–1.5 s forward and 2.2 s back. That's about the same as with the old outer gains plus the feed-forward (about 1.3 s).
- But the 2.8 Hz wobble at rest was larger and **grew during the run**: pitch sd 0.11 → 0.27° over 100 s, against 0.07–0.12° (steady) with the old gains in run `115606` and 0.12–0.14° in `125814`.
- **Decision: keep the original gains (KP 36.7, KD 4.19, KV 4.81, KX 2.94).** The feed-forward already gives the drive response, and the larger outer gains make the 2.8 Hz mode worse.

**Phase 9.4 closed:** 94 Hz IMU filter; l_eff 75–90 mm with about 12 ms of loop delay (kick test); trim and cut-outs re-measured for the wedged IMU (+1.9°, +10.7° / −14.7°); gains kept. **Open item for Phase 11:** the lightly damped ~2.8 Hz mode, which isn't in the model (KD ≈ 4 damps it; smaller KD or larger KV/KX excite it). A model for LQR needs it.

## 2026-11-23: Phase 11.0, the lightly damped mode

- **Ringing after the kicks in run `124753`** (damped sine fitted to q from 0.12 to 1.5 s after each kick): **2.18–2.32 Hz, damping 0.27 at the first kick falling steadily to 0.13 at the tenth** (30 s later). The fit residual is about 3.5 °/s against a 13 °/s ring amplitude. The model (l = 90 mm, 12 ms delay, current gains) predicts no lightly damped mode at all. The falling damping matches the wobble growing during longer runs (`104319`, `130117`).
- `balance_run.py --kicks` now reports the ring frequency and damping per kick and as medians (pure-Python grid fit; same results as the SciPy fit on `124753`).

### 11.0 driver tests, first set (runs `140536`, `141019`, `141148`, `141645`, `141826`)

| Run | Drivers (status at start) | Result | Ring (median) | Damping, kick 1 → kick 10 |
|---|---|---|---|---|
| `140536` A | StealthChop 1.0 A | steady, 41 s | 2.15 Hz | 0.32 → 0.20 |
| `141019` B | SpreadCycle 1.0 A | **ran backwards; RUNAWAY at 33 s** (an earlier attempt also ran off) | 0.9–2.1 Hz, erratic | — |
| `141148` "C" | SpreadCycle 1.0 A (the 1.3 A StealthChop `configure` hadn't taken) | **fell back at 11 s** | — | — |
| `141645` | StealthChop 1.0 A | steady, 47 s | 2.29 Hz | 0.31 → 0.13 |
| `141826` | StealthChop 1.0 A | steady, 43 s | 2.23 Hz | 0.24 → 0.17 |

- **SpreadCycle: steady at rest, but a kick can start a fast oscillation that doesn't die out.** Between kicks it balanced as well as StealthChop (pitch sd 0.2–0.4°, x within a few mm). In `141019` (kick 7) and `141148` (kick 1), a forward kick set off a **violent oscillation at about 8 Hz** (q ±60–90 °/s, u swinging ±4–6 m/s², accelerometer corrections rejected). It kept going, the robot crept and then ran backwards, and it ended on RUNAWAY or a tilt trip within 1–2 s. The spoiled first attempt (`140745`) also went at a kick. StealthChop never did this in six runs (60 kicks). This is a different, faster effect from the 2.2 Hz ring. **The motor drivers are part of the plant.**
- **StealthChop repeats closely:** l_eff 75–77 mm, ring 2.15–2.29 Hz. In every run the **damping starts at about 0.25–0.31 and falls to about 0.13–0.17 within 30 s, while the ring frequency rises** (2.12 → 2.36 Hz). It resets each time balancing is re-armed. Between kicks the robot is steady (pitch 1.92–2.01°, x within 1 cm), but its quiet rocking grows from about 1–2 to 4–5 mm/s over the run.
- **What resets on each arming:** the drivers are disabled and re-enabled (StealthChop's automatic PWM regulation then restarts from its initial values and re-adapts while running), and the controller states (trim estimate, x, v) reset. The trim estimate and x barely move, which points at the drivers.
- Run `142313` (labelled C) was a fourth StealthChop 1.0 A baseline: `configure 1300` was refused with DRIVER_FAULT. Same pattern: l_eff 71 mm, ring 2.25 Hz, damping 0.33 → 0.14.
- **Why configure 1300 failed (both times):** the firmware checks the current read back from the driver against the request with a fixed 25 mA tolerance. The driver sets current in steps of about 55 mA, so 1300 mA reads back more than 25 mA away. The tolerance is now max(25 mA, 1/16 of the request).
- **Run `142738` (StealthChop 1.3 A, confirmed at start):** steady for the full 60 s. The summary file is missing because the SKR was power-cycled while the script was finishing; analysed afterwards from data.csv. l_eff 81 mm (58–133), **ring 2.31 Hz, damping 0.17 → 0.09–0.10**: lower from the first kick than at 1.0 A (0.24–0.33), and still falling.
- **So more current means less damping and a slightly higher ring frequency.** That fits the mode coming from the motor/driver link (a stiffer magnetic "spring" between commanded and real wheel position, with little damping), not from the IMU mount or the chassis. Caveat: this run came after many others, so the drivers were warmer, and temperature and time-in-run aren't separated yet.
- **Long run `143358` (StealthChop 1.0 A, 40 kicks over 136 s, after run C and a few minutes' rest):** l_eff 73 mm (56–94). **Ring 2.4 Hz with damping 0.05–0.09 from the first kick to the last.** No further decline and no instability; the ring amplitude stayed at 10–14 °/s. But this time the damping was low *from the start*, unlike runs A–A2 (0.24–0.33 at the first kick), and the frequency was higher (2.4 against 2.15–2.3 Hz).
- **So "resets on re-arming" isn't the whole story.** The robot starts in the "good" state (damping about 0.3, 2.15 Hz) and drifts towards a "worn" state (about 0.06, 2.4 Hz) within about 30 s. A re-arm after a short rest recovered it early in the session, but not after the 1.3 A run. Something with a time constant of seconds to minutes changes: **motor/driver temperature or battery voltage** (StealthChop drives voltage, not current, so both shift its dynamics). The battery had been in use for several hours by this run.
- **Readings straight after the long run:** motors 28 °C (about room temperature), driver heatsink 35 °C (50 °C after the Phase 2 heat run), battery 14.97 V resting (about 3.74 V per cell, roughly half charged; 16.8 V full). Temperature is an unlikely cause: the motors are barely warm. **Battery voltage is now the main suspect.** A partly discharged LiPo also sags under load and recovers at rest, which would fit the damping falling during a run and partly recovering between runs.
- **Next session:** start with a full battery and cold motors; log the battery voltage at the start and end, and the motor and driver temperatures (IR thermometer) before and after. Run kicks straight away and keep going for 3 minutes. If the damping starts at about 0.3 again and falls as the motors warm, it's temperature. If it starts higher with a fresh battery and falls as the voltage sags, it's the battery.

### Cold start with a full battery (run `161333`, started from the web page)

- **Conditions:** battery 16.78 V (just charged); motors 25.6 °C and driver heatsink 31 °C at the start; 31.6 °C and 47 °C at the end. 130 kicks over 8.6 min of balancing.
- **Result:** l_eff 70 mm (48–135). **Ring 2.36 Hz, damping 0.06 from the first ten kicks, falling slowly to 0.04 by the last ten.** So it was low from the start, with a fresh battery and cold motors.
- **Neither battery voltage nor temperature explains it.** The high-damping runs (0.25–0.33 at the start, `124753`–`142313`) all came before the SKR was **power-cycled at about 14:27**. Every run since (`142738` at 1.3 A, `143358`, `161333`) started low. Earlier changes of firmware were by reflashing over USB, which resets the RP2040 but **leaves the TMC2209s powered**.
- **Hypothesis: StealthChop's automatic tuning.** The driver learns its PWM amplitude and gradient (PWM_OFS_AUTO, PWM_GRAD_AUTO) while it runs, especially at steady medium speeds. The learned values are kept until the driver loses power. They had been learned all morning in the heat run, vibration test, limit tests and drives. After a power cycle they restart from the untuned initial values, and balancing (short, slow, reversing moves) hardly retunes them. The firmware sets only toff, current, microsteps, the chopper mode and pwm_autoscale; the rest of PWMCONF stays at power-on defaults.
- **Test:** on the stand, run both wheels steadily at about 300 mm/s for a minute (StealthChop tuning needs steady motion), then repeat a short kick run. If the damping is back near 0.3, the hypothesis holds, and the fix is to configure and check the StealthChop settings explicitly at boot.

- **Test result (run `163113`): confirmed.** After one minute of steady running on the stand at 300 mm/s (no power cycle in between), a 15-kick run gave **damping 0.50 at the first kick, 0.28–0.29 next, then falling to about 0.15 by kick 15; ring 2.2 Hz**, l_eff 72 mm. That is the morning's "good" behaviour again, and it was visibly steadier. So **StealthChop's automatic tuning, learned in steady motion, sets the damping of the 2.2 Hz mode**, and balancing's short reversing moves slowly detune it again (the decline within each run).
- **Fix to build:** read the drivers' learned PWM values (PWM_AUTO: PWM_OFS_AUTO, PWM_GRAD_AUTO; PWM_SCALE) after a tuning run, then set them as fixed values with automatic gradient tuning off (pwm_autograd = 0) and write the whole PWMCONF at boot, so the drivers behave the same after every power-up and don't drift while balancing.

### StealthChop PWM read-outs (TMC_PWM) and what moves

| When | GRAD_AUTO (X/Y) | OFS_AUTO (X/Y) | Damping in the next kick run |
|---|---|---|---|
| After power-up | 14 / 14 | 36 / 36 | (`161333`: 0.06 from the start) |
| After 1 min at 300 mm/s on the stand | 20 / 20 | 36 / 36 | `164338`: 0.31–0.35 at the start, falling to 0.12 |
| After that balancing run | 24 / 26 | **110 / 113** | — |
| GRAD fixed at 20, AUTOGRAD 0 (no power cycle, so OFS_AUTO still about 110) | 20 / 20 | (not read) | `164939`: 0.13 → 0.07–0.10 |

- **The learned amplitude OFS_AUTO is the main mover.** While balancing, the motors spend most of their time near standstill, and StealthChop's automatic scaling raises OFS_AUTO (36 → about 110) until the full run current flows at low speed. More low-speed current means a stiffer, less damped motor link. That is the same effect as the 1.3 A run, and it explains the damping falling during every run. After a power-up the driver tunes OFS at standstill straight away, so balancing started in the "stiff" state. After the stand run the motors had just been running at speed, where OFS stays at its starting value.
- **Next test: forward-controlled StealthChop** (`AUTOSCALE=0`, OFS 36, GRAD 20), so the amplitude stays fixed. Web runs now record the drivers' PWM state at the start and end in `balance_summary.json`.
- **Forward-controlled test (run `165859`, after a power cycle; AUTOSCALE 0, OFS 36, GRAD 20; recorded unchanged at the start and end): not better.** Ring 2.54 Hz, damping 0.14 → 0.07 over 15 kicks, l_eff 86 mm. The damping still fell during the run with the PWM amplitude held fixed, so **OFS_AUTO climbing is not the cause of the decline either**. Forward mode also raised the ring frequency (2.5 against 2.2 Hz).
- **What is solid so far:** (1) one minute of steady running on the stand, with automatic scaling on, gives a well-damped start (0.3–0.5) in the next run (`163113`, `164338`). (2) In every mode the damping then falls over about 30–60 s, then settles low (0.04–0.06 over 8 minutes in `161333`). (3) SpreadCycle is far worse. The mechanism of (1) and (2) isn't identified yet. The steady, settled state (auto scaling on, after about a minute of balancing) is at least repeatable, which is what a controller comparison needs.

### KD 4.6 in the settled state (runs `170543`, `170714`)

- Power cycle, about 1 min of balancing (driver OFS_AUTO 36 → 111/112, GRAD_AUTO 14 → 16), then KD 4.19 → 4.6 and a new run with 15 kicks. The new run started with the drivers already settled (OFS_AUTO 111/112) and they stayed there.
- **Damping 0.13 → 0.075 (median 0.10), ring 2.28 Hz, l_eff 73 mm.** That's slightly better than the settled KD 4.19 runs (0.06 in `161333`; 0.13 → 0.07 in `164939`), but within the scatter, so it's inconclusive.
- **The damping fell during this run too, with the driver state constant.** So the decline is tied to time since arming, not to the drivers. Between kicks the quiet rocking grows over the run. One reading: the mode is lightly damped and a small self-sustained wobble (limit cycle) builds up after each arming, adding to each kick's ring and making it look less damped. To check next session: fit the ring with a decay towards a steady wobble, and look at the quiet wobble on its own.
- The web page can now change the number of kicks during a run.

### Wobble analysis: is the falling damping real? (no new runs)

Method (12 kick runs): (a) **quiet wobble** = RMS of the 1.2–4 Hz band-passed pitch rate between kicks; (b) **ensemble average** of the kick responses, aligned to each kick and sign-normalised, for the first five and last five kicks. A random-phase background wobble averages out of (b), so its damping is free of the pollution suggested above.

| Run | Quiet wobble (°/s rms), start → end | Single-kick damping, first 5 → last 5 | Ensemble damping, first 5 → last 5 (all) | Ring |
|---|---|---|---|---|
| `140536` A | 0.52 → 1.25 | 0.27 → 0.19 | 0.27 → 0.20 (0.24) | 2.15 Hz |
| `141645` | 0.53 → 1.34 | 0.24 → 0.16 | 0.24 → 0.17 (0.20) | 2.24 Hz |
| `141826` | 0.90 → 1.52 | 0.24 → 0.18 | 0.23 → 0.18 (0.20) | 2.21 Hz |
| `142313` | 0.81 → 1.18 | 0.21 → 0.17 | 0.22 → 0.17 (0.19) | 2.23 Hz |
| `142738` 1.3 A | 1.08 → 1.35 | 0.16 → 0.12 | 0.16 → 0.13 (0.15) | 2.29 Hz |
| `143358` long | 1.39 → 2.96 | 0.10 → 0.07 | 0.10 → 0.07 (0.09) | 2.37 Hz |
| `161333` cold, full battery | 3.05 → 4.46 | 0.07 → 0.05 | 0.09 → 0.06 (0.08) | 2.35 Hz |
| `163113` after stand run | 0.77 → 0.88 | 0.27 → 0.17 | 0.27 → 0.17 (0.21) | 2.21 Hz |
| `164338` after stand run | 0.63 → 0.98 | 0.24 → 0.15 | 0.25 → 0.15 (0.19) | 2.23 Hz |
| `164939` GRAD fixed | 1.65 → 2.43 | 0.13 → 0.11 | 0.14 → 0.11 (0.12) | 2.30 Hz |
| `165859` forward mode | 1.98 → 2.55 | 0.18 → 0.15 | 0.18 → 0.15 (0.16) | 2.43 Hz |
| `170714` KD 4.6 | 0.56 → 1.19 | 0.15 → 0.11 | 0.15 → 0.11 (0.12) | 2.27 Hz |

(SciPy fits; the Pi's grid fit gives somewhat lower damping for some runs, but the same trends.)

- **The falling damping is real.** The ensemble averages fall exactly as the single-kick fits do, so the background wobble isn't fooling the measurement. The quiet wobble grows with it, as you'd expect if the damping really falls; in long runs both settle after about 1 minute (wobble about 3 °/s in `143358`, about 4 °/s in `161333`).
- **There are two levels:** about 0.2 (early session, after a stand run) and about 0.08–0.12 (after a power cycle, or settled). Within every run the damping falls by about 25–40 % over the first minute.
- **Ruled out so far:** battery voltage, motor/driver temperature, StealthChop OFS_AUTO and GRAD_AUTO (forward mode with fixed PWM still falls), background-wobble pollution.
- **New candidates:** something mechanical that changes while the robot stands nearly still on the floor and recovers when the wheels turn a lot. Examples: the tyres taking a set (flat spot) where they rest, or bearing/hub friction. The stand run turns the wheels many times; the power-up cases had the robot resting on its training wheel. **Discriminating tests:** (1) after a power cycle, turn the wheels by hand (motors off) or drive a few metres, then do a kick run. If it's well damped, the cause is mechanical, not the drivers. (2) Partway through a run, drive about 1 m forward and back, then continue kicking. If the damping recovers, it depends on the wheel position or contact patch.

- **Test 1 result (run `171913`): mechanical.** After a power cycle (drivers untuned: OFS_AUTO 36, GRAD_AUTO 14 at the start, the same as `161333`), with both wheels turned several revolutions by hand with the motors off: **ensemble damping 0.29 → 0.20 (all 0.23), ring 2.16 Hz**, quiet wobble 0.84 → 1.03 °/s, l_eff 72 mm. That's the "good" level, against 0.08 for `161333` with the same driver state but no wheel turning. **So the state that sets the damping is mechanical: wheels, tyres or bearings, not the drivers.** Test 2 (driving partway through a run) will show whether it depends on where the tyres rest.

- **Test 2 result (run `172551`): not where the tyres rest.** The run started well damped. At 50–66 s it drove 0.73 m out and back (up to 290 mm/s, about 3.7 wheel turns under power). Ensemble damping: 0.30 (kicks 1–5) → 0.23 (kicks 6–9, before the drive) → **0.23 just after the drive** → 0.17 (last four). The drive didn't restore anything, so a flat spot or set at the resting position is unlikely. The difference from test 1: there the robot was lifted and handled and the motors were unpowered while the wheels turned. **Next suspects:** something that lifting/handling resets and the robot's rocking slowly changes. For example: the battery or IMU settling on its mount, a cable, or the rotors' rest position against the magnetic detents when unpowered.
- The 1.2–4 Hz wobble during the drive (3.3 °/s RMS) was not smaller than while standing (1.9 °/s), so the mode isn't only a standing-still effect.

### Handling tests (runs `173835`–`174503`)

| Run | What was done just before | Damping, first kick → last (median) | Ring |
|---|---|---|---|
| `173835` A | Power cycle, no handling | 0.08 → 0.06, flat (0.073) | 2.40 Hz |
| `174056` B | Robot lifted for 10 s, wheels kept still | 0.32 → 0.10 (0.125) | 2.28 Hz |
| `174254` C | Battery pressed and wiggled, no lift | 0.23 → 0.15 (0.157) | 2.30 Hz |
| `174503` D | Motor mounts wiggled | 0.18 → 0.10 (0.112) | 2.32 Hz |

- **Every mechanical disturbance raises the damping for a while, and it then decays** over about a minute towards the settled value (about 0.07). No single part stands out: lifting, the battery and the motor mounts all help partly. Driving under power doesn't help (test 2).
- **Interpretation:** this behaves like **friction damping in the structure's joints.** When disturbed, a joint (screws, strap, standoffs, hub grub screws, the IMU wedge) can micro-slip and dissipate energy. While the robot rocks gently in place it beds in and stops slipping, so the damping falls. This is a common effect in bolted structures. It can't be fixed in software; it can only be made consistent.
- **Practical decision:** treat the settled state (damping about 0.07, ring about 2.35 Hz) as the design condition for Phase 11. It's repeatable and the worst case, so a controller that handles it handles the rest. When convenient, check and tighten the mechanical joints (plan items 1.3 and 1.4: battery strap, IMU fixed with screws, motor mounts, hub grub screws). That could raise the settled damping or at least make it more consistent.

### Phase 11.1–11.3: model, LQR steps, filter fix, drive tests (evening 23 Nov)

Full details, tables and run numbers: [phase11-model.md](phase11-model.md). In short:

- **Model (11.1):** a pendulum plus about 10 ms delay plus a lightly damped joint pole/zero pair, fitted to the kick ensembles. It reproduces the 2.37 Hz ring and its damping in four runs, and predicted that the earlier redesign would be unstable.
- **Stepping to the LQR gains (11.2):** five steps from KP 36.7/KD 4.19/KV 4.81/KX 2.94 to KP 73.5/KD 4.63/KV 10/KX 11.8.
  - **Steps 1–2:** better than predicted. Step 2 gave a ring of 2.88 Hz, ζ 0.30–0.34.
  - **Steps 3–5:** a new, almost undamped mode at 4.4 → 4.8 → 5.15 Hz.
- **Cause and fix (11.3):** the mode came from the IMU complementary filter's accelerometer path.
  - **Motors ruled out:** 1.3 A didn't move it.
  - **Correction off:** turning ACCEL_COMP off removed it.
  - **Fix:** raising the filter time constant from 0.5 to 2.0 s removed it while keeping the slow ring damped. Firmware default changed (`a0d48b0`).
  - **Result:** at τ 2.0 the LQR gains give kick ring ζ 0.43, pitch std 0.16° and wheel speed std 8.4 mm/s, the best of any setting.
- **Drive tests at τ 2.0 (three falls):**
  - **20 Hz shake:** above about 150–200 mm/s a 20 Hz shake builds up, at both the step 2 and the LQR gains. The accelerometer rejects half its samples and *u* swings ±3 m/s².
  - **Turning:** on the spot at 60 °/s it was fine.
  - **Last fall:** a backward command at speed 300 that started with the wheels backing out from under a 4.4° forward lean.
- **Current setup:** step 2 gains saved as start-up, τ 2.0, 1.0 A, drive speed ≤ 200 mm/s.

## Actions arising

- Fit a LiPo voltage alarm on the balance lead (the SKR doesn't measure the battery).
- Optional, for an accurate coil resistance: pass a known current (bench supply set to about 0.5 A) through one coil and measure the voltage right at the motor leads. Then R = V/I.
- Permanent IMU mount. Since 23 Nov the IMU is wedged between the top plate and the battery, 98 mm above the axle. Make it a screwed bracket at that height (or update `IMU_H`).
- Optional: 100 kΩ NTC thermistor on TH0/THB for logged motor or driver temperature; add the TMC `t120`/`t143` flags to the status.

## Next

- Phase 10 pass: stands 60 s+ holding position, recovers from pushes, drives and turns on command (done 23 Nov).
- Phase 11 (continued in [phase11-model.md](phase11-model.md)):
  - **The 20 Hz drive shake:** traced to the drive train, and removed with a 20.5 Hz notch.
  - **The 4–5 Hz mode:** traced to the wrong acceleration compensation, fixed by ACCEL_COMP=2.
  - **Defaults:** the everyday gains became the firmware defaults.
  - **LQR:** shelved; it shook at speed.
  - **Comparison protocol:** done, three sessions on 24 Nov (summary in the engineering reference, section 8.1). LQR is best standing, pole placement smoothest moving, everyday the best all-round set. Stopping accuracy is limited by the tyre contact band for every controller, and a faster trim walk doesn't fix it.
  - **Next:** a position integral for stopping, and a turn-lean feed-forward.
  - **Current state:** [engineering-reference.md](engineering-reference.md).
