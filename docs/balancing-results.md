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

## Actions arising

- Fit a LiPo voltage alarm on the balance lead (the SKR doesn't measure the battery).
- Optional, for an accurate coil resistance: pass a known current (bench supply set to about 0.5 A) through one coil and measure the voltage right at the motor leads. Then R = V/I.
- Permanent IMU mount (the current one is about 10° off; make sure it can't rock).
- Optional: 100 kΩ NTC thermistor on TH0/THB for logged motor or driver temperature; add the TMC `t120`/`t143` flags to the status.

## Next

Phase 7 (speed and acceleration limits, free-fall fit for ω₀, balance angle θ_trim), then Phase 8 (safety layer).
