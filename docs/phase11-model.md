# Phase 11.1: balance model and first LQR gains (23 Nov 2026)

Tool: `analysis/phase11/phase11_model.py` (`fit`, `check`, `steps`); fitted parameters in `analysis/phase11/model_fit.json`.

## Model

- **Pendulum on the axle:** l·θ'' = g·θ − a. Here *a* is the commanded wheel acceleration, delayed by τ (stepper, IMU filter and loop timing together).
- **Joint compliance:** a pole pair and a zero pair between the body and the IMU reading, with DC gain 1. This is the prototype's compliant joints. Their damping is what handling changes.
- **Controller:** the firmware's, exactly: u = KP·θ + KD·θ̇ + KV·v + KX·x + kick, at 500 Hz.

The model is fitted to the average response to the 40 ms, 2 m/s² kicks, matching both the pitch rate and *u* over 1.5 s.

A rigid pendulum plus delay cannot match the data (fit error 8.9 °/s; data RMS 14). Its best fit is l 116 mm and it predicts no ring at all. Adding the pole and zero pair cuts the error to 3.2.

## Fitted parameters and what they predict

| Run | Kicks | l (mm) | Delay (ms) | Pole (Hz, ζ) | Zero (Hz, ζ) | Predicted ring | Measured ring |
|---|---|---|---|---|---|---|---|
| 161333, settled (12 min) | 130 | 42.8 | 10.6 | 1.19, 0.12 | 1.81, 0.25 | 2.37 Hz, ζ 0.054 | 2.36 Hz, ζ 0.048 |
| 143358, long run | 40 | 45.1 | 9.7 | 1.22, 0.15 | 1.84, 0.27 | 2.37 Hz, ζ 0.067 | 2.40 Hz, ζ 0.067 |
| 173835, A (power cycle) | 15 | 44.5 | 10.5 | 1.20, 0.09 | 1.80, 0.25 | 2.37 Hz, ζ 0.066 | 2.40 Hz, ζ 0.073 |
| 172551, just handled | 20 | 46.4 | 9.7 | 0.99, 0.16 | 1.45, 0.39 | 2.15 Hz, ζ 0.246 | 2.17 Hz, ζ 0.225 |

- **Ring from a fit to other quantities:** the closed-loop ring frequency and damping were not fitted directly. The fit only matches the pitch-rate and *u* traces.
- **Cross-check:** the settled model predicts the other two settled runs about as well as their own fits do.
- **What handling changes:** only the compliance damping and frequency. The length, delay and gains stay the same.
- **Other gain sets:** the model also predicts the instability of the redesigned gains (KP 30.55, KD 2.19): ζ −0.21, and the robot fell in 2 s. It predicts KD 4.6's slightly better ring: 2.31 Hz, ζ 0.08 against the measured 2.28 Hz, ζ 0.10.

## What this says about the current gains

- **Why it rings:** the 2.37 Hz ring is a closed-loop pole drawn toward the compliance's lightly damped zero pair. That's why the joint friction sets its damping.
- **Little margin:** with the current gains (KP 36.7, KD 4.19, KV 4.81, KX 2.94), the model goes unstable if the joints get a little looser than ever measured. It is close to unstable if l is 20 % longer.

## LQR

- **Textbook LQR fails here:** using Bryson's rule on the rigid model gives KP about 120, KV/KX about 30. Those gains are unstable on the fitted model once the delay grows by 4 ms.
- **The chosen gains:** a sweep of Q/R weights was checked on the fitted model and on five variants: just handled; delay +4 ms; l ±20 %; joints looser than ever seen. The most robust is Q = diag(10, 1, 300, 0.3) on [x, v, θ, θ̇], R = 0.1, with l 75 mm: **KP 73.5, KD 4.63, KV 10.0, KX 11.8.**
- **Prediction for these gains:** the settled ring becomes 2.95 Hz with ζ 0.38 (now 0.054), and a kick settles in 0.6 s (now 2.6 s). The worst-case damping over the variants is 0.22. Peak *u* for a kick is 3.7 m/s², under the 6 m/s² limit.
- **KD hardly changes.** The extra damping comes from more KP, KV and KX.

## Test plan: step there and check each prediction

Each step moves 1/5 of the way and is predicted to be better than the last. The measured ring frequency rising from 2.37 Hz is the check that the model holds at gains it hasn't seen.

| Step | KP | KD | KV | KX | Predicted ring, settled | Kick settles |
|---|---|---|---|---|---|---|
| 0 (now) | 36.7 | 4.19 | 4.81 | 2.94 | 2.37 Hz, ζ 0.05 | 2.6 s |
| 1 | 44.1 | 4.28 | 5.85 | 4.71 | 2.49 Hz, ζ 0.09 | 1.5 s |
| 2 | 51.4 | 4.37 | 6.89 | 6.48 | 2.61 Hz, ζ 0.14 | 1.1 s |
| 3 | 58.8 | 4.45 | 7.92 | 8.26 | 2.73 Hz, ζ 0.20 | 1.0 s |
| 4 | 66.1 | 4.54 | 8.96 | 10.03 | 2.85 Hz, ζ 0.27 | 0.9 s |
| 5 (LQR) | 73.5 | 4.63 | 10.00 | 11.80 | 2.95 Hz, ζ 0.38 | 0.6 s |

**Each step:**
1. Set the gains in the web Tuning panel, with "Allow gain changes over 15 % in one step" ticked.
2. Run about 2 minutes with 20 kicks.
3. Compare the reported ring with the table.

Stop and go back to the defaults (power cycle) if the robot buzzes, wobbles more, or the measured ring moves away from the prediction instead of toward it. After the last step, check driving and turning again, because KV and KX are much larger.

## Results

| Step | Run | Kicks | Measured ring (mean) | Predicted (settled / handled) | Refit with this run's gains |
|---|---|---|---|---|---|
| 1 | 203054 | 20 | 2.58 Hz, ζ 0.20 (late kicks 2.6 Hz, ζ 0.15–0.20) | 2.49 Hz, ζ 0.09 / 2.31 Hz, ζ 0.31 | l 43.9 mm, delay 11.2 ms, as before; joints more damped (zero ζ 0.38, pole ζ 0.26). It reproduces the ring (2.57 Hz, ζ 0.18) and says the old gains would have given 2.40 Hz, ζ 0.14 in the same state. |

Step 1: the frequency rose as predicted, and length and delay are unchanged at the new gains, so the model holds. The joints happened to be in a better-damped state than the settled reference, so damping alone can't be compared run to run. Compare frequency, and use the refit's old-gains figure.
| 2 | 204006 | 20 | 2.88 Hz, ζ 0.34 (late kicks 2.8–2.9 Hz, ζ 0.25–0.30) | 2.61 Hz, ζ 0.14 settled; 2.75 Hz, ζ 0.23 with the step 1 refit (this evening's joints) | — |

Step 2: better than predicted. Both the frequency and the damping rise faster with the gains than the model says, so the model is on the cautious side here. (Runs 203415 and 203707 were at step 1 gains: the new values weren't applied. They repeat step 1: 2.58 Hz, ζ 0.19–0.20.)
| 3 | 204308 | 20 | **New mode at 4.2–4.6 Hz, ζ 0.00–0.05** (the 2.8–3.9 Hz ring still ζ 0.35) | 2.98 Hz, ζ 0.29; no 4–5 Hz mode in the model | — |

**Step 3: stopped here.** A second mode at 4–5 Hz grows with the gains. It isn't in the model: it was well damped and small at the old gains, so the kick fits never saw it. RMS pitch rate in the 3.8–5.2 Hz band during balancing: step 1 about 2.2 °/s, step 2 about 3.0, step 3 about 5.3 and still rising through the run, at ζ ≈ 0. Step 2 is the best gain set so far. Next: add the 4–5 Hz mode to the model, fitted from run 204308, where it is strongly excited, then redesign.
| 4 | 204602 | 20 | 4.7–4.9 Hz, ζ 0.00–0.04 on every kick | 2.85 Hz, ζ 0.27 | — |

Step 4 confirms step 3: the 4–5 Hz mode moves up in frequency with the gains (4.4 → 4.8 Hz) and stays almost undamped. The band RMS is about 7 °/s through the whole run, a sustained shake. Step 5 was not run. Step 2 is the configuration to use.
| 5 (LQR) | 204928 | 20 | 5.1–5.2 Hz, ζ 0.00–0.03 on every kick | 2.95 Hz, ζ 0.38 | — |

Step 5 (the LQR set) stood for 89 s, but with a sustained 5.15 Hz shake: band RMS about 11 °/s, *u* std 0.84 m/s² (step 2: 0.47), peak *u* 5.0 m/s², close to the 6 m/s² limit. The fast mode's frequency follows the gains (step 3 4.4, step 4 4.8, step 5 5.15 Hz) at near-zero damping. It looks like a limit cycle (amplitude steady at about 11 °/s rather than growing), so something nonlinear is capping it, most likely the step-rate or acceleration limits or backlash. The LQR gains are not usable until this mode is in the model. Step 2 stays the configuration to use.
| 2 again | 205340 | 20 | 2.88 Hz, ζ 0.30 (late kicks ζ 0.25–0.30); 4 Hz band 3.1 °/s, *u* std 0.46 | as first step 2 run | — |

Step 2 repeated after steps 3–5: the same as the first step 2 run (2.88 Hz, ζ 0.34; band 2.9 °/s; *u* std 0.47). The strong shaking at steps 3–5 left no lasting change. **Step 2 (KP 51.4, KD 4.37, KV 6.89, KX 6.48) is the new best known configuration.**

## 11.3 The 4–5 Hz mode: first look (23 Nov evening)

- **The mode moves with the gains.** Its frequency rises with them (4.4, 4.8, 5.15 Hz at steps 3–5) at ζ 0.01–0.03, so it is a loop mode, not a fixed structural resonance.
- **The linear plant at 3–8 Hz looks the same at every step.** u→q measured over each whole run: about 0.26–0.30 at +65° near 5 Hz, close to the model's 0.23–0.25 at +60°. The plant did not change with the gains.
- **Simple additions don't reproduce it.** Each was fitted to the measured poles of steps 1–5: extra delay, a first-order lag after the delay, a compliant drive (2nd order between commanded and real wheel acceleration), and the same with l and the joint damping free. Each predicts a fast mode near 5.5–6 Hz whose frequency barely moves with gain, while the robot's moves from 4.4 to 5.15 Hz. The best (compliant drive plus extra delay) still misses step 3 badly. Fit code: `polefit.py`, `poles_ct.py` (in the session workspace, not yet in the repo).
- **What's left:** something the linear plant model doesn't include. Candidates:
  1. **The motors:** a stepper's stiffness and lag depend on load and current.
  2. **The pitch estimator's acceleration compensation (ACCEL_COMP):** it feeds the commanded acceleration back into the angle estimate, a second loop that grows with the gains. Less likely: the accelerometer path is low-passed at 0.32 Hz (τ 0.5 s), so at 5 Hz it is about 16 times weaker.
- **Discriminating runs, both at the step 3 gains, where the mode is clear but bounded:**
  - **(a) Motor current 1.3 A instead of 1.0 A.** A motor cause should shift or damp the mode.
  - **(b) ACCEL_COMP off, only if (a) changes nothing.** An estimator cause should remove or move the mode.

**Test (a), run 211356: step 3 gains at 1.3 A, 220 s.** The fast mode stays at the same frequency (spectral peak 4.27 Hz, the same as at 1.0 A; kick rings 4.4–4.7 Hz). It is a little better damped: kick ζ 0.03–0.09 against 0.01–0.03; band RMS about 4.0 °/s against 5.3; *u* std 0.42 against 0.53. Motor stiffness doesn't set the frequency, so the motors are not the main cause, though more current helps a little. Next: test (b), ACCEL_COMP off.

**Run 212054: step 2 gains at 1.3 A.** Kick ring 2.91 Hz, ζ 0.25 (1.0 A: 2.88 Hz, ζ 0.30–0.34). 4–5 Hz band 2.75 °/s (1.0 A: 2.9–3.1); *u* std 0.47 (the same). At step 2, 1.3 A makes no useful difference, so 1.0 A stays the everyday current: cooler motors and drivers.

**Test (b), run 212700: step 3 gains, 1.0 A, ACCEL_COMP off. The fast mode is gone.**

| Step 3 gains | Comp on (204308) | Comp off (212700) |
|---|---|---|
| Pitch rate RMS, 3.8–5.2 Hz band | 5.2 °/s | **1.5** |
| *u* std | 0.53 m/s² | **0.37** |
| Pitch std | 0.44° | **0.20°** |
| Kick ring | 4.4 Hz, ζ 0.03 | 2.12 Hz, ζ 0.08 (joint mode, less damped) |

**The 4–5 Hz mode comes from the pitch estimator's acceleration compensation.** The earlier "only 1/16 as strong at 5 Hz" argument ignored the gain. The accelerometer path enters as roughly KP·u/(g·τ·ω), about 0.4 per m/s² at 5 Hz with KP 59, so it closes a second loop of real strength. Without compensation the accelerometer still sees the IMU's own acceleration (the wheel acceleration plus h·θ̈), which makes a different, slower loop. That is probably also part of what the fitted joint "dipole" stood for.

A model with the filter in it (rigid l 94 mm, which matches the 95 mm measured in Phase 1, a 7.8 ms delay and the IMU 170 mm above the axle) reproduces the fast mode's rise with gain (4.5, 4.8, 5.1 Hz against the measured 4.4, 4.8, 5.15). It does not yet also reproduce the 2–3 Hz joint ring (`est_model.py`, `est2*.py` in the session workspace).

**Next test (c): step 3 gains, comp on, filter τ 2.0 s instead of 0.5 s.** This weakens the accelerometer loop about 4 times whichever way it is compensated. If both the fast mode and the slow ring improve, the fix is a longer filter time constant, or a better compensation: subtract the IMU's actual acceleration, the delayed u plus h·θ̈, instead of u.

**Test (c), run 214945: step 3 gains, comp on, τ 2.0 s. Best run of the day.**

| Step 3 gains | τ 0.5, comp on | Comp off | **τ 2.0, comp on** |
|---|---|---|---|
| 3.8–5.2 Hz band | 5.2 °/s | 1.5 | **1.8** |
| Kick ring | 4.4 Hz, ζ 0.03 | 2.12 Hz, ζ 0.08 | **2.6 Hz, ζ 0.30** (0.22–0.59) |
| Pitch std | 0.44° | 0.20 | **0.18** |
| *u* std | 0.56 m/s² | 0.38 | 0.44 |
| Wheel speed std | 16.7 mm/s | 11.4 | **9.5** |

A longer filter time constant removes the fast mode and keeps the slow ring damped. `Config::IMU_FILTER_TAU_S` is now 2.0 s, which needs a build and flash. Until then, set it after each power cycle with `call("imu_set", tau=2.0)`. Next: steps 4 and 5 with τ 2.0, then a fresh model fit at τ 2.0, where the estimator loop matters much less and the plant should be close to the rigid model.

**Run 215717: step 4 gains, τ 2.0 (new firmware build).** Kick ring 2.81 Hz, ζ 0.51 (0.40–0.53, very consistent). 3.8–5.2 Hz band 1.8 °/s; pitch std 0.17°; *u* std 0.46; wheel speed std 9.2 mm/s. At τ 0.5 the same gains gave a 4.8 Hz ring at ζ 0.02, a 7.1 °/s band, pitch std 0.44° and 18.8 mm/s.

**Run 220023: step 5 (the LQR gains), τ 2.0. Best of all runs.** Stood 109 s, kick ring 3.5 Hz, ζ 0.43 (0.35–0.48 on every kick), 3.8–5.2 Hz band 1.8 °/s, pitch std 0.16°, *u* std 0.42, wheel speed std 8.4 mm/s. At τ 0.5 the same gains gave a 5.15 Hz shake, pitch std 0.58°, *u* std 0.88 and 25.4 mm/s. The Phase 11.1 prediction for these gains was a ring at 2.95 Hz, ζ 0.38: close in damping, 0.5 Hz low in frequency.

### Summary, steps 3–5 at τ 2.0

| Gains | Kick ring | Pitch std | Wheel speed std | *u* std |
|---|---|---|---|---|
| Step 2 at τ 0.5 (morning's best) | 2.88 Hz, ζ 0.30–0.34 | 0.34° | 14.5 mm/s | 0.49 |
| Step 3 | 2.6 Hz, ζ 0.30 | 0.18° | 9.5 | 0.44 |
| Step 4 | 2.8 Hz, ζ 0.51 | 0.17° | 9.2 | 0.46 |
| **Step 5 = LQR** | **3.5 Hz, ζ 0.43** | **0.16°** | **8.4** | **0.42** |

With the estimator fixed, the LQR gains from the rigid model are the best controller so far, and the motors work no harder. Still to check: driving and turning with these gains.

### Drive test with the LQR gains at τ 2.0: lost control in a turn (runs 221152, 221218 black box)

- **Forward drive, t ≈ 11.8–15.5 s:** reached about 290 mm/s, with a 1–1.5 Hz pitch oscillation of ±50 °/s (*u* ±3.8 m/s²). The accelerometer was rejected for much of it (flag 0x04).
- **Turn, t ≈ 16.0–18.4 s, about 90 °/s:** the robot's forward speed ran away from about 50 to 460 mm/s, with a steady *u* of about +0.2 m/s² and the pitch reading going from +5° to −4°. The inner (right) wheel sat near zero, so it looked like one wheel had stopped. This is the same symptom as the 23 Nov yaw-coupling bug (balancing-results.md), but about four times larger.
- **After the turn:** braking, then the collision with the desk at about t = 19.8 s. Leaning on it at 7–10° for about 3 s while oscillating, then TILT_FWD at 23.3 s. The run ended with "link lost".
- **Leading hypothesis:** with τ = 2 s, the accelerometer corrects pitch errors four times more slowly, so any yaw-coupling error left after the learned GYRO_QR correction (and the centripetal pull on the off-centre IMU) builds up four times larger during a turn. The higher KV and KX then turn the false lean into forward speed.
- **To separate the two changes, on open floor, gentle turns first:**
  1. Step 2 gains at τ 2.0, which isolates τ.
  2. LQR gains at τ 0.5 with turns only, short and slow.
- **If τ is the cause, the likely fix:** shorten τ only while turning (|r| > 30 °/s), or improve the coupling correction.

### Drive test 2, LQR gains at τ 2.0 (run 221515, TILT_FWD after 63 s)

- **A 20.5 Hz oscillation above about 200 mm/s.** Every time the wheel speed passed about 200 mm/s, forward or back, a violent oscillation started: pitch-rate sd 30–40 °/s, *u* sd 2–3 m/s², and the accelerometer rejected 50–60 % of samples. Spectral peak 20.5 Hz in all five episodes. RMS pitch rate in the 16–25 Hz band against wheel speed:

  | Run | 0–100 mm/s | 100–200 | 200–260 | >260 |
  |---|---|---|---|---|
  | 113258 (old gains, τ 0.5) | 1.3 | 0.6 | 0.6 | — |
  | 221515 (LQR, τ 2.0) | 3.6 | 13.6 | 22.9 | 28.8 °/s |

- **Turning on the spot was fine.** About 60 °/s for 6 s, t = 39.5–46 s, at low speed. So the first test's loss of control was probably this speed oscillation, not the turn itself.
- **Earlier hint:** a ~19–20 Hz, undamped "compliant drive" resonance also came out of the 11.3 pole fit (drive+base). Something in the drive train (rotor and wheel/tyre against the body) seems to resonate near 20 Hz. The stepper's own damping falls with speed, and the LQR gains give more loop gain at 20 Hz than the old ones.
- **Likely fix:** a low-pass filter (about 8–10 Hz) on the gyro rate used by the KD term, and on *u*. That's standard practice. It removes the loop gain at 20 Hz while barely changing the 2–5 Hz behaviour.
- **Until then:** drive only at the step 2 gains, and keep the speed under about 0.15 m/s with the LQR gains.

### Drive test 3, step 2 gains at τ 2.0 (run 222024, stood 102 s, TILT_FWD)

- **The 20 Hz shake appears with these gains too.** RMS pitch rate in the 16–25 Hz band: 2.9 / 11.7 / 19.1 / 27.4 °/s at 0–100 / 100–200 / 200–260 / >260 mm/s, almost the same as with the LQR gains. So it is not the LQR gains. Of today's changes, the τ 2.0 firmware is the main suspect: this morning's drives at τ 0.5 showed none (0.6 °/s at 200–260 mm/s). The mechanism is still unclear. A direct check: the same drive at step 2 gains with τ set back to 0.5.
- **The fall:** after the speed was raised to 300, the next command was backward. For 4 s before it, the robot stood still at a steady 4.4° pitch, 2.5° past the 1.9° trim. At 109.46 s *u* went to −1.2 m/s² and the wheels backed out from under the forward lean. Pitch went 4.4 → 10.7° in 0.25 s, and the controller's forward correction came too late.

### 24 Nov: drive test at step 2 gains with τ 0.5 (run 060114): the 20 Hz shake comes from τ 2.0

Speeds of 300, then 200, then 300 mm/s, back and forth for 100 s. Smooth, and no fall.

| Wheel speed | 0–100 | 100–200 | 200–260 | >260 mm/s |
|---|---|---|---|---|
| 20 Hz band, τ 2.0 (222024) | 2.9 | 11.7 | 19.1 | 27.4 °/s |
| 20 Hz band, τ 0.5 (060114) | 1.8 | 1.6 | 2.0 | 4.6 °/s |
| *u* sd, τ 2.0 | 0.36 | 0.93 | 1.49 | 2.12 m/s² |
| *u* sd, τ 0.5 | 0.36 | 0.27 | 0.27 | 0.39 m/s² |

**Why τ matters while driving:** the pitch estimate drifts from the accelerometer pitch. Compare the low-passed estimate with the low-passed, acceleration-compensated accelerometer pitch, by speed band:

| Run | Estimate − accelerometer pitch while driving |
|---|---|
| τ 2.0 (222024, 221515) | −1.2 to +1.6° |
| τ 0.5 (060114) | within ±0.13° |

At 150–600 mm/s forward, the accelerometer pitch was 6.1° at τ 2.0 against 4.0° at τ 0.5. With τ 2.0 the gyro's errors while driving (bias, yaw coupling, vibration) are corrected four times more slowly. The robot then runs leaning 1–2° more than intended, which loads the motors harder at speed. The likely result is the 20 Hz drive-train/stepper oscillation. That is a hypothesis, but it fits the speed dependence and the accelerometer rejects.

**Where this leaves the time constant:**

| τ | Standing, high gains | Driving |
|---|---|---|
| 0.5 s | 4–5 Hz mode (the ACCEL_COMP mismatch) | Accurate estimate |
| 2.0 s | Fine | Estimate drifts 1–2°, then the 20 Hz shake at speed |

**The underlying fault is the acceleration compensation.** The IMU sits about 0.16 m above the axle, higher than l_eff (about 0.094 m). It feels the wheel acceleration *delayed* by about 10 ms, plus h·θ̈, and the h·θ̈ term largely cancels and reverses the wheel term. Subtracting the commanded *u* is therefore wrong at 2–5 Hz.

**Proposed fix (ACCEL_COMP = 2):** subtract the IMU's own acceleration, A = u delayed 10 ms + h·dq/dt, where dq/dt is the gyro rate's derivative. It is noisy, but the filter's accelerometer path averages it heavily. Keep a quicker τ (0.5 s) for an accurate estimate. Make it selectable at runtime for A/B runs. Interim alternative: schedule τ (2.0 standing, 0.5 driving).

### 24 Nov test 1: ACCEL_COMP=2, τ 0.5, step 3 gains, standing with 20 kicks (run 061900): pass

| Step 3 gains | Comp 1, τ 0.5 | Comp off, τ 0.5 | Comp 1, τ 2.0 | **Comp 2, τ 0.5** |
|---|---|---|---|---|
| 3.8–5.2 Hz band | 5.2 °/s | 1.5 | 1.8 | **1.8** |
| Kick ring | 4.4 Hz, ζ 0.03 | 2.12 Hz, ζ 0.08 | 2.6 Hz, ζ 0.30 | **2.59 Hz, ζ 0.26** |
| Pitch std | 0.44° | 0.20 | 0.18 | 0.21 |
| *u* std | 0.56 | 0.38 | 0.44 | 0.47 |
| Wheel speed std | 16.7 | 11.4 | 9.5 | 10.5 |

The new compensation removes the 4–5 Hz mode at τ 0.5 and keeps the slow ring damped, much the same as τ 2.0 with the old compensation. That confirms the cause was the u/g compensation mismatch (IMU at 98 mm ≈ l_eff). Next: test 2, driving.

### 24 Nov test 2: ACCEL_COMP=2, τ 0.5, step 2 gains, driving (run 062717): the 20 Hz shake again, then a RUNAWAY fall

- **What happened:** one button press and hold, out and back at 200 then 300. The shake started above about 220 mm/s every time (16–25 Hz band: 9.2 / 12.1 / 16.9 °/s at 100–200 / 200–260 / >260 mm/s; *u* sd up to 2.5 m/s²; up to 75 % accelerometer rejects). It ended in a RUNAWAY trip reversing at about −460 mm/s.
- **Not estimate drift:** this time the estimate tracked the accelerometer pitch within about 0.3° until each shake began.
- **Comparing the three settings:** τ 2.0 + comp 1 shook, this morning's τ 0.5 + comp 1 (also sustained >260 mm/s for 24 s) did not, and τ 0.5 + comp 2 shook.
- **What that suggests:** the 20 Hz mode at speed is marginal under all settings and small differences tip it. A plausible contributor for comp 2: the h·dq/dt term reaches *u* at 20 Hz with a gain of roughly KP·h/(g·τ) ≈ 1, at −90°, next to KD ≈ 4.4.
- **Robust fix:** a notch at 20.5 Hz on the feedback part of *u* (`NOTCH_HZ`, `NOTCH_Q`, firmware below, default off). At Q = 2 it costs about 4° of phase at 3 Hz and 7° at 5 Hz.

### 24 Nov test 3: notch 20.5 Hz, Q 2, with ACCEL_COMP=2, τ 0.5, step 2 gains, driving (run 064053): pass

| 16–25 Hz band by wheel speed | 0–100 | 100–200 | 200–260 | >260 mm/s |
|---|---|---|---|---|
| τ 0.5, comp 1 (060114) | 1.8 | 1.6 | 2.0 | 4.6 °/s |
| τ 0.5, comp 2 (062717) | 2.5 | 9.2 | 12.1 | 16.9 |
| **τ 0.5, comp 2, notch (064053)** | **1.1** | **0.7** | **0.5** | **0.8** |
| *u* sd, notch run | 0.28 | 0.19 | 0.15 | 0.17 m/s² |

The notch removes the 20 Hz shake entirely: 13 s above 260 mm/s, *u* sd ≤ 0.28 m/s², no accelerometer rejects, and the 4–5 Hz band below 1.6 °/s. Smoothest drive so far.

**New firmware defaults:** ACCEL_COMP 2 (IMU_H 0.098, COMP_DELAY_MS 10), NOTCH_HZ 20.5, NOTCH_Q 2, IMU filter τ 0.5. The Pi's `balance_tools.DEFAULTS` and `check_config.py` are updated to match. Next: the LQR gains, standing with kicks and then driving, on these defaults.

### 24 Nov: the LQR gains on the new defaults (comp 2, τ 0.5, notch 20.5 Hz, Q 2)

| LQR gains | Kick ring | 5.2–8 Hz band | Pitch std | Wheel speed std | *u* std |
|---|---|---|---|---|---|
| Comp 1, τ 2.0, no notch (220023, 23 Nov) | 3.5 Hz, ζ 0.43 | 2.1 °/s | 0.16° | 8.4 mm/s | 0.42 |
| Comp 2, τ 0.5, notch Q 2 (065011) | **5.4–6.0 Hz, ζ ≈ 0 on every kick** | 5.4 | 0.28 | 14.5 | 0.57 |
| Comp 2, τ 0.5, notch off (065538) | 3.2 Hz, ζ 0.43; 3 of 20 kicks at 3.8–5.3 Hz, ζ 0 | 2.7 | 0.20 | 10.5 | 0.52 |

At the LQR gains there is a marginal mode at about 5–6 Hz. With comp 2 alone it shows on some kicks. The notch's phase lag (about 8° at 5.6 Hz for Q 2) tips it onto every kick. Comp 1 at τ 2.0 was a little more robust at these gains (but drifts while driving). Options:
1. **A narrower notch, Q 4:** about 4° at 5.6 Hz and still −14 dB over 20–21 Hz.
2. **Tune comp 2** (IMU_H, COMP_DELAY_MS), whose mismatch may be what feeds the 5 Hz mode at KP 73.5.
3. **Accept slightly softer gains,** between step 3 and LQR.

Step 2 gains (saved) remain the practical everyday setting.
