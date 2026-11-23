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
