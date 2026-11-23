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
