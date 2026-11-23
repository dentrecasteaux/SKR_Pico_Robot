# Random questions: balancing robot (23 Nov 2026)

Questions asked during commissioning, mainly for background. Nothing was changed as a result of them. Run names refer to folders in `runs/`; details are in `docs/balancing-results.md`.

---

## 1. Did the physical tests agree with what we found once it was balancing?

Mostly yes. The measurements that were simple and direct agreed very well. The ones that relied on a model were close but not exact.

| Physical test | Dynamic finding | Agreement |
|---|---|---|
| Wheel diameter from a 10-turn roll: 62.33 mm | `move 500` on the floor went exactly 500 mm, in a straight line | Exact |
| Training-wheel angles by protractor: 28° span | IMU span between the stops: 29.6° (then 29.4° after re-wedging) | Within 2° |
| Balance angle from the free-fall tests: −3.9° | First balance averaged −3.904° | Within 0.004°. This is why the first balance stayed in place without a position loop. |
| l_eff from the swing test and CoM: 95 mm (91–103) | Kick test: 75 mm (integral method); 90 mm (fit with delay) | About 5–20 % low. The free-fall tests only bounded it (95–200 mm). |
| Speed limit on the stand: 800 mm/s clean, StealthChop | Balancing never came near it, except the one run-away push that pinned VMAX at 0.55 m/s | Consistent |
| Heat run: motors cool, driver heatsink 50 °C | After today's runs: motors 28 °C, heatsink 35 °C | Consistent |
| Vibration test: SpreadCycle resonance at 1–2 rev/s, StealthChop quieter | SpreadCycle broke into an 8 Hz oscillation after kicks; StealthChop never did | Same direction: SpreadCycle is rougher with this motor |

**Things the physical tests didn't predict:**
- The steady 2–3° lean needed while driving, for rolling drag.
- The 2.2–2.4 Hz lightly damped ring, which seems to come from the motor/driver link.
- The gyro picking up a little yaw rate as pitch rate while turning.
- The IMU creeping on its tape.

All four only show up while the robot is actually balancing and driving.

**One model mistake:** I designed the first gains for l_eff = 200 mm, taken from the free-fall tests. That turned out to be about 2.5 times too long. The gains still worked because the design was robust to it. But my later "correct" redesign for 75 mm went unstable, because the model was missing the 2.2 Hz mode.

---

## 2. What do KV and KX do?

The controller is:

  u = KP·e + KD·θ̇ + KV·(v − v_ref) + KX·(x − x_ref)

- **KP and KD** handle the balancing itself (the angle and how fast it's changing).
- **KV** is the gain on wheel speed error: how fast the robot is going compared with how fast it should go.
- **KX** is the gain on position error: where the robot is compared with where it should be.

**The odd part is the sign.** KV and KX are positive. That means if the robot is drifting forward, those terms ask for *more* forward acceleration for a moment. That sounds wrong, but it's how a balancer works. Pushing the wheels forward tips the body backwards. The angle loop then sees the body leaning back and brakes hard to catch it. So the robot ends up slowing down and returning. To go backwards, a balancer first has to lean backwards, and it gets that lean by briefly driving the wheels forward.

**What changing them does:**
- **KV = KX = 0** is plain PD balancing. The robot stays up but wanders. Run `084515` drifted 0.7 m and reached speeds near 0.5 m/s.
- **KX alone** pulls the robot back to its start point, like a spring. KV adds damping so it doesn't overshoot.
- **Larger KV and KX** give a faster return and a faster speed response. They also eat into the damping of the fast balancing loop. Run `130117` (KV 6, KX 5) drove well, but the 2.2 Hz wobble grew during the run. That's why we stayed at KV 4.81, KX 2.94.

**In the model's equation** they appear as the two lowest terms:

  l·s⁴ + (KD − KV·l)·s³ + (KP − g − KX·l)·s² + KV·g·s + KX·g = 0

Both must be positive for stability. KV also reduces the effective damping term (KD − KV·l), which is the effect seen in run `130117`.

---

## 3. It basically worked first time. How did the development process help?

**Remove the usual causes of failure before trying.** Most balancing projects fail on one of three things: a wrong sign, an unknown balance angle, or guessed gains. Phases 4–8 removed all three before the first attempt:
- Every direction was checked: wheels, IMU pitch sign, gyro sign, turn direction.
- The balance angle was measured (−3.9°).
- The gains were computed from measured mass, inertia and CoM height.

**The most important check came first.** The dry run (9.1) checked the controller's sign with the motors disabled. A wrong sign there makes the robot throw itself at the floor at full speed.

**Safety and recording before balancing.**
- Tilt cut-outs, a run-away trip, an IMU health trip and soft start were all in place first.
- The 500 Hz telemetry and the black box (the last 2 s before every trip) were working before the first attempt.
- So every failure afterwards could be diagnosed from data, not from memory of what it looked like.

**Small steps with pass criteria.** Each phase had a clear pass/fail test, and results were recorded before moving on.

**Diagnose from the logs, not by eye.** Examples:
- **STOP dropping the robot:** robotd sent STOP when a drive hold expired, and STOP ended balancing.
- **One wheel stopping in turns:** gyro yaw-to-pitch coupling, not a loose wire.
- **Slow driving:** the rolling-drag lean, not low acceleration.
- **Wobble growing during runs:** the IMU creeping on its tape.

Each was found in the data, fixed, and then confirmed in the next run.

**What it didn't prevent:** my own model errors. The free-fall assumption was wrong, l_eff was guessed at 200 mm, and the redesigned gains went unstable. The process caught those only because every run was logged and checked.

---

## 4. How quickly can the motor current change? Are we being kind to these motors?

**How fast the current can change.** The coil inductance limits it:

  dI/dt = (V − back-EMF) / L

At standstill that's 14.8 V / 4.8 mH ≈ 3 A per millisecond. So the coil can go from zero to the 1.4 A peak in about half a millisecond. That's far faster than anything the balance loop asks for, which updates every 2 ms and has about 12 ms of delay end to end.

**Two things make it less of an issue still:**
- **The current is already flowing.** A stepper holds its run current all the time while balancing. Torque comes from the rotor lagging slightly behind the commanded position, so it responds almost at once to a change in commanded speed.
- **Speed is where inductance bites.** As speed rises, back-EMF rises and there's less voltage left to reverse the current each step. That's why torque falls off above about 0.6–0.9 m/s, and why the clean top speed on the stand was 800 mm/s. It's a speed limit, not a "jump" limit.

**Are we being kind? Yes, very.**
- **Current:** we run 1.0 A RMS of a 1.8 A rating.
- **Torque:** balancing needs roughly 0.03 N·m per motor, against 0.7 N·m holding torque.
- **Acceleration:** the hardest commands seen were about 3.7 m/s², against a 6 m/s² limit.

So torque and current are not the limit here. Speed, delay, and the springiness of the motor/driver link are.

**A twist:** more current actually made things *worse*. At 1.3 A the 2.2 Hz ring was *less* damped (run `142738`). A stiffer magnetic "spring" with very little damping rings more. So turning the current up isn't free.

---

## 5. Is auto-tuning feasible?

Yes, at several levels. Most of the parts already exist.

**1. Measure the plant automatically, then design the gains.**
- **Measure:** the kick test already measures l_eff and the loop delay automatically. A chirp would also capture the 2.2 Hz ring.
- **Design:** a script could fit a model to that data and compute gains (LQR or pole placement).
- **Try:** it would then apply the new gains in small, checked steps.

This is the most practical option. The catch, shown today, is that the model must include the ring, or the "optimal" gains can be unstable. So any auto-tuner needs a verification step before accepting new gains. It should, for example, refuse to cut KD by more than about 10 % at a time, and repeat the kick test with each new set.

**2. Online adaptation, which is already running in small ways:**
- The automatic trim learns the balance angle while balancing.
- The gyro yaw-to-pitch coupling is learned whenever the robot turns.

Extending this to adapt the gains themselves (adaptive control, gain scheduling with battery voltage) is possible. It's also harder to make safe.

**3. Classic "relay" or trial-and-error auto-tuning** deliberately pushes the system to the edge of stability. That's fine for a heater, risky for a balancing robot.

**Verdict:** option 1 is feasible as a `balance_run.py --autotune` mode once we have a model that reproduces the ring. It's a natural follow-on from the LQR work.

---

## 6. Can it be tuned manually from the web page?

Not yet from the page itself, but all the plumbing is there.

- The firmware accepts `BAL_SET` while balancing, so gains can be changed live. We've been doing this with `robotdctl bal-set`.
- robotd and the web server already pass `bal_set` through; it's on the web allow-list.
- **What's missing:** controls on the page, such as fields or sliders for KP, KD, KV, KX, TRIM, DRIVE_LEAN and DRIVE_ACCEL, plus a live display of the balance state, error, speed and position.

**Two things to decide before adding it:**
- **Persistence:** balance parameters are lost at reboot, because they aren't saved to flash. A "save" button would need flash storage, or a settings file on the Pi that is re-applied at the start of each run.
- **Safety:** limit the ranges on the page. The firmware already rejects out-of-range values, but a page could also stop KD being cut a lot in one step. Today showed what that does.

---

## 7. Which equations are used for balancing right now?

All of these run on core 1, every 2 ms (500 Hz), straight after each IMU sample.

**Pitch estimate (complementary filter):**
- Gyro pitch rate, corrected for the learned yaw coupling c (−0.0177):
  θ̇ = q − c·r
- Propagate:
  θ ← θ + θ̇·dt
- Accelerometer angle, with the wheels' own acceleration removed (ACCEL_COMP):
  θ_acc = atan2(−(a_x − u/g), √(a_y² + a_z²))
- Blend towards it slowly (τ = 0.5 s), unless the total acceleration is more than 0.15 g away from 1 g:
  θ ← θ + k·(θ_acc − θ),  k = dt/(τ + dt)

**Drive set points (from the web page or robotdctl):**
- The acceleration set point a_ref heads towards 4·(V_cmd − v_ref), limited to ±DRIVE_ACCEL (0.8 m/s²). It changes by at most 4 m/s³ (jerk-limited).
- Then:
  v_ref ← v_ref + a_ref·dt
  x_ref ← x_ref + v_ref·dt
- The turn rate set point ramps at up to 180 °/s².

**Target lean, the drive feed-forward:**

  θ_ref = a_ref/g + DRIVE_LEAN·tanh(v_ref / 0.05 m/s)

The first term is the lean needed to accelerate. The second is the extra lean (2°) needed against rolling drag while moving.

**Error:**

  e = θ − trim_est − θ_ref

**Controller output (a wheel acceleration):**

  u = KP·e + KD·θ̇ + KV·(v − v_ref) + KX·(x − x_ref) + a_ref
  KP 36.7, KD 4.19, KV 4.81, KX 2.94 (SI units: rad, m, s); u clamped to ±6 m/s²

**Integrate to speed and position, then to wheels:**
- Speed and position:
  v ← v + u·dt  (clamped to ±0.55 m/s)
  x ← x + v·dt
- Left and right wheels for turning:
  v_L = v − ω_ref·(track/2),  v_R = v + ω_ref·(track/2)
- Step rate = wheel speed × 16,343 microsteps per metre. The PIO step generators produce the step pulses.

**Automatic trim**, only while calm (|v − v_ref| < 0.05 m/s and |θ̇| < 5 °/s):

  trim_est ← trim_est − TRIM_RATE·(x − x_ref)·dt

**The model behind the gain design** (a cart-pendulum with acceleration input):

  θ̈ = (g·θ − u) / l_eff

**Safety checks, every sample:**
- **Tilt:** trip if pitch > +10.7° or < −14.7°.
- **Run-away:** trip if the speed sits at VMAX for 0.3 s.
- **IMU:** trip if a sample is more than 20 ms late or invalid.

---

## 8. How does the balancing work, in plain terms?

Think of balancing a broom on your hand. If it starts to tip, you move your hand underneath it, a bit further and faster than it's tipping, so the bottom catches up with the top. The robot does the same with its wheels.

1. **Sense:** 500 times a second the IMU reports the pitch rate (gyro) and the direction of gravity (accelerometer). Core 1 blends them into a pitch angle that is both fast and drift-free.
2. **Decide:** the controller compares that angle with the balance point (trim) and works out a wheel acceleration:
   - Leaning forward means accelerating forward, to get the wheels back under the centre of mass.
   - Tipping quickly means acting harder (the KD term).
   - The speed and position terms (KV, KX) gently steer it back to where it should be. To do that they first move the wheels the "wrong" way, so the body leans the right way.
3. **Act:** the acceleration is integrated into a wheel speed. The speed becomes a step rate for the two stepper motors. Steppers follow commanded speed exactly, which is why the controller can command acceleration directly.
4. **Drive:** to drive forward, the target lean is set ahead of time (feed-forward), so the robot leans and goes straight away. For turning, the two wheels are given slightly different speeds.
5. **Keep it safe:** core 1 checks the tilt, run-away and IMU health every sample. On a trip the steps stop instantly and the robot drops onto a training wheel. Driving then stays locked until it's cleared.
6. **Everything else is on core 0:** USB commands from the Pi, telemetry and the black box, and the TMC driver UART. That keeps the 2 ms control loop free of interruptions.

---

## 9. Why is there no integral term?

There is integral action, just not in the usual place.

- **Position is the integral of speed.** In the speed loop, the KX·(x − x_ref) term is an integral term: it builds up while the robot drives off at the wrong speed, and pulls it back. The Phase 10 plan wrote this as a PI speed loop (K_i on speed); KX is the same thing.
- **The automatic trim is a second, slower integrator.** It adjusts the balance angle until the robot rests at its home position. It removes the steady offset that an integral term usually exists to fix.

**Why not integrate the angle error as well?**
- **It would fight the trim.** The "right" angle isn't known exactly, so an integrator on angle error would wind up trying to hold the wrong angle. The run where the trim was 5.8° out (`115606`) shows what a steady angle error does: the robot parked 0.9 m from home.
- **The robot is unstable on its own.** An integrator adds phase lag. On a plant that falls over without help, that eats the stability margin, which is already tight because of the 12 ms delay and the 2.2 Hz ring.
- **Wind-up:** during a long push or a trip-and-catch, an angle integrator would build up a large value and then overshoot.

So the integral jobs (no drift, no steady offset) are covered by KX and the automatic trim, which is safer.

---

## 10. Tidy up the web page. A new one for…?

*(The question was cut off after "New one for". My reading: a separate page for the balancing robot. Please confirm.)*

**What a balance page could hold:**
- **Arm and stop:** ARM, Ctrl-C-style STOP, ESTOP and the drive-lock Clear button.
- **Live status:** balance state, pitch, error, speed, position, trim estimate and the learned gyro coupling.
- **Drive pad:** the balance-mode limits (250 mm/s, 60 °/s) and the zero-on-release behaviour.
- **Tuning panel:** KP, KD, KV, KX, TRIM, DRIVE_LEAN, DRIVE_ACCEL, with safe ranges and limited step sizes (see question 6).
- **Run controls:** start a logged run with notes, fire a set of kicks, and show the ring frequency and damping afterwards.
- **Runs list:** downloads and quick plots of pitch, u and speed.

**What would move off the main page:** the three-wheeler items. That means finite MOVE/TURN, the IMU test panel and the TMC tuning, which could go to a "bench" page.

---

## 11. Would BLDC motors be easier to tune?

Easier in some ways, harder in others. The problems would be different, not fewer.

**What steppers give us:**
- **Speed and position for free:** the wheel goes exactly where it's told unless it slips steps. No encoders are needed, and the controller can command acceleration directly, which makes the model very simple.
- **Holding torque:** strong at low speed, which is exactly where a balancer spends its time.

**What steppers cost us:**
- **No feedback:** a lost step goes unnoticed.
- **Limited top speed:** the 4.8 mH inductance limits it to about 0.8 m/s here.
- **Driver-mode quirks,** as found today: SpreadCycle's 8 Hz oscillation, and StealthChop's 2.2 Hz ring that changes with current and battery state.
- **Noise and vibration.**

**What BLDC motors with field-oriented control give:**
- For example hoverboard hub motors, or gimbal motors with SimpleFOC or ODrive drivers.
- **Torque control:** the controller asks for torque directly. That's the textbook balancing setup.
- **Smooth, quiet running and much higher speed.**
- **Back-drivable motors,** so pushes are absorbed more gently.

**What BLDC motors cost:**
- **Encoders:** they need encoders or hall sensors on each wheel for speed and position.
- **More drivers to set up:** the current loop and the encoder alignment, although the driver usually handles both.
- **A more complex model:** wheel and body inertia couple through the torque.
- **Cost and complexity:** often more expensive and more complex to wire.

**Verdict for this project:** the steppers are working well. The remaining issue, the lightly damped ring, looks fixable within the stepper setup, through battery state, driver settings or a better model. BLDC would be worth it if you wanted much higher speed or quieter running, or the textbook torque-control setup for the controller comparison. It would mostly mean starting the motor side again rather than being "easier".
