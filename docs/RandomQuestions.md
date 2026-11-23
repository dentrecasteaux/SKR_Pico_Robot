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

---

# More questions (23 Nov, late afternoon)

Context: the robot has a lightly damped wobble at about 2.2–2.4 Hz. A kick makes it ring, and the damping of that ring changes between and within runs (about 0.3 at best, 0.06 at worst). Turning the wheels by hand after a power cycle restored good damping (run `171913`). Driving about 0.7 m forward and back partway through a run did not (run `172551`).

## 12. Does the resonance change with microstepping?

Probably not its frequency, but microstepping could affect what keeps the wobble going. **Not yet tested.**

**Why the frequency shouldn't move:**
- **The timescales are far apart.** The 2.2 Hz wobble is a slow motion of the whole robot: the balance loop, the pendulum and some springy element together. At 1/16 microstepping one microstep is 0.061 mm of travel, and while balancing the step rate is tens to thousands of steps per second.
- **What microstepping changes:** the size of each step and the torque ripple. Neither is a spring or a damper at 2.2 Hz.

**Where it could matter:**
- **Near standstill the wheel moves in visible jerks.** At 10 mm/s the steps come only every 6 ms, and the step generator only changes rate at the end of the current step. The current waveform is also coarser at low step rates.
- **Those jerks are small, repeated disturbances.** They could keep feeding a lightly damped mode, which would raise the quiet wobble without changing the frequency.
- **What 1/32 would do:** halve the jerks. The speed limit is fine at 1/32: 0.55 m/s is about 18,000 steps/s, under the 20,000 limit.

**How to test:** `configure 1000 32 150 STEALTHCHOP`, then a kick run. The firmware reads the microstep setting when it arms, so no reflash is needed. Compare the quiet wobble and the ensemble damping with the 1/16 runs.

## 13. Did my "drive lean" and "drive accel" changes affect the resonance?

No.
- **They only act while driving.** DRIVE_LEAN and DRIVE_ACCEL only do anything while there is a drive command: the target lean is a_ref/g + DRIVE_LEAN·tanh(v_ref / 0.05), and both terms are zero when v_ref and a_ref are zero.
- **The kick tests ran with the robot standing still,** with no drive command, so both settings were inactive throughout.
- **The wobble predates them.** It was already visible this morning (2.6–2.9 Hz at rest, before the feed-forward existed), and the damping changes happened with these settings unchanged.

## 14. The tyres are rubber with some squish. Would that affect things?

Yes, tyres can matter. Today's tests make a flat spot an unlikely explanation for the changing damping, though.

**How tyres enter the dynamics:**
- **A springy contact.** A squishy tyre is a spring, plus some rubber damping, between the wheel rim and the floor. It can flex sideways (tangentially), which gives a little "wind-up" between where the motor puts the rim and where the robot actually is. It can also flex vertically and roll the contact patch.
- **The fall tests hinted at this.** The robot appeared to balance at two different angles (about ±2°) depending on which way it fell, as if pivoting on the front or back edge of a flat contact patch about 4 mm long.
- **A compliant link like this adds a mode,** and can set the frequency and damping of a wobble like the 2.2 Hz one.

**What today's tests say:**
- **A set or flat spot where the robot rests seemed likely,** because turning the wheels by hand restored the damping.
- **But driving about 0.7 m out and back** (about 3.7 wheel turns under power, run `172551`) did *not* bring the damping back up. It carried on falling: 0.23 before the drive, 0.23 just after, 0.17 at the end.
- **So the tyre's resting position isn't the main factor.** The tyres can still be part of the mode itself, as a constant spring.

**The difference between the two tests:**
- **Hand-turning:** the motors were off and the robot was lifted off the floor and handled.
- **Driving:** done under power, without handling.

So the next suspects are things that lifting and handling the robot could reset. For example: the battery or IMU settling on its mount as the robot rocks, a cable, or the stepper rotor's position relative to its poles when it's unpowered.

## 15. Is the ringing an artifact of standing still?

Mostly no. It's a genuine mode of the robot, though standing still is where it's most visible and where its strength varies.

- **It's a closed-loop motion,** made up of the controller, the pendulum and a compliant element (motor magnetic spring, tyre or mount). Kicks excite it while standing, but it doesn't need the robot to be stationary.
- **It's there while moving too.** In run `172551` the 1.2–4 Hz wobble during the drive was about 3.3 °/s RMS, against about 1.9 °/s while standing (excluding the seconds after kicks). That's not smaller.
- **Standing still is special for the steppers, though.** Near zero speed the step generator is at its most jerky, and the motor sits in its magnetic detents. Those effects could be what slowly changes the damping during a standing run, which is why it shows up there.

---

# Second set (24 Nov 2026)

Again for background. Nothing has been changed as a result of these.

## 16. What effect did the 94 Hz IMU filter have?

The MPU6050 has its own low-pass filter (DLPF) on both sensors. In 9.4 we widened it from about 44 Hz (CONFIG 0x03) to about 94 Hz (CONFIG 0x02).

- **What it bought:** about 2 ms less delay (roughly 4.9 → 3.0 ms). Delay costs phase in proportion to frequency: 2 ms is about 2° at the 3 Hz balance wobble and about 14° at 20 Hz. So it helped a little at balance frequencies and more at higher ones. The whole loop delay is about 10 ms, and this was a fifth of it.
- **What it cost:** more vibration gets through. A 44 Hz filter only takes about 20 % off a 20 Hz signal anyway, so it wouldn't have stopped the 20 Hz drive shake. The notch now does that job directly.
- **Measured effect:** no dramatic change at the time. The noise test in 5.6 had shown the wider filter was quiet enough, and the kick tests afterwards gave the same l_eff with a slightly shorter fitted delay.

## 17. iPhone: holding the drive button only drives for 4–8 s

Not diagnosed yet; the logs only show the motion, not why it stopped. How the page drives:
- Pressing a button starts a loop that sends a velocity command, waits for the reply, then sends the next 200 ms later. Each command carries a 1 s lease.
- The robot stops if no new command arrives within the lease.
- The page stops the loop on any failed command, when the button loses its "pointer capture" (pointercancel, lostpointercapture), when the window loses focus, or when a status poll says the robot isn't RUNNING. That last check also disables the buttons, and disabling a held button can itself end the press.

**Likely causes, most likely first:**
1. **A slow or failed request.** A Wi-Fi hiccup, or the phone's Wi-Fi power saving, makes one request take more than about 1 s or fail. Then the lease runs out, or the page sees an error and ends the drive. On the earlier project the Pi's own Wi-Fi power saving caused something similar, and that is now turned off (plan step 3.4).
2. **A momentary bad status poll.** It reads as "not running", so the page ends the drive and briefly disables the buttons.
3. **iOS cancelling a long touch** with pointercancel, for its long-press gesture. The CSS already blocks the callout and text selection, which makes this less likely, but Safari can still do it.

**Diagnosis when we get to it:** make the page say why each drive ended (a short toast or log line: error, lease, cancel, status), and tolerate one slow or failed renewal before giving up.

## 18. Would rigid wheels help?

Somewhat, but they aren't the big lever now.
- **For:** squishy tyres are a spring between the motor and the floor. They are a candidate for part of the ~20 Hz drive mode and the low-frequency compliance in the model. Rigid wheels would make the wheel motion follow the steps more exactly, and push any tyre resonance higher in frequency.
- **Against:** less grip on smooth floors, so more risk of wheel slip during hard corrections, which the controller can't see because it has no wheel encoders. They also pass more of the stepper vibration straight into the chassis, and they ride badly over small bumps and floor edges.
- **Today:** the notch has dealt with the 20 Hz shake, and the slow wobble turned out to be joints and the estimator rather than tyres. Firmer rubber or a thinner tyre on the same rim is a reasonable later experiment. A hard plastic rim alone probably isn't worth it.

## 19. Occasionally during a turn one wheel does all the work

The robot's forward speed drifts during the turn. A turn is the two wheels at plus and minus the turn speed around the forward speed. If the forward speed drifts to about half the turn's wheel-speed difference, the inner wheel stops and the robot pivots on it. That was diagnosed on 23 Nov. While turning, the gyro's pitch axis picks up a small part of the turning rate (the IMU is about 1° off level), so the pitch estimate goes wrong and the controller drives to "correct" it.

The fix was to subtract a learned coupling factor (GYRO_QR). On 24 Nov it read **+0.0347 within 86 s of power-up, before any balancing.** The value measured on the 23rd was −0.0177. The firmware learns it whenever the yaw rate exceeds 30 °/s, so it also learns while the robot is being carried and rotated by hand, and while it is pitching during a turn. A wrong learned value brings the drift back. So "occasionally" probably means "after the robot has been handled in a particular way".

**Likely fix:** learn only while balancing and turning smoothly (small pitch rate), start from the measured −0.0177 at each arm, and limit how far it can move. Not changed yet.

## 20. Wouter Klop's robot uses a variable step size. What does he gain?

I couldn't read his code from the links available: the GitLab page and his write-up don't describe it. So this is the general idea rather than his exact method. His ESP32 robot uses DRV8825 drivers, 24 V and a 200 Hz cascaded loop, and aims at very high speed.

"Variable step size" in stepper balancing robots usually means switching the microstep resolution with speed. For example, 1/16 or 1/32 near standstill for smooth, fine corrections, then 1/8 or 1/4 at high speed. The gain is mainly at high speed: fewer step pulses per second for the same wheel speed, so the processor's step timer and interrupt load stay manageable. On drivers without internal interpolation, low speeds also stay smooth.

For us there is little to gain:
- **Pulse rate:** the PIO hardware generates the pulses, and we use 1/16. 0.55 m/s needs about 9,000 steps/s, well under the 20,000 steps/s cap.
- **Smoothness:** the TMC2209 already interpolates every step to 1/256 internally (MicroPlyer).
- **Risk:** switching resolution on the fly would risk a small position jump at each switch.

## 21. What would a newer IMU give us?

Modern parts such as the ICM-42688-P, BMI088 or LSM6DSO:
- **Lower gyro noise and much better bias stability,** including over temperature. That is the gyro drift the accelerometer has to correct, so with a better gyro the accelerometer can be trusted less (a longer time constant) without the angle wandering.
- **Better, configurable anti-alias filtering and higher output rates,** which help reject motor vibration before it is sampled.
- **Better vibration robustness** (the BMI088 is designed for drones).
- **SPI,** which is faster and more deterministic than our 400 kHz I²C.

But none of our problems so far was sensor noise. The 4–5 Hz mode was the compensation model, and the 20 Hz shake was a mechanical/drive resonance fed by the loop. A new IMU would make the estimator a little better; it wouldn't have fixed either. Mounting matters as much as the part: a rigid mount is worth more than a better chip on tape and a wedge.

## 22. Why did going from τ 0.5 s to 2.0 s help so much, and what did it cost?

- **What τ does:** it sets how strongly the accelerometer pulls the pitch estimate, as a nudge of dt/τ of the difference every sample.
- **What was wrong:** the accelerometer is only meant to supply the long-term direction of gravity. With ACCEL_COMP=1 we subtracted the commanded wheel acceleration from it. But the IMU sits 98 mm above the axle, about the robot's effective pendulum length, so it barely feels the wheel acceleration: the body's rotation cancels it. The subtraction therefore *added* an error at 2–5 Hz, and the estimate passed it to the controller. With KP around 60 that formed a second feedback loop strong enough to make an almost undamped 4–5 Hz mode.
- **Why τ 2.0 helped:** that loop's strength scales with 1/τ, so going to 2.0 s made it four times weaker. That was enough to remove the mode and keep the slow wobble damped.
- **What it cost:** the accelerometer corrects *every* gyro error four times more slowly too: bias, turn coupling, vibration. While driving, the estimate drifted 1–1.6° from the true angle, the robot leaned more than intended, and the 20 Hz drive shake built up at speed.
- **What replaced it:** the real fix was correct compensation (ACCEL_COMP=2: subtract what the IMU actually feels, the delayed u plus h·dq/dt). That let τ go back to 0.5 s, and the 20 Hz shake was then removed with a notch.

## 23. Could τ be 0.5 s while driving and 2.0 s while balancing? Or is a better filter the answer?

Scheduling τ would have worked as a stopgap. The firmware can change τ on the fly without a jump in the estimate. But it treats a symptom (the wrong compensation) rather than the cause. It would also have kept the 4–5 Hz problem whenever τ was short with high gains, which is exactly while driving.

A better filter is the answer, and it has largely happened. ACCEL_COMP=2 fixed the compensation, so one short τ (0.5 s) now works standing and driving (test 1 on 24 Nov), and the notch handles the 20 Hz shake. The next step up is a Kalman filter (question 24).

## 24. What filter is used now? Is it a complementary filter? Is it time for a Kalman filter? Would we still need the notch?

**Now: yes, a complementary filter,** at 500 Hz on the second core:
1. **Gyro step:** pitch += (q − GYRO_QR·r)·dt.
2. **Correction:** subtract the IMU's own acceleration from the accelerometer X axis (ACCEL_COMP mode 2).
3. **Accelerometer nudge:** pitch += (dt/τ)·(accelerometer pitch − pitch). The nudge is skipped when the total acceleration is more than 0.15 g away from 1 g.
4. **Around it:** a 94 Hz filter on the sensor itself, gyro bias measured at power-up, and the KD term using the gyro rate directly.

**Kalman filter: a good next step, but not urgent now.** It would track the gyro's bias as a separate quantity and weight the accelerometer by how believable it is at the time: less while accelerating, more while still. So it can correct drift as well as τ 0.5 does with less sensitivity to acceleration. It also leads naturally to LQG (LQR plus a Kalman estimator), which is one of the controllers to compare. Now that the compensation is right, expect a modest improvement, not a transformation.

**The notch would still be needed.** The 20 Hz shake is a resonance in the drive train that the *controller* feeds through its output, whatever estimates the angle. A Kalman filter wouldn't remove it. Alternatives are a low-pass on the KD rate, or curing the resonance mechanically.

## 25. Can the Pi Zero flash the SKR Pico?

Yes. The Pi is the SKR's USB host (USB CDC serial), and the firmware uses the arduino-pico core, which supports the standard "1200-baud touch": opening the serial port at 1200 baud and closing it reboots the RP2040 into its USB bootloader.
1. Copy `firmware.uf2` to the Pi (as `deploy-pi.sh` does for the Python files).
2. Stop `robotd` so it releases the serial port.
3. Do the 1200-baud touch on `/dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_*`.
4. The board reappears as a USB drive (RPI-RP2). Mount it and copy the `.uf2` onto it, or use `picotool load -x firmware.uf2`, which also needs no button press.
5. Wait for the serial port to return, then restart `robotd`.

That would make "build on the Mac, flash over Wi-Fi" one script (`tools/flash-skr.sh`). Two cautions:
- **Only flash with the robot off the floor or on its training wheels.** It resets mid-flash.
- **If a bad build ever stops the USB port working,** the 1200-baud trick won't reach it. Then the BOOTSEL button (or a jumper) is the fallback, so keep that accessible.

## 26. What is LQR meant to do?

LQR stands for **Linear-Quadratic Regulator.** It's a way of *calculating* the controller's gains from a model of the robot, instead of tuning them by hand.

**What it does:**
- **The controller:** our robot already uses a full-state controller. It looks at four things, the angle, the angle rate, the wheel speed and the wheel position. Then it sets the wheel acceleration as one weighted sum: u = KP·θ + KD·θ̇ + KV·v + KX·x. LQR doesn't change that structure. It only chooses the four numbers.
- **How it chooses them:** you tell it what you care about, with weights:
  - **Q** says how much you dislike errors in each state. For example: "I really don't want the angle to wander", or "I don't mind drifting a few cm".
  - **R** says how much you dislike using the motors, meaning large or jerky accelerations.

  LQR then finds the gains that minimise the total "cost" over time, the sum of those weighted errors and that motor effort. For a linear model, the answer is exact and comes from one matrix equation (the Riccati equation).
- **Why it's attractive:** all four gains are balanced against each other automatically. With hand tuning, changing KP usually means re-tuning KD, KV and KX. With LQR you adjust one intuitive weight, such as "care more about position", and it reworks all four consistently. It also guarantees a stable result with good margins, but only *for the model you gave it*.

**For our robot:**
- **Where the original gains came from:** they were designed by pole placement, which is choosing where the response should sit directly.
- **What the LQR gains are:** KP 73.5, KD 4.63, KV 10, KX 11.8. They came from the rigid-pendulum model with Q = diag(10, 1, 300, 0.3) on [x, v, θ, θ̇] and R = 0.1. That means "care about the angle a lot, position moderately, and allow fairly brisk motor use". Compared with the old gains, they hold the angle and position much more firmly: the best standing run (23 Nov, τ 2.0) had half the pitch wander and wheel speed wander of step 2.
- **The catch:** LQR is only as good as the model behind it. Ours treats the robot as rigid, with ideal sensing. It knows nothing about:
  - the joint compliance (the 2.4 Hz wobble),
  - the pitch estimator's own dynamics (the 4–5 Hz problem),
  - the drive-train resonance (20 Hz),
  - or the notch filter.

  Firmer gains push the loop closer to those unmodelled effects. That has happened repeatedly with these gains: each time one missing piece is fixed, the higher gains run into the next. On 24 Nov it was a marginal ~5 Hz mode that the notch's small phase lag tipped over.

**The comparison study** asks: for a fair test (the same push, the same speed step), does a model-based controller (LQR) beat a hand-tuned one (the step 2 gains), and by how much? Part of the answer is already in: on a messy real robot, a model-based design pays off only once the model includes the important real effects. Hence the work on the model and the estimator.

## 27. Is the LQR model close? Is it only for static balancing? Is driving a fundamentally different problem?

**Is the model close?** For standing, the fitted model (11.1) was good. It reproduced the 2.4 Hz ring and its damping in four runs, predicted that the earlier redesign would be unstable, and tracked the trend through steps 1–2. But the model used to *design* the LQR gains was simpler: a rigid pendulum with ideal sensing and no delay. That is where it fell short. It knows nothing of:
- the 10 ms delay,
- the joint compliance,
- the estimator's own loop (the 4–5 Hz mode),
- the drive-train resonance (20 Hz),
- the notch.

Its gains were right in shape (much more KP, KV and KX; about the same KD), and they worked standing once the estimator was fixed. They had little margin, because the effects it left out all sit just above the bandwidth the gains asked for.

**Is it only for static balancing?** Mathematically LQR is a *regulator*: it drives the state to a set point (here x = x_ref, v = v_ref, θ = θ_ref). Driving is handled the same way, by moving the set point (v_ref ramps, x_ref integrates, plus the feed-forward lean and acceleration). So the same gains are meant to cover driving: the controller sees driving as "standing still relative to a moving reference". The LQR weights do decide how hard it tracks that reference. Our Q weighted position heavily, which makes the robot fight any lag in following v_ref. That is fine standing still, but harsh at speed.

**Is driving fundamentally different?** Not in the control law, but in the *plant*. These things change at speed, and none of them is in a standing-still model:
- **The motors:** steppers lose torque and damping as they speed up, and drive-train resonances (our 20 Hz) get excited.
- **Rolling drag and the drive lean:** the robot must hold a steady lean against drag.
- **The estimator:** it sees sustained accelerations and vibration (accelerometer rejects, τ trade-offs).
- **Turns:** they add yaw coupling into the gyro and shift the balance point (0.38°·r²).
- **The non-minimum-phase nature:** to go forward you must first roll back, so speed changes can't be instant and the speed loop is inherently slow.

So it is the same problem in a harder operating region. A design that is "optimal" standing can be fragile at speed, which is exactly what we saw. A robust design either includes those effects in the model, or deliberately keeps margin (softer gains, filters) where the model is weak.

## 28. If starting again with another balancing device: what have we learnt, what would make it easier, what order, and what tests before moving on?

**What we learnt:**
1. **Most of the hard problems were not the control law.** They were sign conventions, the balance angle, the estimator (the accelerometer compensation and τ), delay, mechanical compliance and drive-train resonance. Gain tuning was the easy part once those were right.
2. **Measure before modelling, and model before redesigning.** The kick test plus an ensemble fit was the single most useful tool. It turned vague "it wobbles" into numbers (frequency, damping, l_eff, delay) that could be compared run to run.
3. **Change one thing per run, and log everything.** The bands-by-speed tables and kick fits only worked because every run had the same structure. Record every parameter in the run's meta, including estimator and filter settings, and not just the gains. We lacked ACCEL_COMP and the notch in the logs.
4. **The robot changes during a session** (joint damping 0.07–0.43). Compare runs within one session, and repeat the baseline.
5. **Know where the IMU is.** Its height relative to l_eff decided whether the acceleration compensation was right. Measure it on day one.
6. **Standing and driving must both be tested** before a setting is called "best".
7. **A rigid, repeatable mechanical build is worth more than clever control:** IMU screwed down, battery strapped, motor mounts tight.

**What would make it easier:**
- A rigid frame and IMU mount, the IMU near the axle or at a known height, and wheel encoders (or BLDC motors with encoders) so the wheels' real motion can be checked against the command.
- A test stand: a pendulum "hammer" for repeatable pushes, and a way to hold the robot near upright.
- From the start, a firmware structure where every controller, estimator and filter parameter is runtime-settable and logged, with A/B switches (as ACCEL_COMP 0/1/2 and the notch ended up).
- Flashing from the Pi (question 25), so firmware iterations don't need the robot at the Mac.
- An analysis script for the standard plots and tables: kick ensemble and fit, band RMS by speed, estimate against accelerometer pitch, turn episodes.

**Sequence, and the gate to pass before moving on:**

| Stage | What to do | Gate (must pass) |
|---|---|---|
| 1. Mechanics | Rigid build; measure mass, CoM height, wheel diameter, IMU position (height above the axle, offset) | Nothing moves when shaken; numbers recorded |
| 2. Motors | Directions, steps/m, current, chopper mode; run to speed on a stand | A commanded move goes exactly that distance; no stalls up to max speed under load |
| 3. IMU | Axes and signs, noise, bias; filter type and τ; acceleration compensation with the measured IMU height | Correct signs; pitch matches a protractor; the estimate tracks the accelerometer within 0.2° while the robot is pushed around by hand |
| 4. Model | Balance angle, l_eff (swing test), loop delay | Values consistent between two methods |
| 5. First balance | Conservative gains from the model; arming window, soft start, tilt limits, runaway trip, black box | Stands 60 s; trips safely in every direction |
| 6. Identify | Kick tests; fit the model, delay and any modes | The model reproduces the kick response (frequency within 10 %, damping within ±0.05) |
| 7. Tune or design | Pole placement or LQR on the fitted model; step toward the new gains with a prediction per step | Each step measured within its prediction; no new mode appears |
| 8. Drive | Feed-forward, lean, speed ramp; drive tests at increasing speed | No new oscillation up to max speed (band RMS by speed); estimate within 0.3° of the accelerometer while driving |
| 9. Turn | Turn rate ramp, yaw coupling, turn lean | Forward speed stays near zero in a spin on the spot; no inner-wheel stall |
| 10. Robustness | +200 g, low battery, different floor, cold start | All gates above still pass |

**In short:** get the mechanics, signs and estimator right before balancing; identify before you design; and treat standing and driving as two separate gates.

## 29. How does our control system compare with Wouter Klop's balancingrobot?

From his `Software/src/main.cpp` (ESP32, MPU6050, two DRV8825-type step-stick drivers on 24 V; the version in `~/Downloads/balancingrobot-master`).

| | Klop | Ours |
|---|---|---|
| Processors | One ESP32 (control, Wi-Fi, web page, OTA) | SKR Pico RP2040 (control on core 1, motors and comms on core 0) plus Pi Zero 2 W (robotd, web, logging) |
| Loop rate | 200 Hz (5 ms), polled in `loop()` | 500 Hz, driven by the IMU data-ready interrupt |
| Controller structure | **Cascaded PIDs:** an outer loop gives an angle set point, and the inner angle PD gives a change in motor speed (an acceleration) | **One full-state law:** u = KP·θ + KD·θ̇ + KV·(v − v_ref) + KX·(x − x_ref) + feed-forward, with u the wheel acceleration |
| Outer loop | **Two modes:** position PD (K 1, Td 1.2) when there's no input; speed PI (K 6, Ti 5) while driving, back to position 2 s after the input stops (the step counters reset at each switch) | Always both speed and position, against a moving reference (v_ref ramps, x_ref integrates) |
| Inner loop | Angle PD (K 0.65, Td 0.075, derivative filter N 15), output limited to ±20; integrated into motor speed | The same idea: u is integrated into the wheel speed v |
| Integral action | In the speed loop (Ti 5) | None in the gains; an automatic trim estimator walks the balance angle instead |
| Driving | The stick input, low-pass filtered (0.9 per 5 ms), is the speed set point; no feed-forward | S-curve v_ref (jerk-limited), drive feed-forward (a_ref into u and a lean of a_ref/g), plus a drive lean for rolling drag |
| Steering | ± steer added directly to the two wheel speeds (open loop) | The same in effect (w_ref ramp, ± per wheel), plus the gyro yaw-coupling correction |
| Angle estimate | Complementary filter, α 0.996 at 200 Hz (τ ≈ 1.25 s); accelerometer from atan2(ay, az); **no acceleration compensation, no rejection** | Complementary filter, τ 0.5 s at 500 Hz, ACCEL_COMP mode 2 (the IMU's own acceleration), a ±0.15 g rejection gate, GYRO_QR turn correction |
| IMU settings | ±500 °/s; DLPF left at the library default (wide band); 400 kHz I²C | ±500 °/s, ±4 g; DLPF 94 Hz; 400 kHz I²C |
| Filters on the output | None | 20.5 Hz notch on the feedback part of u |
| Stepping | **Variable microstepping:** 1/32 at low speed, halving (down to full step) as speed rises, with hysteresis (switch at 150·32/µ, back at 130·32/µ steps/s); µ set by the driver pins | Fixed 1/16 (TMC2209 interpolates to 1/256), PIO step generation up to 20 k steps/s, StealthChop, current set over UART |
| Safety | Disable when tilted beyond 70°; re-enable when held within 0.5° of upright | Arming window and soft start, tilt limits at the training wheels, runaway trip, IMU-stale trip, drive lockout, leases on every drive command, e-stop, black box |
| Tuning and testing | Live PID parameters and plots over a WebSocket; an optional **noise source** injected into the output for system identification | Live BAL_SET, saved start-up tuning, kicks for identification, 250 Hz logs on the Pi, offline model fits |

**What's really the same:** a cascade of angle PD and position PD is also full-state feedback on angle, angle rate, position and speed. The inner output is integrated into wheel speed in both, so both command *acceleration*. The difference is in how it's parameterised and tuned. His cascade is intuitive to tune by hand, one loop at a time (inner first). Our single law fits model-based design (pole placement, LQR).

**Where his choices answer problems we met:**
1. **Separate speed and position modes.** While driving his robot has *no position loop*: just speed PI to an angle set point. That avoids "fighting the position error" at speed, which is what made our high-KX LQR set harsh while driving. The cost: a jump in behaviour at each mode switch (he resets the counters), and no position hold while driving.
2. **Integral in the speed loop:** it absorbs rolling drag and balance-point errors automatically. We do the same with the drive lean and the trim estimator.
3. **A longer filter time constant (1.25 s) with no compensation.** That is roughly where our τ 2.0 result pointed. It avoids the u/g compensation trap but drifts in sustained accelerations. With moderate gains and no high-gain angle loop, he probably never hit the 4–5 Hz estimator mode.
4. **Variable microstepping:** this is about speed. On 24 V with DRV8825s he chases very high wheel speeds. Coarser steps at speed keep the pulse rate inside what the ESP32 timer can generate, and give more torque per step. We have neither problem: PIO hardware for the pulses, TMC2209 interpolation, and our top speed is modest.
5. **A noise source for identification:** the same purpose as our kicks. Continuous noise gives a frequency response directly, which is a nice alternative to the kick ensemble.

**Where ours goes further:**
- **Model-based design and validation:** fitted model, predicted steps, LQR.
- **Estimator:** correct acceleration compensation using the IMU height, rejection gate, turn coupling.
- **Drive feed-forward and S-curve,** so the robot leans before it moves instead of after.
- **The notch for the drive-train resonance.**
- **Higher loop rate and lower delay:** 500 Hz with the 94 Hz DLPF, against 200 Hz on a wide-band DLPF, which also invites aliasing of motor vibration.
- **Much more safety and logging.**

**Worth borrowing:**
1. **The speed-mode idea while driving:** reduce or freeze KX when v_ref ≠ 0 and restore it at standstill, with x_ref re-anchored to avoid a jump. That would probably let the stiffer standing gains (LQR) coexist with smooth driving.
2. **A small integral term on speed** while driving.
3. **A noise-injection option** for frequency-response identification.

---

## 30. Is the best position for the IMU in line with the axles?

**For the accelerometer, yes: on the axle line, and on the centreline between the wheels.** For the gyro it doesn't matter. Every point of a rigid body turns at the same rate, so the gyro reads the same anywhere (apart from vibration).

The accelerometer measures specific force at its own position. Along the robot's forward axis, at height h above the axle:

  f_x ≈ a − g·θ + h·θ̈        (a = wheel acceleration; small angles)

- **The −g·θ term** is the tilt we want.
- **a** is the base acceleration. It's there at every height, and ACCEL_COMP corrects it from u.
- **h·θ̈** is the tangential acceleration from the pitch swinging, plus a smaller h·θ̇² term. ACCEL_COMP=2 corrects it with IMU_H·dq/dt, the noisiest part of the estimate, since it differentiates the gyro.

On the axle (h = 0) the last term vanishes and only the base acceleration is left, which we know well from u.

**Our 98 mm is about the worst height in principle.** It's almost exactly l_eff (94 mm), the centre of percussion (Q32). There, the dynamics give h·θ̈ ≈ g·θ − a, so f_x ≈ 0 whenever the robot is moving: the accelerometer x axis sees almost nothing during motion. That's why comp 1, which subtracts only a, was wrong at 2–5 Hz and produced the 4–6 Hz estimator mode. Comp 2 works because it puts the h·θ̈ term back.

**Sideways:** on the centreline the IMU sees no centripetal acceleration in spins. Off it, a spin adds r²·d along the offset.

**Practical:** on the axle line is often hard (the motors and wheels are there, with more vibration). A low mount as near the axle as practical, centred, rigid, and with IMU_H measured, is the goal. With comp 2, our height works, at the cost of a noisier rate term.

---

## 31. What has caused ringing?

| Ringing | Frequency | Cause | Fix |
|---|---|---|---|
| The lightly damped standing mode | 2.2–2.9 Hz | The balance loop plus joint compliance: battery strap, motor mounts, the wedged IMU. Damping started at about 0.3 after arming and fell to 0.05–0.1 within a minute; SpreadCycle was worse. | Higher KD and KV (everyday set); accepted for the prototype; tighten the mechanics |
| The estimator-loop mode | 4–6 Hz | ACCEL_COMP=1 subtracted u/g but not the IMU-height term (Q30), so the estimate fed back into the loop at 2–5 Hz | First masked by τ 2.0; fixed by ACCEL_COMP=2 at τ 0.5 |
| The drive shake | about 20 Hz | A drive-train resonance (motor, mount, tyre), excited at speed. τ 2.0 made it worse, because it trusts the gyro more and lets more of the shake into the loop. | 20.5 Hz notch on the feedback part of u |
| The slow fore–aft swing in turns | about 0.15–0.2 Hz (about 1 rad/s) | The position loop's slow mode, lightly damped with KV 6.89 / KX 6.48. The rolling drags cancel in a spin, so nothing physical damps it, and the tyre contact band releases a stored offset when the turn starts. | KV 10 / KX 11.8 (today) |
| Unstable from the start | — | A redesign on the wrong model (l 0.20 m instead of 0.094) | Fitted model, gains checked on it |

A pattern: **the controller law was rarely the cause.** The estimator, the delay and the mechanics were.

---

## 32. Is l_eff an effective pendulum length?

**Yes.** It's the length of the simple point-mass pendulum that has the same small-angle dynamics as the real robot on its wheels:

  θ̈ = (g·θ − a)/l_eff

For a real body with mass m, centre of mass at height l_c and moment of inertia J about the axle (wheel inertia adds small corrections):

  l_eff = J/(m·l_c) = l_c + k²/l_c

Here k is the radius of gyration about the centre of mass.
- **So l_eff is always longer than the centre-of-mass height.** Our l_eff of 94 mm means the CoM itself is lower.
- **It's also the centre of percussion:** the point that doesn't accelerate sideways when the base accelerates. That's why an accelerometer near 94 mm sees so little (Q30).
- **What it sets:** the falling speed. The unstable rate is √(g/l_eff) ≈ 10.2 rad/s, a time constant of 98 ms. Shorter is faster to fall and needs a faster controller.

We measured it from the kicks (9.4b): the response to a known acceleration pulse. The earlier design figure of 0.20 m was a guess at the CoM height, and the redesign on it was unstable.

---

## 33. What does τ 0.5 s or 2.0 s actually mean?

τ is the **crossover time constant of the complementary filter.**
- **The gyro path:** the gyro is integrated to track pitch.
- **The accelerometer path:** each sample, the estimate is pulled towards the accelerometer's pitch by dt/(τ + dt). In effect, the accelerometer is low-passed with time constant τ, and the gyro high-passed with the same τ.

| | τ 0.5 s | τ 2.0 s |
|---|---|---|
| Crossover frequency 1/(2πτ) | 0.32 Hz | 0.08 Hz |
| Faster than this: trust | the gyro | the gyro, down to slower |
| A constant accelerometer error (an uncorrected acceleration, a lean in a spin) shows fully in pitch after about 3τ | 1.5 s | 6 s |
| A gyro error (bias b, or a turn coupling c·r) leaves a steady pitch error of b·τ | 0.1 °/s → 0.05° | 0.1 °/s → 0.2° |
| Accelerometer noise and vibration let through | more | less |

**The trade:**
- **A larger τ** ignores more accelerometer disturbance. That's why τ 2.0 hid the 4–6 Hz mode caused by the wrong comp 1.
- **But it corrects gyro errors more slowly,** and it leaves a slower estimator dynamic in the loop. That interacted with the drive shake.

Once comp 2 made the accelerometer path correct, τ 0.5 became the better choice.

---

## 34. Is the notch filter still in use? Is it still needed?

**Still in use:** the firmware default is NOTCH_HZ 20.5, Q 2.0, on the feedback part of u (not on the feed-forward). The everyday-stiff comparison ran with it.

**Still needed, as far as we know:**
- The 20 Hz shake comes from the drive train, which hasn't changed.
- Runs without the notch at the same speed show it: the LQR comparison runs had a 20 Hz band of 2.4 °/s at 100 mm/s, against 0.7–0.9 with the notch.
- It's cheap: about 4° of phase at 3 Hz.

It hasn't been re-tested without the notch on the new defaults. The test would be one drive at 200–300 mm/s with `NOTCH_HZ=0`, comparing the 20 Hz band. If the mechanics are stiffened (motor mounts, harder tyres), the resonance may move; the notch would then need retuning, or might become unnecessary.

---

## 35. Do the stiffer settings for clean turns change any of the earlier assumptions?

Several, yes. The set is the everyday KP 51.4 / KD 4.37 with LQR's KV 10 / KX 11.8.

1. **Why LQR failed while driving.** We put LQR's driving trouble down to its whole gain set. KV 10 and KX 11.8 with the notch drove cleanly, and turned better. So the problem was more likely LQR's higher KP (73.5), or its τ 2.0 with no notch, than its outer gains.
2. **"Stopping accuracy needs integral action."** Not for now. The stiffer KX pulls through the tyre band: the final error was −7 mm against −106. KI added nothing in the turn tests, and is off.
3. **"The one-wheel turns are turn creep from the spin lean."** Revised. They are the slow position mode, released by the turn from the offset the robot was parked with. A turn-lean feed-forward may no longer be needed.
4. **"The everyday set is the best all-round."** The stiffer set is better on kicks, pushes, stopping, moving smoothness and turns. It is slightly worse standing (pitch RMS 0.138° against 0.116°), with more stop overshoot.
5. **Margins.** The everyday set was chosen as two-fifths of the way to LQR and checked on the fitted model. This mixed set wasn't designed on the model. Its stability margins should be checked there (phase11_model.py) before it becomes the default, alongside the 300 mm/s drive test.
6. **Unchanged:** the plant (l_eff, delay), the estimator (comp 2, τ 0.5), the notch, and the AMAX saturation on hard pushes.

---

## 36. Is it time for a Kalman filter?

**Not yet, in my view.** The problems left aren't estimator problems:
- the tyre contact band,
- the drive-train resonance,
- the joint compliance,
- dead reckoning without encoders.

The complementary filter with comp 2 already does what a Kalman filter would do with the same sensors: it uses the known wheel acceleration (u) as an input, and blends the gyro and accelerometer at a sensible crossover.

**What a Kalman filter would add:**
- **Gyro bias:** tracked continuously as a state, instead of by the auto-bias while still.
- **Adaptive trust:** it would trust the accelerometer less during accelerations and spins, instead of using the hard 0.15 g gate.
- **A principled noise trade-off,** instead of choosing τ by hand.
- **A clean path to LQG,** with the controller and estimator designed together on the fitted model. That's the "controller comparison" item.

**When it becomes worth it:**
- **When wheel encoders are fitted:** a Kalman filter is the natural way to fuse the true wheel motion (slip, missed steps) with the IMU and the commanded steps.
- **When you want the LQG comparison for its own sake.**

Before that, a rigid, low, centred IMU mount (Q30) and the mechanical tightening would gain more.
