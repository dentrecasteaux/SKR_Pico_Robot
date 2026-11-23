# Robot backlog

The balancing robot is commissioned and runs the everyday configuration (see [docs/engineering-reference.md](docs/engineering-reference.md), sections 8–9). The longer-term plan is in [docs/roadmap.md](docs/roadmap.md).

Next candidates:

- turn-lean feed-forward for spins (θ_ref += 0.38°·(w_ref in rad/s)²), with an on/off setting;
- confirm the iPhone drive fix (1.5 s hold, end reasons);
- log every controller, estimator and filter setting in each run's `meta.json`;
- a position integral for accurate stopping (anti-windup, active only when stopped), the same for every controller; the comparison (done 24 Nov) showed the robot parks up to 0.5 m short in the tyre contact band, and a faster trim walk doesn't fix it;
- a physical push test (pendulum hammer) and the sensitivity tests, to finish the comparison protocol;
- flash the SKR from the Pi (1200-baud touch + `picotool`);
- permanent rigid IMU mount at a known height; tighten the battery strap and motor mounts;
- battery voltage alarm, then sensing;
- a physical emergency stop;
- wheel encoders;
- a Kalman or LQG estimator for the controller comparison.
