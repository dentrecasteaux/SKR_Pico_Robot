# Robot backlog

The balancing robot is commissioned and runs the everyday configuration (see [docs/engineering-reference.md](docs/engineering-reference.md), sections 8–9). The longer-term plan is in [docs/roadmap.md](docs/roadmap.md).

Next candidates:

- turn-lean feed-forward for spins (θ_ref += 0.38°·(w_ref in rad/s)²), with an on/off setting;
- confirm the iPhone drive fix (1.5 s hold, end reasons);
- log every controller, estimator and filter setting in each run's `meta.json`;
- the Phase 11 comparison protocol (repeatable push, speed step, sensitivity), with the everyday set as the baseline;
- flash the SKR from the Pi (1200-baud touch + `picotool`);
- permanent rigid IMU mount at a known height; tighten the battery strap and motor mounts;
- battery voltage alarm, then sensing;
- a physical emergency stop;
- wheel encoders;
- a Kalman or LQG estimator for the controller comparison.
