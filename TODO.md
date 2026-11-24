# Robot backlog

The balancing robot is commissioned and runs the everyday configuration (see [docs/engineering-reference.md](docs/engineering-reference.md), sections 8–9). The longer-term plan is in [docs/roadmap.md](docs/roadmap.md).

Added on 24 Nov and used on the robot: profiled `MOVE` / `TURN` while balancing, the pitch rate without the roll term, the position integral (`KI`, off by default) and `SPIN_WAIT` (off by default).

Next candidates:

- confirm the SKR runs the final 24 Nov build (`bal-status` shows `SPIN_WAIT=0`, `KV=6.890`, `KX=6.480`);
- confirm the iPhone drive fix (1.5 s hold, end reasons);
- one-wheel look in turns (accepted for now): options if revisited are the stiffer outer gains while turning only, a turn feed-forward, or harder, narrower tyres (docs/phase11-model.md);
- stopping accuracy: the robot can park up to about 0.3 m short in the tyre contact band; `KI` didn't help in the turn tests, and the stiffer KX did but looked jittery at rest;
- a physical push test (pendulum hammer) and the sensitivity tests, to finish the comparison protocol;
- first use of `tools/flash-skr.sh` (flash the SKR from the Pi over Wi-Fi);
- permanent rigid IMU mount, as low and central as practical (RandomQuestions Q30); tighten the battery strap and motor mounts;
- battery voltage alarm, then sensing;
- a physical emergency stop;
- wheel encoders, then a Kalman or LQG estimator (RandomQuestions Q36).
