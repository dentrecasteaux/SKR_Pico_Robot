# Initial drive calibration

## Balancing chassis (current, 22 Nov 2026)

- **Wheel diameter: 62.33 mm.** Loaded roll test, 1958 mm in 10 revolutions.
  Checked with `move 500` on the floor: 500 mm exactly, straight.
- **Wheel track: 223 mm.** Measured tyre-centreline distance. A `turn 90`
  on the training wheels gave 94.5° by IMU yaw, which is affected by scrub,
  so the track is not yet turn-calibrated.
- 1/16 microstepping: 16,343 microsteps/m.

The sections below are the three-wheeler history.

These values are deliberately provisional. They are sufficient for early
bench testing and can be refined later with repeatable distance and angle
tests, or with encoders.

## Wheel diameter

- Nominal wheel diameter: 65 mm
- Bench-test effective range: 61–62 mm
- Recommended working value: **61 mm**

Evidence: with a 62 mm configuration, `move 500` travelled 508 mm. Scaling
the configured diameter by `500 / 508` gives approximately 61 mm.

## Wheel track

- Physical wheel-centreline measurement: 194 mm
- Timing-test effective range: 194–201 mm
- Historical working estimate: **194 mm**

The wider timing-derived value is likely affected by manual stopwatch timing
and acceleration. The physical centreline measurement and later turn result
both supported 194 mm as an intermediate baseline.

## Three-wheeler final configuration (historical)

Later finite distance and angle tests established the three-wheeler's firmware values:

- effective wheel diameter: **61 mm**
- effective wheel track: **188 mm**

These were the three-wheeler's `Config.h` values. `Config.h` now holds the
balancing-chassis values above and remains the source of truth. The
194–201 mm figures above are retained as useful measurement history. Open-loop
geometry remains provisional and should be rechecked with repeatable trials,
then refined when encoders are installed.
