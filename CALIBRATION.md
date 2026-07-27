# Initial drive calibration

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

## Current configuration

Later finite distance and angle tests established the present firmware values:

- effective wheel diameter: **61 mm**
- effective wheel track: **188 mm**

These are the values in `Config.h` and are the current source of truth. The
194–201 mm figures above are retained as useful measurement history. Open-loop
geometry remains provisional and should be rechecked with repeatable trials,
then refined when encoders are installed.
