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
- Recommended working value: **194 mm**

The wider timing-derived value is likely affected by manual stopwatch timing
and acceleration. The physical centreline measurement and later turn result
both support 194 mm as the current baseline.

## Current temporary configuration

At the time this note was written, `Config.h` still contains the last
experimental values: 62 mm wheel diameter and 201 mm track. Before the next
physical test session, set it to the recommended 61 mm and 194 mm baseline if
that remains the chosen calibration.
