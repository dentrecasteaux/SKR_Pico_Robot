# Robot firmware roadmap

The Raspberry Pi executive-controller milestone is complete: the versioned
USB protocol, Pi service and local web interface are working.

The maintained longer-term plan is now in [docs/roadmap.md](docs/roadmap.md).

Next candidates:

- design and fit a physical emergency stop;
- select and integrate wheel encoders;
- design battery sensing and low-voltage behaviour;
- add automated protocol/parser tests;
- add IMU and odometry after encoder feedback is stable.
