#pragma once

// Application entry point. Device ownership will move here incrementally as
// further subsystems (battery, IMU, encoders) are introduced.
class Robot
{
public:
  void begin();
  void update();
};
