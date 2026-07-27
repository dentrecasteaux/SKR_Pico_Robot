# Robot firmware roadmap

## Next milestone: Raspberry Pi Zero 2 W executive controller

Use the Pi as the high-level controller and the SKR Pico as the real-time
motor controller.

Planned work:

- Select the physical Pi-to-SKR communications link.
- Define a small, versioned command/status protocol.
- Add a Pico communications module without changing the motor-control API.
- Add a Pi-side service that sends motion commands and monitors Pico status.
- Define safe behaviour for a communications loss.

## Deferred hardware work

### Wheel encoders

On hold until encoder hardware is selected and fitted. They will provide
closed-loop wheel speed control and odometry.

### Battery status

On hold until the battery-voltage divider and any current-sensing hardware are
finalised. This will provide voltage monitoring, low-voltage warnings, and a
safe motor-disable threshold.
