# HelionOS

HelionOS is Raspberry Pi motion-control software for RobStride motors using the
official USB-CAN adapter and a local web control surface.

The current project is Pi-focused:

- `raspi/` contains the RobStride USB private-protocol command-line tool,
  HelionOS web app, install script, and GitHub update script.
- `motorbridge-main/` is the reference MotorBridge code and documentation kept
  for comparison and future integration work.

Start with [raspi/README.md](raspi/README.md) for the fresh Raspberry Pi setup,
HelionOS install, USB adapter checks, and update flow.
