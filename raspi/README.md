# RobStride On Raspberry Pi

This folder contains the Raspberry Pi command-line tool and Pi-local web
dashboard for the official RobStride USB-CAN adapter using RobStride private
extended-ID control.

```bash
python3 raspi/robstride_usb.py --serial-port auto
python3 raspi/robstride_dashboard.py --serial-port auto --host 0.0.0.0 --port 8080
```

The adapter usually appears as a CH340 serial device such as `/dev/ttyUSB0`.
The tools auto-detect `/dev/serial/by-id/*`, `/dev/ttyUSB*`, and
`/dev/ttyACM*`.

## USB Adapter

Scan for private-protocol RobStride motors:

```bash
python3 raspi/robstride_usb.py --serial-port auto --command scan
```

Useful adapter checks:

```bash
ls -l /dev/serial/by-id /dev/ttyUSB* /dev/ttyACM*
python3 raspi/robstride_usb.py --self-test
```

The default private-protocol motor ID is `0x7F` and host ID is `0xFD`.

## Dashboard Install

From a fresh Pi:

```bash
sudo apt update
sudo apt install -y git
git clone https://github.com/kermitine/Helion.git
cd Helion
bash raspi/install_dashboard.sh
```

To be explicit about the adapter:

```bash
SERIAL_PORT=auto SERIAL_BAUD=921600 bash raspi/install_dashboard.sh
```

The installer creates and enables `robstride-dashboard.service`, so the
dashboard starts automatically every time the Pi boots. It also adds the
dashboard service user to the `dialout` group so it can open the USB adapter.
If an older install created CAN helper wrappers or `robstride-can.service`, the
installer disables and removes those stale pieces.

Open the dashboard from another machine on the same network:

```text
http://<pi-ip-address>
```

The installed service listens on port `80` by default, which is why the browser
URL does not need `:8080`. The service grants the Python dashboard only the
`CAP_NET_BIND_SERVICE` capability needed to bind that low port. To keep using
the old explicit port instead, reinstall with `DASHBOARD_PORT=8080`.

The install creates these commands:

```bash
helion-dashboard --serial-port auto --host 0.0.0.0 --port 80
helion-robstride --serial-port auto --command scan
helion-update
```

`helion-update` pulls the current GitHub branch with `git pull --ff-only`,
checks the Python files, and restarts the dashboard service.

If the Pi has local edits, `helion-update` saves them in a Git stash before
pulling so the update can continue without overwriting those changes. To disable
that behavior for a one-off update:

```bash
HELION_UPDATE_AUTO_STASH=0 helion-update
```

Existing installs can use `helion-update` for the USB/private-only migration.
After the update is running, rerun `bash raspi/install_dashboard.sh` when you
want the installed systemd service and helper commands cleaned up to match the
new layout.

The web UI shows the dashboard version in the header. When changing dashboard or
Raspberry Pi backend code, increment `APP_VERSION` in
`raspi/robstride_dashboard.py` before committing so the Pi page makes it obvious
which update is running.

Use **Shutdown Pi** in the dashboard before removing Raspberry Pi power. The
button stops the arm motors, saves dashboard values, flushes the filesystem, and
then requests Linux poweroff. Wait for the Pi activity LED to stop blinking
before cutting power. The installer grants the dashboard user passwordless sudo
for `/usr/local/sbin/helion-poweroff` and restarts the dashboard service so the
new sudo rule is used. If the button reports interactive authentication, update
again and rerun `bash raspi/install_dashboard.sh`; the dashboard log will list
the Linux user and every shutdown command it tried.

## MG90S Gripper

The dashboard has an **MG90S Gripper** panel for a small PWM servo gripper. It
uses BCM GPIO numbering and defaults to GPIO `18` on physical pin `12`, with a
50 Hz servo signal and conservative `1000..2000 us` pulse bounds. Gripper
commands do not require the RobStride USB-CAN adapter to be online.

Use the panel's **Guide** button for the built-in wiring and calibration flow.
In short: connect the servo signal wire to the selected GPIO pin, power the
servo from a regulated 5 V supply sized for the gripper load, and tie servo
ground, supply ground, and Raspberry Pi ground together. Avoid powering a loaded
servo directly from the Pi 5 V pin if it causes brownouts or resets.

For calibration, use **Test Angle** and **Move Angle** to find the fully closed
and fully open positions without forcing the linkage. Press **Closed Here** and
**Open Here** to store those angles, then use the position slider: `0%` maps to
the closed angle and `100%` maps to the open angle. Use **Release** to stop the
servo PWM output, or enable **Release After Move** if you want the dashboard to
pulse the servo briefly without holding torque. Press **Save Values** after
calibration so the GPIO pin, pulse bounds, and open/closed angles load on the
next dashboard start.

Use SSH for the initial install, service-level changes such as port, Linux
permissions or systemd edits, and code updates.

Normal update flow:

```bash
# On your dev machine
git add .
git commit -m "Update RobStride dashboard"
git push

# On the Pi
helion-update
```

To apply a service setting change, such as switching an existing install to port
`80`, pull the latest repo and rerun the installer:

```bash
cd ~/Helion
git pull
bash raspi/install_dashboard.sh
```

Keep the dashboard on a trusted local network only. It can move the motor and
change saved motor setup values.

Useful service checks:

```bash
systemctl status robstride-dashboard.service
systemctl is-enabled robstride-dashboard.service
journalctl -u robstride-dashboard.service -f
ls -l /dev/serial/by-id /dev/ttyUSB* /dev/ttyACM*
```

Useful first tests:

```bash
python3 raspi/robstride_usb.py --self-test
python3 raspi/robstride_usb.py --serial-port auto --command scan
python3 raspi/robstride_usb.py --serial-port auto --command configure
python3 raspi/robstride_usb.py --serial-port auto --command jog-right
```

If the dashboard opens but no motors appear, wait a few seconds and press
**Scan Private** before rebooting the Pi. The dashboard also runs a delayed
startup scan automatically. If a scan finds no motors, it reopens the RobStride
USB-CAN adapter and retries, which usually fixes boot timing races between the
Pi, adapter, and motor power.

## Arm IK

The dashboard Arm IK panel solves a three-axis base/shoulder/elbow arm in the
browser as you edit the values. The canvas uses the same link lengths, target,
elbow-up setting, joint offsets, and motor directions that will be sent by
**Move IK**.

The setup wizard can split total reach into link lengths, nudge or preset the
target point, flip joint directions, set offsets from the currently solved pose
with **Zero Here**, and home the arm with **Home Zero**.

The **Safety** step sets link radii for self-collision checks and per-joint
maximum twist for wire protection. A `180` degree twist limit allows each joint
to move from `-180` to `+180` degrees around the homed zero pose. IK angles are
routed to an equivalent physical angle inside that window, so a move from `+179`
degrees to `-179` degrees is broken into safe waypoints back through zero
instead of slipping past the `+180` degree wire limit.

For homing, manually move the arm to its mechanical/kinematic zero pose, then
press **Home Zero**. The dashboard disables the arm motors, reads each motor's
current private-protocol position, stores those positions as the IK offsets, and
saves them for the next dashboard start.

The quick **Home** target preset is separate from **Home Zero**: **Home** points
the arm vertically up at `x=0`, `y=0`, `z=link1+link2`, while **Home Zero** keeps
the flat-forward zero pose.

The **Files** step can save the current dashboard values on the Pi, download
them as JSON, or upload a JSON values file. Saved values are loaded on dashboard
startup from:

```text
~/.config/helion/dashboard-values.json
```

For each IK move, the backend computes:

```text
motor_target = joint_offset + direction * solved_joint_angle
```

Those motor targets are sent as private-protocol operation-control targets to
the base, shoulder, and elbow motor IDs.

Target editing does not silently clamp negative `z` values; the IK preview will
show them and report safety warnings. Move commands still require the solved arm
geometry to stay at or above the base plane, so a `180` degree shoulder twist
limit allows horizontal-forward through vertical to horizontal-backward motion
without allowing the arm to route underneath the horizontal plane.

Loaded arms use RobStride operation-control frames for IK moves so the hold loop
has explicit damping and feed-forward torque. IK targets are streamed as small
smootherstep route samples with velocity feed-forward; `Velocity Limit` and
`Acceleration` control the planned route duration. The dashboard defaults to
`0.35 rad/s`, `2.5 rad/s^2`, `Kp=4.0`, `Kd=2.0`, and `4 A`, and caps saved arm
values at `1.5 rad/s`, `8 rad/s^2`, `Kp=10.0`, `Kd=5.0`, and `+/-5 Nm` assist
torque. Motion presets are generated from the current link lengths, elbow bend,
twist limits, link radii, and reach envelope. Their accepted poses are streamed
as one continuous spline, so the arm flows through the preset instead of
stopping at each named pose. Lower `Velocity Limit`/`Acceleration` values make
presets slower and smoother too. Active routes start from the current commanded
hold target and keep stronger assist while moving in the load-bearing direction,
so gravity compensation does not suddenly disappear at the start of a move. If
the arm starts bouncing, press **Stop Arm**, support the load, raise
`Damping Kd`, then adjust `Position Kp` only as needed for hold stiffness. If the
arm slowly falls even when stable, raise current limit and add signed shoulder or
elbow assist torque. If feedback reseeding finds the arm already slightly below
the base plane, the route planner allows a limited recovery path back to a safe
target instead of blocking the move at the first below-plane waypoint.

**Adaptive Assist** adds a slow learned shoulder/elbow trim on top of the manual
assist values. It learns only while the arm is holding still near the target,
waits briefly after each routed move, ignores large errors or fast feedback, and
clamps the runtime trim to `+/-2 Nm` per joint. The learned trim is intentionally
not saved; it resets when adaptive assist is toggled, the arm is stopped, faults
are cleared, Home Zero runs, or the arm's joint/motor/direction setup changes.

To set it up, first make IK stable with Adaptive Assist off. Start with manual
`Shoulder Assist Nm` and `Elbow Assist Nm` at `0.0` or a very small known-good
baseline, move to a reachable hold pose, then turn **Adaptive Assist** on. The
**Learned Trim** readout should change slowly as the arm settles. If it climbs to
the `+/-2 Nm` cap, raise current limit or add a little manual assist baseline. If
the trim moves the joint the wrong way, press **Stop Arm** and verify that joint's
direction/home zero before trying again.

Beginner loaded-arm tuning:

```text
Velocity Limit: 0.20 rad/s
Acceleration: 0.50 rad/s^2
Current Limit: 4.00 A
Position Kp: 4.0
Damping Kd: 2.0
Shoulder Assist Nm: 0.3, then add 0.2 Nm at a time
Elbow Assist Nm: 0.0, then add only if the forearm sags
```

Use the smallest assist torque that holds the arm near the target. If assist
makes the joint move the wrong way, flip its sign. If the arm overshoots or
bounces, raise `Damping Kd` before raising `Position Kp`; if the arm is stable
but droopy, raise current limit or assist torque before raising `Position Kp`.

Interactive command keys for `robstride_usb.py`: `p`, `v`, `f`, `b`, `<`, `>`,
`g`, `0`, `s`, `+`, `-`, `e`, `r`, `a`, `x`, `d`, `c`, `h`, `t`, `?`, and `q`.
