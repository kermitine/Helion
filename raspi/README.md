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

Those motor targets are sent as private-protocol position references to the
base, shoulder, and elbow motor IDs.

Interactive command keys for `robstride_usb.py`: `p`, `v`, `f`, `b`, `<`, `>`,
`g`, `0`, `s`, `+`, `-`, `e`, `r`, `a`, `x`, `d`, `c`, `h`, `t`, `?`, and `q`.
