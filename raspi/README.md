# RobStride On Raspberry Pi

This folder contains the Raspberry Pi command-line tool and Pi-local web
dashboard:

```bash
python3 raspi/robstride_socketcan.py --transport robstride-serial --serial-port auto
python3 raspi/robstride_dashboard.py --transport robstride-serial --serial-port auto --host 0.0.0.0 --port 8080
```

It has no Python package dependencies. It can either talk directly to the
official RobStride USB-CAN serial adapter or use Linux raw SocketCAN for native
CAN interfaces.

## Official RobStride USB-CAN Adapter

If the adapter appears as a CH340 serial device (`/dev/ttyUSB0`), use the direct
RobStride serial transport. Do not run `slcand` for this adapter.

```bash
sudo pkill -f slcand || true
python3 raspi/robstride_socketcan.py --transport robstride-serial --serial-port auto --command scan
```

The dashboard installer now defaults to this direct serial path:

```bash
bash raspi/install_dashboard.sh
```

To be explicit:

```bash
HELION_TRANSPORT=robstride-serial SERIAL_PORT=auto SERIAL_BAUD=921600 bash raspi/install_dashboard.sh
```

The installer adds the dashboard service user to the `dialout` group so it can
open `/dev/ttyUSB0`.

## SocketCAN Adapters

Bring up a SocketCAN-compatible adapter at 1 Mbps first:

```bash
sudo bash raspi/can_up.sh can0
```

Then run the tools with `--transport socketcan`:

```bash
python3 raspi/robstride_socketcan.py --transport socketcan --interface can0 --command scan
```

Some third-party adapters use SLCAN. For those, and only those, you can still
create `can0` with:

```bash
sudo HELION_CAN_BACKEND=slcan SLCAN_PORT=/dev/ttyUSB0 bash raspi/can_up.sh can0
python3 raspi/robstride_socketcan.py --transport socketcan --interface can0 --command scan
```

`ip -details link show can0` may show `bitrate 0` for SLCAN adapters. That is
normal; the CAN bitrate is selected by `slcand -s8` for 1 Mbps.

## Dashboard Install

From a fresh Pi:

```bash
sudo apt update
sudo apt install -y git
git clone https://github.com/kermitine/Helion.git
cd Helion
bash raspi/install_dashboard.sh
```

The installer creates and enables `robstride-dashboard.service`, so the
dashboard starts automatically every time the Pi boots.

For the official RobStride CH340 serial adapter, install the services with:

```bash
HELION_TRANSPORT=robstride-serial SERIAL_PORT=auto bash raspi/install_dashboard.sh
```

For a native SocketCAN adapter instead:

```bash
HELION_TRANSPORT=socketcan CAN_INTERFACE=can0 bash raspi/install_dashboard.sh
```

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
helion-can-up can0
helion-can-down can0
helion-dashboard --transport robstride-serial --serial-port auto --host 0.0.0.0 --port 80
helion-update
```

`helion-update` pulls the current GitHub branch with `git pull --ff-only`,
checks the Python files, and restarts the dashboard service.

The web UI shows the dashboard version in the header. When changing dashboard or
Raspberry Pi backend code, increment `APP_VERSION` in
`raspi/robstride_dashboard.py` before committing so the Pi page makes it obvious
which update is running.

Use SSH for the initial install and for service-level changes such as port,
Linux permissions, or systemd edits. After that, normal dashboard/backend/UI code
updates can be applied from the web UI with **Update From GitHub**, as long as
the new code has already been committed and pushed to GitHub.

Normal terminal update flow:

```bash
# On your dev machine
git add .
git commit -m "Update RobStride dashboard"
git push

# On the Pi
helion-update
```

Normal web update flow:

```bash
# On your dev machine
git add .
git commit -m "Update RobStride dashboard"
git push
```

Then open the dashboard and press **Update From GitHub** in the Repository
panel. The dashboard will run the pull/check/restart sequence in the
background. The page may disconnect for a few seconds while the service
restarts.

To apply a service setting change, such as switching an existing install to port
`80`, pull the latest repo and rerun the installer:

```bash
cd ~/Helion
git pull
bash raspi/install_dashboard.sh
```

Keep the dashboard on a trusted local network only. It can move the motor and
pull executable code from the configured Git remote.

Useful service checks:

```bash
systemctl status robstride-dashboard.service
systemctl is-enabled robstride-dashboard.service
journalctl -u robstride-dashboard.service -f
ls -l /dev/serial/by-id /dev/ttyUSB* /dev/ttyACM*
```

Useful first tests:

```bash
python3 raspi/robstride_socketcan.py --self-test
python3 raspi/robstride_socketcan.py --transport robstride-serial --serial-port auto --command scan
python3 raspi/robstride_socketcan.py --transport robstride-serial --serial-port auto --command configure
python3 raspi/robstride_socketcan.py --transport robstride-serial --serial-port auto --command jog-right
```

The default protocol is RobStride private extended-ID mode with motor `0x7F`
and host `0xFD`. To try the MotorBridge MIT-standard path instead:

```bash
python3 raspi/robstride_socketcan.py --transport robstride-serial --serial-port auto --protocol mit
```

For a one-shot MIT-standard jog:

```bash
python3 raspi/robstride_socketcan.py --transport robstride-serial --serial-port auto --protocol mit --command jog-right
```

Interactive commands: `p`, `v`, `f`, `b`, `<`, `>`, `g`, `0`, `s`, `+`, `-`,
`e`, `r`, `a`, `m`, `x`, `d`, `c`, `h`, `t`, `?`, and `q`.
