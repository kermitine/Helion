#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAN_INTERFACE="${CAN_INTERFACE:-can0}"
HELION_TRANSPORT="${HELION_TRANSPORT:-robstride-serial}"
HELION_CAN_BACKEND="${HELION_CAN_BACKEND:-${CAN_BACKEND:-auto}}"
SLCAN_PORT="${SLCAN_PORT:-}"
SERIAL_PORT="${SERIAL_PORT:-auto}"
SERIAL_BAUD="${SERIAL_BAUD:-921600}"
BITRATE="${BITRATE:-1000000}"
RESTART_MS="${RESTART_MS:-100}"
TX_QUEUE_LEN="${TX_QUEUE_LEN:-2000}"
DASHBOARD_PORT="${DASHBOARD_PORT:-8080}"
DASHBOARD_HOST="${DASHBOARD_HOST:-0.0.0.0}"
SERVICE_USER="${HELION_USER:-${SUDO_USER:-$(id -un)}}"
SERVICE_GROUP="${HELION_GROUP:-$(id -gn "$SERVICE_USER")}"

escape_sh() {
  printf "%s" "$1" | sed "s/'/'\\\\''/g"
}

REPO_ESC="$(escape_sh "$REPO_DIR")"

sudo apt update
sudo apt install -y git python3 can-utils iproute2

case "$HELION_TRANSPORT" in
  robstride-serial|socketcan)
    ;;
  *)
    echo "Unknown HELION_TRANSPORT=$HELION_TRANSPORT; use robstride-serial or socketcan." >&2
    exit 1
    ;;
esac

if [[ "$HELION_TRANSPORT" == "robstride-serial" && "$SERVICE_USER" != "root" ]]; then
  sudo usermod -aG dialout "$SERVICE_USER" || true
fi

chmod +x "$REPO_DIR/raspi/robstride_socketcan.py"
chmod +x "$REPO_DIR/raspi/robstride_dashboard.py"
chmod +x "$REPO_DIR/raspi/update_from_github.sh"
chmod +x "$REPO_DIR/raspi/can_up.sh"
chmod +x "$REPO_DIR/raspi/can_down.sh"

sudo tee /usr/local/bin/helion-dashboard >/dev/null <<EOF
#!/usr/bin/env bash
set -euo pipefail
cd '$REPO_ESC'
exec /usr/bin/python3 raspi/robstride_dashboard.py "\$@"
EOF

sudo tee /usr/local/bin/helion-update >/dev/null <<EOF
#!/usr/bin/env bash
set -euo pipefail
cd '$REPO_ESC'
exec bash raspi/update_from_github.sh "\$@"
EOF

sudo tee /usr/local/bin/helion-can-up >/dev/null <<EOF
#!/usr/bin/env bash
set -euo pipefail
cd '$REPO_ESC'
exec bash raspi/can_up.sh "\$@"
EOF

sudo tee /usr/local/bin/helion-can-down >/dev/null <<EOF
#!/usr/bin/env bash
set -euo pipefail
cd '$REPO_ESC'
exec bash raspi/can_down.sh "\$@"
EOF

sudo chmod +x /usr/local/bin/helion-dashboard /usr/local/bin/helion-update /usr/local/bin/helion-can-up /usr/local/bin/helion-can-down

sudo tee /usr/local/sbin/helion-restart-dashboard >/dev/null <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
exec systemctl restart robstride-dashboard.service
EOF
sudo chmod +x /usr/local/sbin/helion-restart-dashboard

if [[ "$SERVICE_USER" != "root" ]]; then
  sudo tee /etc/sudoers.d/helion-dashboard >/dev/null <<EOF
$SERVICE_USER ALL=(root) NOPASSWD: /usr/local/sbin/helion-restart-dashboard
EOF
  sudo chmod 440 /etc/sudoers.d/helion-dashboard
  sudo visudo -cf /etc/sudoers.d/helion-dashboard >/dev/null
fi

sudo tee /etc/systemd/system/robstride-can.service >/dev/null <<EOF
[Unit]
Description=RobStride SocketCAN interface
After=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
Environment=HELION_CAN_BACKEND=$HELION_CAN_BACKEND
Environment=SLCAN_PORT=$SLCAN_PORT
Environment=BITRATE=$BITRATE
Environment=RESTART_MS=$RESTART_MS
Environment=TX_QUEUE_LEN=$TX_QUEUE_LEN
ExecStart=/usr/local/bin/helion-can-up $CAN_INTERFACE
ExecStop=/usr/local/bin/helion-can-down $CAN_INTERFACE

[Install]
WantedBy=multi-user.target
EOF

if [[ "$HELION_TRANSPORT" == "robstride-serial" ]]; then
  sudo tee /etc/systemd/system/robstride-dashboard.service >/dev/null <<EOF
[Unit]
Description=RobStride web dashboard
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=$SERVICE_USER
Group=$SERVICE_GROUP
SupplementaryGroups=dialout
NoNewPrivileges=false
ExecStart=/usr/local/bin/helion-dashboard --host $DASHBOARD_HOST --port $DASHBOARD_PORT --transport robstride-serial --interface $CAN_INTERFACE --serial-port $SERIAL_PORT --serial-baud $SERIAL_BAUD
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
else
  sudo tee /etc/systemd/system/robstride-dashboard.service >/dev/null <<EOF
[Unit]
Description=RobStride web dashboard
After=network-online.target robstride-can.service
Wants=network-online.target robstride-can.service

[Service]
Type=simple
User=$SERVICE_USER
Group=$SERVICE_GROUP
AmbientCapabilities=CAP_NET_RAW
CapabilityBoundingSet=CAP_NET_RAW
NoNewPrivileges=false
ExecStart=/usr/local/bin/helion-dashboard --host $DASHBOARD_HOST --port $DASHBOARD_PORT --transport socketcan --interface $CAN_INTERFACE
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
fi

sudo systemctl daemon-reload
if [[ "$HELION_TRANSPORT" == "robstride-serial" ]]; then
  sudo systemctl disable --now robstride-can.service 2>/dev/null || true
else
  sudo systemctl enable --now robstride-can.service
fi
sudo systemctl enable --now robstride-dashboard.service

echo "Dashboard installed."
echo "Open: http://$(hostname -I | awk '{print $1}'):${DASHBOARD_PORT}"
echo "Update later with: helion-update"
echo "Web updates run as user: ${SERVICE_USER}"
echo "Transport: ${HELION_TRANSPORT}"
if [[ "$HELION_TRANSPORT" == "robstride-serial" ]]; then
  echo "RobStride serial adapter: ${SERIAL_PORT} at ${SERIAL_BAUD} baud"
else
  echo "CAN backend: ${HELION_CAN_BACKEND}${SLCAN_PORT:+ via ${SLCAN_PORT}}"
fi
