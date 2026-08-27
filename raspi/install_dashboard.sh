#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERIAL_PORT="${SERIAL_PORT:-auto}"
SERIAL_BAUD="${SERIAL_BAUD:-921600}"
DASHBOARD_PORT="${DASHBOARD_PORT:-80}"
DASHBOARD_HOST="${DASHBOARD_HOST:-0.0.0.0}"
SERVICE_USER="${HELION_USER:-${SUDO_USER:-$(id -un)}}"
SERVICE_GROUP="${HELION_GROUP:-$(id -gn "$SERVICE_USER")}"

escape_sh() {
  printf "%s" "$1" | sed "s/'/'\\\\''/g"
}

REPO_ESC="$(escape_sh "$REPO_DIR")"

DASHBOARD_CAPABILITY_LINES=""
if (( DASHBOARD_PORT < 1024 )); then
  DASHBOARD_CAPABILITY_LINES="AmbientCapabilities=CAP_NET_BIND_SERVICE
CapabilityBoundingSet=CAP_NET_BIND_SERVICE"
fi

sudo apt update
sudo apt install -y git python3

if [[ "$SERVICE_USER" != "root" ]]; then
  sudo usermod -aG dialout "$SERVICE_USER" || true
fi

chmod +x "$REPO_DIR/raspi/robstride_usb.py"
chmod +x "$REPO_DIR/raspi/robstride_dashboard.py"
chmod +x "$REPO_DIR/raspi/update_from_github.sh"

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

sudo tee /usr/local/bin/helion-robstride >/dev/null <<EOF
#!/usr/bin/env bash
set -euo pipefail
cd '$REPO_ESC'
exec /usr/bin/python3 raspi/robstride_usb.py "\$@"
EOF

sudo chmod +x /usr/local/bin/helion-dashboard /usr/local/bin/helion-update /usr/local/bin/helion-robstride
sudo rm -f /usr/local/bin/helion-can-up /usr/local/bin/helion-can-down

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

sudo systemctl disable --now robstride-can.service 2>/dev/null || true
sudo rm -f /etc/systemd/system/robstride-can.service

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
$DASHBOARD_CAPABILITY_LINES
NoNewPrivileges=false
ExecStart=/usr/local/bin/helion-dashboard --host $DASHBOARD_HOST --port $DASHBOARD_PORT --serial-port $SERIAL_PORT --serial-baud $SERIAL_BAUD
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now robstride-dashboard.service

PI_IP="$(hostname -I | awk '{print $1}')"
if [[ "$DASHBOARD_PORT" == "80" ]]; then
  DASHBOARD_URL="http://${PI_IP}"
else
  DASHBOARD_URL="http://${PI_IP}:${DASHBOARD_PORT}"
fi

echo "Dashboard installed."
echo "Open: ${DASHBOARD_URL}"
echo "Update later with: helion-update"
echo "Dashboard service runs as user: ${SERVICE_USER}"
echo "RobStride USB adapter: ${SERIAL_PORT} at ${SERIAL_BAUD} baud"
