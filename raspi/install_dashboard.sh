#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CAN_INTERFACE="${CAN_INTERFACE:-can0}"
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

chmod +x "$REPO_DIR/raspi/robstride_socketcan.py"
chmod +x "$REPO_DIR/raspi/robstride_dashboard.py"
chmod +x "$REPO_DIR/raspi/update_from_github.sh"
chmod +x "$REPO_DIR/raspi/can_up.sh"

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

sudo chmod +x /usr/local/bin/helion-dashboard /usr/local/bin/helion-update /usr/local/bin/helion-can-up

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
ExecStart=/usr/local/bin/helion-can-up $CAN_INTERFACE
ExecStop=/sbin/ip link set $CAN_INTERFACE down

[Install]
WantedBy=multi-user.target
EOF

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
ExecStart=/usr/local/bin/helion-dashboard --host $DASHBOARD_HOST --port $DASHBOARD_PORT --interface $CAN_INTERFACE
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now robstride-can.service
sudo systemctl enable --now robstride-dashboard.service

echo "Dashboard installed."
echo "Open: http://$(hostname -I | awk '{print $1}'):${DASHBOARD_PORT}"
echo "Update later with: helion-update"
echo "Web updates run as user: ${SERVICE_USER}"
