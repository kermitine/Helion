#!/usr/bin/env bash
set -euo pipefail

CAN_INTERFACE="${1:-can0}"

ip link set "$CAN_INTERFACE" down 2>/dev/null || true

if command -v pkill >/dev/null 2>&1; then
  pkill -TERM -f "slcand .* ${CAN_INTERFACE}$" 2>/dev/null || true
fi
