#!/usr/bin/env bash
set -euo pipefail

CAN_INTERFACE="${1:-can0}"
BITRATE="${BITRATE:-1000000}"
RESTART_MS="${RESTART_MS:-100}"
TX_QUEUE_LEN="${TX_QUEUE_LEN:-2000}"

ip link set "$CAN_INTERFACE" down 2>/dev/null || true
ip link set "$CAN_INTERFACE" txqueuelen "$TX_QUEUE_LEN"
ip link set "$CAN_INTERFACE" type can bitrate "$BITRATE" restart-ms "$RESTART_MS"
ip link set "$CAN_INTERFACE" up
ip -details -statistics link show "$CAN_INTERFACE"
