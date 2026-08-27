#!/usr/bin/env bash
set -euo pipefail

CAN_INTERFACE="${1:-can0}"
HELION_CAN_BACKEND="${HELION_CAN_BACKEND:-${CAN_BACKEND:-auto}}"
BITRATE="${BITRATE:-1000000}"
RESTART_MS="${RESTART_MS:-100}"
TX_QUEUE_LEN="${TX_QUEUE_LEN:-2000}"
SLCAN_PORT="${SLCAN_PORT:-}"

slcan_speed_code() {
  case "$1" in
    10000) echo 0 ;;
    20000) echo 1 ;;
    50000) echo 2 ;;
    100000) echo 3 ;;
    125000) echo 4 ;;
    250000) echo 5 ;;
    500000) echo 6 ;;
    800000) echo 7 ;;
    1000000) echo 8 ;;
    *)
      echo "Unsupported SLCAN bitrate: $1" >&2
      return 1
      ;;
  esac
}

detect_slcan_port() {
  local dev
  for dev in /dev/ttyUSB* /dev/ttyACM*; do
    if [[ -e "$dev" ]]; then
      printf "%s\n" "$dev"
      return 0
    fi
  done
  return 1
}

modprobe_can() {
  modprobe can 2>/dev/null || true
  modprobe can_raw 2>/dev/null || true
  modprobe slcan 2>/dev/null || true
}

bring_up_native_can() {
  ip link set "$CAN_INTERFACE" down 2>/dev/null || true
  ip link set "$CAN_INTERFACE" txqueuelen "$TX_QUEUE_LEN"
  ip link set "$CAN_INTERFACE" type can bitrate "$BITRATE" restart-ms "$RESTART_MS"
  ip link set "$CAN_INTERFACE" up
}

bring_up_slcan() {
  local port="${SLCAN_PORT:-}"
  local speed

  speed="$(slcan_speed_code "$BITRATE")"
  if [[ -z "$port" ]]; then
    port="$(detect_slcan_port || true)"
  fi
  if [[ -z "$port" ]]; then
    echo "No SLCAN serial adapter found. Set SLCAN_PORT=/dev/ttyUSB0." >&2
    return 1
  fi

  modprobe_can
  if ip link show "$CAN_INTERFACE" >/dev/null 2>&1; then
    echo "$CAN_INTERFACE already exists; leaving existing slcand/socketcan device in place."
  else
    echo "Starting slcand on $port as $CAN_INTERFACE at ${BITRATE} bit/s."
    slcand -o -c "-s${speed}" "$port" "$CAN_INTERFACE"
    sleep 0.3
  fi

  ip link set "$CAN_INTERFACE" txqueuelen "$TX_QUEUE_LEN" 2>/dev/null || true
  ip link set "$CAN_INTERFACE" up
}

modprobe_can

case "$HELION_CAN_BACKEND" in
  native|socketcan)
    bring_up_native_can
    ;;
  slcan)
    bring_up_slcan
    ;;
  auto)
    if bring_up_native_can 2>/tmp/helion-can-native.err; then
      :
    else
      echo "Native SocketCAN setup failed; trying SLCAN serial adapter." >&2
      cat /tmp/helion-can-native.err >&2 || true
      bring_up_slcan
    fi
    ;;
  *)
    echo "Unknown HELION_CAN_BACKEND=$HELION_CAN_BACKEND; use auto, native, or slcan." >&2
    exit 1
    ;;
esac

ip -details -statistics link show "$CAN_INTERFACE"
