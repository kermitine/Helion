#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE="${1:-origin}"
BRANCH="${2:-}"

if ! git -C "$REPO_DIR" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "Not a git checkout: $REPO_DIR" >&2
  exit 2
fi

if [[ -z "$BRANCH" ]]; then
  BRANCH="$(git -C "$REPO_DIR" symbolic-ref --quiet --short HEAD || true)"
fi

if [[ -z "$BRANCH" ]]; then
  echo "Could not determine current branch. Pass one explicitly, e.g. helion-update origin main" >&2
  exit 2
fi

echo "[update] repo=$REPO_DIR remote=$REMOTE branch=$BRANCH"
git -C "$REPO_DIR" fetch "$REMOTE"
git -C "$REPO_DIR" pull --ff-only "$REMOTE" "$BRANCH"

python3 -m py_compile \
  "$REPO_DIR/raspi/robstride_socketcan.py" \
  "$REPO_DIR/raspi/robstride_dashboard.py"

if command -v systemctl >/dev/null 2>&1 && systemctl list-unit-files robstride-dashboard.service >/dev/null 2>&1; then
  if [[ "${HELION_NO_RESTART:-0}" == "1" ]]; then
    echo "[update] HELION_NO_RESTART=1; skipped dashboard restart."
  elif [[ "${EUID:-$(id -u)}" -eq 0 ]]; then
    systemctl restart robstride-dashboard.service
    systemctl --no-pager --lines=12 status robstride-dashboard.service || true
  elif sudo -n /usr/local/sbin/helion-restart-dashboard; then
    systemctl --no-pager --lines=12 status robstride-dashboard.service || true
  elif [[ -t 0 ]]; then
    sudo /usr/local/sbin/helion-restart-dashboard
    systemctl --no-pager --lines=12 status robstride-dashboard.service || true
  else
    echo "[update] dashboard restart was not permitted; run: sudo systemctl restart robstride-dashboard.service"
  fi
else
  echo "[update] robstride-dashboard.service is not installed; skipped restart."
fi
