#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE="${1:-origin}"
BRANCH="${2:-}"
GIT_LOCK_STALE_SECONDS="${GIT_LOCK_STALE_SECONDS:-60}"

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

clear_stale_git_lock() {
  local lock_path="$REPO_DIR/.git/index.lock"
  if [[ ! -e "$lock_path" ]]; then
    return
  fi

  if command -v fuser >/dev/null 2>&1 && fuser "$lock_path" >/dev/null 2>&1; then
    echo "[update] Git lock is currently held: $lock_path" >&2
    echo "[update] Wait for the other Git process to finish, then run helion-update again." >&2
    exit 3
  fi

  local now modified age
  now="$(date +%s)"
  modified="$(stat -c %Y "$lock_path" 2>/dev/null || echo "$now")"
  age=$((now - modified))

  if (( age < GIT_LOCK_STALE_SECONDS )); then
    echo "[update] Git lock exists but is only ${age}s old: $lock_path" >&2
    echo "[update] Refusing to remove a recent lock. Try again in a minute." >&2
    exit 3
  fi

  echo "[update] Removing stale Git lock (${age}s old): $lock_path"
  rm -f "$lock_path"
}

echo "[update] repo=$REPO_DIR remote=$REMOTE branch=$BRANCH"
clear_stale_git_lock
git -C "$REPO_DIR" fetch "$REMOTE"
clear_stale_git_lock
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
