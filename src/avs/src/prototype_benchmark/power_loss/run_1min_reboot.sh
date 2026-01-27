#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! sudo -n true >/dev/null 2>&1; then
  echo "[ERR] sudo -n not available. Configure NOPASSWD for reboot or run with sudo."
  exit 1
fi

exec python3 "$SCRIPT_DIR/power_loss_experiment.py" \
  --start-new \
  --auto-reboot-after 60 \
  --recovery-timeout 600
