#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

exec python3 "$SCRIPT_DIR/power_loss_experiment.py" \
  --start-new \
  --auto-reboot-after 60 \
  --recovery-timeout 600
