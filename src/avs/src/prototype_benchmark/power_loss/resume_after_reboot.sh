#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROS_DISTRO="${ROS_DISTRO:-}"
if [ -z "$ROS_DISTRO" ] && [ -d /opt/ros ]; then
  ROS_DISTRO="$(ls -1 /opt/ros 2>/dev/null | sort | tail -n 1 || true)"
fi

if [ -n "$ROS_DISTRO" ] && [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
  # shellcheck disable=SC1090
  set +u
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  set -u
fi

if [ -f "/home/avs/AVS-PI/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  set +u
  source "/home/avs/AVS-PI/install/setup.bash"
  set -u
fi

exec python3 "$SCRIPT_DIR/power_loss_experiment.py" \
  --resume
