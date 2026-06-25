#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

source /opt/ros/humble/setup.bash

if [ ! -f "$WS_DIR/install/setup.bash" ]; then
  echo "Workspace is not built yet: $WS_DIR/install/setup.bash not found"
  echo "Run: cd $WS_DIR && colcon build --symlink-install --packages-select rc2026_field mmrobot_description"
  exit 1
fi

source "$WS_DIR/install/setup.bash"

exec ros2 launch rc2026_field rc2026_mmrobot_rviz.launch.py "$@"
