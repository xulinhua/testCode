#!/usr/bin/env bash
# 启动 Nova 抓取 Qt UI（需已 source 工作空间与 MoveIt/MuJoCo 仿真）。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${WS_ROOT}/install/setup.bash"
else
  echo "ERROR: workspace not built. Run: colcon build --packages-select nova_grasp_moveit" >&2
  exit 1
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

exec ros2 run nova_grasp_moveit grasp_qt_ui_node "$@"
