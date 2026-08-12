#!/usr/bin/env bash
# 启动 Isaac Sim 并启用独立扩展 hs.calib.isaac_calib_sim（不依赖其他工程）。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EXT_FOLDER="$(cd "$ROOT/.." && pwd)"

ISAAC_PYTHON="${ISAAC_PYTHON:-/home/hs/anaconda3/envs/isaac_env/bin/isaacsim}"
ISAAC_KIT="${ISAAC_KIT:-/home/hs/anaconda3/envs/isaac_env/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.full.kit}"
ISAAC_SITE="${ISAAC_SITE:-/home/hs/anaconda3/envs/isaac_env/lib/python3.11/site-packages/isaacsim}"
ROS2_BRIDGE_EXT="${ISAAC_SITE}/exts/isaacsim.ros2.bridge"

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ -d "${ROS2_BRIDGE_EXT}/humble/lib" ]]; then
  case ":${LD_LIBRARY_PATH:-}:" in
    *":${ROS2_BRIDGE_EXT}/humble/lib:"*) ;;
    *) export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${ROS2_BRIDGE_EXT}/humble/lib" ;;
  esac
fi

# 避免系统 ROS Python 污染 Isaac 解释器
if [[ -n "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v '/opt/ros/' | paste -sd: - || true)"
fi

echo "Starting Isaac Sim with hs.calib.isaac_calib_sim ..."
echo "  ext-folder: $EXT_FOLDER"
echo "  ROS_DISTRO=$ROS_DISTRO RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
exec "$ISAAC_PYTHON" "$ISAAC_KIT" \
  --ext-folder "$EXT_FOLDER" \
  --enable hs.calib.isaac_calib_sim \
  "$@"
