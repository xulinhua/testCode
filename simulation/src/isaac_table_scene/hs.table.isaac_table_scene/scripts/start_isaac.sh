#!/usr/bin/env bash
# 同步 python/ → hs/ 包，并启动 Isaac Sim + 本扩展。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EXT_FOLDER="$(cd "$ROOT/.." && pwd)"

"$ROOT/scripts/sync_runtime.sh"

ISAAC_PYTHON="${ISAAC_PYTHON:-/home/hs/anaconda3/envs/isaac_env/bin/isaacsim}"
ISAAC_KIT="${ISAAC_KIT:-/home/hs/anaconda3/envs/isaac_env/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.full.kit}"
ISAAC_SITE="${ISAAC_SITE:-/home/hs/anaconda3/envs/isaac_env/lib/python3.11/site-packages/isaacsim}"
ROS2_BRIDGE_EXT="${ISAAC_SITE}/exts/isaacsim.ros2.bridge"

# pip 版 Isaac Sim 需使用内置 humble 库；未设置时自动配置
export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ -d "${ROS2_BRIDGE_EXT}/humble/lib" ]]; then
  case ":${LD_LIBRARY_PATH:-}:" in
    *":${ROS2_BRIDGE_EXT}/humble/lib:"*) ;;
    *) export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${ROS2_BRIDGE_EXT}/humble/lib" ;;
  esac
fi

echo "Starting Isaac Sim with hs.table.isaac_table_scene ..."
echo "  ext-folder: $EXT_FOLDER"
echo "  ROS_DISTRO=$ROS_DISTRO RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
exec "$ISAAC_PYTHON" "$ISAAC_KIT" \
  --ext-folder "$EXT_FOLDER" \
  --enable hs.table.isaac_table_scene \
  "$@"
