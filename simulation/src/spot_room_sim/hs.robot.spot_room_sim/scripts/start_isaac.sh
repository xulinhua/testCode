#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EXT_FOLDER="$(cd "$ROOT/.." && pwd)"

ISAAC_PYTHON="${ISAAC_PYTHON:-/home/hs/anaconda3/envs/isaac_env/bin/isaacsim}"
ISAAC_KIT="${ISAAC_KIT:-/home/hs/anaconda3/envs/isaac_env/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.full.kit}"
ISAAC_SITE="${ISAAC_SITE:-/home/hs/anaconda3/envs/isaac_env/lib/python3.11/site-packages/isaacsim}"
ROS2_BRIDGE_EXT="${ISAAC_SITE}/exts/isaacsim.ros2.bridge"

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-31}"
if [[ -d "${ROS2_BRIDGE_EXT}/humble/lib" ]]; then
  case ":${LD_LIBRARY_PATH:-}:" in
    *":${ROS2_BRIDGE_EXT}/humble/lib:"*) ;;
    *) export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:${ROS2_BRIDGE_EXT}/humble/lib" ;;
  esac
fi

# 去掉系统 ROS(Python3.10) 路径，避免 Isaac(Python3.11) 误 import 系统 rclpy
if [[ -n "${PYTHONPATH:-}" ]]; then
  _NEW_PP=""
  IFS=':' read -r -a _PP_ARR <<< "$PYTHONPATH"
  for _p in "${_PP_ARR[@]}"; do
    case "$_p" in
      */opt/ros/*|*/ros/humble/*) continue ;;
    esac
    if [[ -n "$_p" ]]; then
      _NEW_PP="${_NEW_PP:+$_NEW_PP:}$_p"
    fi
  done
  export PYTHONPATH="$_NEW_PP"
  unset _NEW_PP _PP_ARR _p
fi

# 本地资产：Simple Room + Spot + policy
if [[ "${SPOT_SKIP_ASSET_CHECK:-0}" != "1" ]]; then
  if ! python3 "$ROOT/scripts/download_assets.py" --verify-only >/dev/null 2>&1; then
    echo "Local assets missing — downloading once (~130MB, needs network) ..."
    python3 "$ROOT/scripts/download_assets.py" || {
      echo "ERROR: asset download failed. Fix network and re-run, or set SPOT_SKIP_ASSET_CHECK=1"
      exit 1
    }
  else
    echo "Local assets OK (data/scenes|robots|policies)"
  fi
fi

# 压缩 Simple Room 贴图（~100MB → 几 MB），显著加快 Load；已 lite 则跳过
if [[ "${SPOT_SKIP_LITE_TEXTURES:-0}" != "1" ]]; then
  python3 "$ROOT/scripts/make_lite_textures.py" || {
    echo "WARNING: make_lite_textures failed (need Pillow or ImageMagick). Load may stay slow."
  }
fi

# 默认关掉残留 Isaac，避免多实例卡 RTX / “节点还开着”
if pgrep -f 'isaacsim.*isaacsim\.exp\.full\.kit' >/dev/null 2>&1; then
  if [[ "${SPOT_KILL_EXISTING:-1}" == "1" ]]; then
    echo "Stopping existing Isaac Sim process(es) ..."
    pkill -f 'isaacsim.*isaacsim\.exp\.full\.kit' || true
    sleep 2
  else
    echo "Warning: 已有 Isaac Sim 在运行。设 SPOT_KILL_EXISTING=1 可自动结束旧进程。"
  fi
fi

ENABLE_ARGS=()
if [[ "${SPOT_ROOM_SIM_ENABLE:-1}" == "1" ]]; then
  ENABLE_ARGS=(--enable hs.robot.spot_room_sim)
fi

echo "Starting Isaac Sim (ext-folder: $EXT_FOLDER)"
echo "  extension: hs.robot.spot_room_sim"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  panel: Hs Robot Spot Room Sim -> Load -> Play"
# Perf + quieter logs:
# - Spot physics is 500Hz; cap catch-up so low FPS cannot schedule hundreds of steps/frame
# - ecoMode / disable heavy RTX effects for mapping
exec "$ISAAC_PYTHON" "$ISAAC_KIT" \
  --ext-folder "$EXT_FOLDER" \
  "${ENABLE_ARGS[@]}" \
  --/app/player/maxTimeStepsPerFrame=8 \
  --/app/player/useFixedTimeStepping=false \
  --/app/runLoops/main/rateLimitEnabled=true \
  --/app/runLoops/main/rateLimitFrequency=30 \
  --/rtx/ecoMode/enabled=true \
  --/rtx/reflections/enabled=false \
  --/rtx/indirectDiffuse/enabled=false \
  --/rtx/ambientOcclusion/enabled=false \
  --/log/channels/omni.isaac.core=error \
  --/log/channels/omni.isaac.manipulators=error \
  --/log/channels/omni.isaac.menu=error \
  --/log/channels/omni.isaac.motion_generation=error \
  --/log/channels/omni.hydra=error \
  "$@"
