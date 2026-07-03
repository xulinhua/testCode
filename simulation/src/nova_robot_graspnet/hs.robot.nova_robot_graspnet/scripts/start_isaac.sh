#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EXT_FOLDER="$(cd "$ROOT/.." && pwd)"

_run_prepare() {
  local script="$1"
  shift
  python3 "$script" "$@" || true
}

# 仅在源文件更新时跑预处理，避免每次启动多等几十秒
ROBOT_SRC="$ROOT/data/robot/nova_robot.usda"
ROBOT_PREP="$ROOT/data/robot/nova_robot_prepared.usda"
if [[ ! -f "$ROBOT_PREP" ]] || [[ "$ROBOT_SRC" -nt "$ROBOT_PREP" ]]; then
  _run_prepare "$ROOT/scripts/prepare_urdf.py"
  _run_prepare "$ROOT/scripts/prepare_robot_usd.py"
else
  echo "skip prepare_robot (prepared USD up to date)"
fi

BOX_META="$ROOT/data/box/grasp_box_meta.json"
BOX_OBJ="$(find "$ROOT/data/box/model" -maxdepth 1 -name '*.obj' 2>/dev/null | head -1 || true)"
if [[ ! -f "$BOX_META" ]] || { [[ -n "$BOX_OBJ" ]] && [[ "$BOX_OBJ" -nt "$BOX_META" ]]; }; then
  _run_prepare "$ROOT/scripts/prepare_box_usd.py"
else
  echo "skip prepare_box (meta up to date)"
fi

ISAAC_PYTHON="${ISAAC_PYTHON:-/home/hs/anaconda3/envs/isaac_env/bin/isaacsim}"

# 启动 Isaac 前烘焙盒子 mesh+贴图，避免首次 Load scene 解析 80MB OBJ
_ensure_box_baked() {
  local baked="$ROOT/data/box/grasp_box_baked.usdc"
  local obj png stamp
  obj="$(find "$ROOT/data/box/model" -maxdepth 1 -name '*.obj' 2>/dev/null | head -1 || true)"
  png="$(find "$ROOT/data/box/model" -maxdepth 1 -name '*.png' 2>/dev/null | head -1 || true)"
  if [[ -z "$obj" ]]; then
    return 0
  fi
  stamp="$obj"
  if [[ -n "$png" && "$png" -nt "$stamp" ]]; then
    stamp="$png"
  fi
  if [[ -f "$BOX_META" && "$BOX_META" -nt "$stamp" ]]; then
    stamp="$BOX_META"
  fi
  if [[ -f "$baked" && "$baked" -nt "$stamp" ]]; then
    echo "skip box bake (grasp_box_baked.usdc up to date)"
    return 0
  fi
  if [[ "${NOVA_SKIP_BOX_BAKE:-1}" == "1" ]]; then
    echo "skip box bake (Load scene builds mesh in Isaac; set NOVA_SKIP_BOX_BAKE=0 to pre-bake)"
    return 0
  fi
  echo "Pre-baking box mesh + texture -> grasp_box_baked.usdc (~30-60s, only when OBJ/PNG changes) ..."
  export PYTHONPATH="$ROOT/hs${PYTHONPATH:+:$PYTHONPATH}"
  ISAAC_PY="${ISAAC_PY:-/home/hs/anaconda3/envs/isaac_env/bin/python3.11}"
  if "$ISAAC_PY" "$ROOT/scripts/bake_box_mesh.py"; then
    echo "box bake done"
  else
    echo "Warning: box bake failed — Load scene may be slow and texture may be missing"
  fi
}
_ensure_box_baked
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

if pgrep -f 'isaacsim.*isaacsim\.exp\.full\.kit' >/dev/null 2>&1; then
  echo "Warning: 检测到已有 Isaac Sim 在运行，请先退出再启动，否则可能卡在 RTX 初始化。"
fi

ENABLE_ARGS=()
if [[ "${NOVA_GRASPNET_ENABLE:-1}" == "1" ]]; then
  ENABLE_ARGS=(--enable hs.robot.nova_robot_graspnet)
fi

echo "Starting Isaac Sim (ext-folder: $EXT_FOLDER)"
echo "  扩展: hs.robot.nova_robot_graspnet (NOVA_GRASPNET_ENABLE=${NOVA_GRASPNET_ENABLE:-1})"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID  (rqt/ros2 终端必须一致，否则看不到话题)"
echo "  面板: 顶栏菜单「Hs Robot Nova Robot GraspNet」，或 Window 子菜单"
echo "  注意: 必须用本脚本启动；直接点桌面 Isaac 图标不会加载本扩展"
exec "$ISAAC_PYTHON" "$ISAAC_KIT" \
  --ext-folder "$EXT_FOLDER" \
  "${ENABLE_ARGS[@]}" \
  "$@"
