#!/usr/bin/env bash
# 将 python/ 源码同步到 Isaac Sim 实际加载的 hs/box/isaac_box_grasp/ 包。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
rsync -a --delete "$ROOT/python/impl/" "$ROOT/hs/box/isaac_box_grasp/impl/"
cp "$ROOT/python/"*.py "$ROOT/hs/box/isaac_box_grasp/"
echo "Synced to $ROOT/hs/box/isaac_box_grasp/"
