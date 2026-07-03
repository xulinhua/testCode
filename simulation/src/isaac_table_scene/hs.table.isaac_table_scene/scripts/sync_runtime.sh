#!/usr/bin/env bash
# 将 python/ 源码同步到 Isaac Sim 实际加载的 hs/table/isaac_table_scene/ 包。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
rsync -a --delete "$ROOT/python/impl/" "$ROOT/hs/table/isaac_table_scene/impl/"
cp "$ROOT/python/"*.py "$ROOT/hs/table/isaac_table_scene/"
echo "Synced to $ROOT/hs/table/isaac_table_scene/"
echo ""
echo "Note: sync does NOT start Isaac Sim. Run:"
echo "  $ROOT/scripts/start_isaac.sh"
