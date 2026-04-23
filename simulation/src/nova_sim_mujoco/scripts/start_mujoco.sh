#!/bin/bash
set -euo pipefail

URDF_FILE="${1:-}"

if [[ -z "${URDF_FILE}" ]]; then
  echo "[nova_sim_mujoco] 用法: start_mujoco.sh <urdf_file>"
  exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "[nova_sim_mujoco] 未找到 python3，无法启动 MuJoCo。"
  exit 1
fi

python3 - "$URDF_FILE" <<'PY'
import pathlib
import sys
import tempfile

urdf_path = pathlib.Path(sys.argv[1]).resolve()
if not urdf_path.exists():
    raise SystemExit(f"[nova_sim_mujoco] URDF 不存在: {urdf_path}")

try:
    import mujoco  # type: ignore
    import mujoco.viewer  # type: ignore
except Exception as exc:
    raise SystemExit(f"[nova_sim_mujoco] Python MuJoCo 不可用: {exc}")

try:
    from urdf2mjcf import convert_urdf_to_mjcf  # type: ignore
except Exception as exc:
    raise SystemExit(
        "[nova_sim_mujoco] 缺少 URDF->MJCF 转换依赖 urdf2mjcf。\n"
        "请安装后重试: pip install urdf2mjcf\n"
        f"详细错误: {exc}"
    )

with tempfile.TemporaryDirectory(prefix="nova_mujoco_") as tmp_dir:
    mjcf_path = pathlib.Path(tmp_dir) / "model.xml"
    convert_urdf_to_mjcf(str(urdf_path), str(mjcf_path))

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)
    print(f"[nova_sim_mujoco] MuJoCo 启动成功，模型: {mjcf_path}")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()
PY
