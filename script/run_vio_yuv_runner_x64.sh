#!/usr/bin/env bash
# 在 x64 上运行 vio_yuv_runner
# 默认数据目录: <工作区>/data/yuv_data/init/outdoor/stone_brick/move
# （数据在 quadrotor_ws/data 时，open_vins 仅为子目录，不能用 open_vins/data）
# 用法: ./script/run_vio_yuv_runner_x64.sh [数据目录] [配置文件]
#       或只传选项如 --no-display

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)" # open_vins 根
# 上一级常为 quadrotor_ws（与 open_vins 并列放 data/）
WS_ROOT="$(cd "$ROOT/.." && pwd)"
cd "$ROOT"

DEFAULT_SCENE_REL="data/yuv_data/init/outdoor/stone_brick/move"
default_data_dir() {
  if [ -d "$WS_ROOT/$DEFAULT_SCENE_REL" ]; then
    echo "$WS_ROOT/$DEFAULT_SCENE_REL"
  elif [ -d "$ROOT/$DEFAULT_SCENE_REL" ]; then
    echo "$ROOT/$DEFAULT_SCENE_REL"
  else
    echo "$WS_ROOT/$DEFAULT_SCENE_REL"
  fi
}
DEFAULT_DATA="$(default_data_dir)"
DEFAULT_CONFIG="$ROOT/ov_yuv_parser/config/openvins/estimator_config.yaml"

# x64 Release 构建目录（vcpkg preset）
BUILD_DIR="${ROOT}/build/x86_64/Release/x64-release-vcpkg"
BIN="${BUILD_DIR}/ov_yuv_parser/vio_yuv_runner"
# 若已安装则优先用 install 下的
if [ -x "${ROOT}/install/x86_64/Release/bin/vio_yuv_runner" ]; then
  BIN="${ROOT}/install/x86_64/Release/bin/vio_yuv_runner"
  export LD_LIBRARY_PATH="${ROOT}/install/x86_64/Release/lib:${ROOT}/install/x86_64/Release/thirdparty/lib:${LD_LIBRARY_PATH}"
else
  [ ! -x "$BIN" ] && { echo "错误: 未找到 x64 可执行文件，请先构建: cmake --build --preset x64-release-vcpkg" >&2; exit 1; }
  export LD_LIBRARY_PATH="${BUILD_DIR}/ov_core:${BUILD_DIR}/ov_yuv_parser:${BUILD_DIR}/vcpkg_installed/x64-linux-custom/lib:${LD_LIBRARY_PATH}"
fi

# 无参数时使用绝对路径，避免依赖 cwd
if [ $# -eq 0 ]; then
  set -- "$DEFAULT_DATA" "$DEFAULT_CONFIG"
fi

exec "$BIN" "$@"
