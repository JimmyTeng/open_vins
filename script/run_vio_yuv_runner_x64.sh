#!/usr/bin/env bash
# 在 x64 上运行 vio_yuv_runner
# 默认数据目录: data/yuv_data/init/outdoor/stone_brick/move
# 用法: ./script/run_vio_yuv_runner_x64.sh [数据目录] [配置文件]
#       或只传选项如 --no-display

set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

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

# 无参数时使用默认数据目录和配置
if [ $# -eq 0 ]; then
  set -- "data/yuv_data/init/outdoor/stone_brick/move" "ov_yuv_parser/config/openvins/estimator_config.yaml"
fi

exec "$BIN" "$@"
