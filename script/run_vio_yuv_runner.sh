#!/bin/sh
# 设置依赖库路径并运行 vio_yuv_runner（与 bin 同级的 lib 目录）
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
WS_ROOT=$(cd "$ROOT/.." && pwd)
export LD_LIBRARY_PATH="${ROOT}/lib:${ROOT}/thirdparty/lib:${LD_LIBRARY_PATH}"

# 查找可执行文件：script 同目录 > install > build
BIN=""
if [ -x "${SCRIPT_DIR}/vio_yuv_runner" ]; then
  BIN="${SCRIPT_DIR}/vio_yuv_runner"
elif [ -x "${ROOT}/install/x86_64/Release/bin/vio_yuv_runner" ]; then
  BIN="${ROOT}/install/x86_64/Release/bin/vio_yuv_runner"
  export LD_LIBRARY_PATH="${ROOT}/install/x86_64/Release/lib:${ROOT}/install/x86_64/Release/thirdparty/lib:${LD_LIBRARY_PATH}"
elif [ -x "${ROOT}/build/x86_64/Release/x64-release/ov_yuv_parser/vio_yuv_runner" ]; then
  BIN="${ROOT}/build/x86_64/Release/x64-release/ov_yuv_parser/vio_yuv_runner"
  BUILD_DIR="${ROOT}/build/x86_64/Release/x64-release"
  export LD_LIBRARY_PATH="${BUILD_DIR}/ov_core:${BUILD_DIR}/ov_yuv_parser:${LD_LIBRARY_PATH}"
fi

if [ -z "$BIN" ]; then
  echo "错误: 未找到 vio_yuv_runner，请先构建或安装（如 cmake --build --preset x64-release）" >&2
  exit 1
fi

DEFAULT_SCENE_REL="data/yuv_data/init/outdoor/stone_brick/move"
if [ -d "$WS_ROOT/$DEFAULT_SCENE_REL" ]; then
  DEFAULT_DATA="$WS_ROOT/$DEFAULT_SCENE_REL"
elif [ -d "$ROOT/$DEFAULT_SCENE_REL" ]; then
  DEFAULT_DATA="$ROOT/$DEFAULT_SCENE_REL"
else
  DEFAULT_DATA="$WS_ROOT/$DEFAULT_SCENE_REL"
fi
DEFAULT_CONFIG="$ROOT/ov_yuv_parser/config/openvins/estimator_config.yaml"
[ $# -eq 0 ] && set -- "$DEFAULT_DATA" "$DEFAULT_CONFIG"
exec "$BIN" "$@"
