#!/bin/sh
# 设置依赖库路径并运行 vio_yuv_runner（与 bin 同级的 lib 目录）
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
export LD_LIBRARY_PATH="${SCRIPT_DIR}/../lib:${SCRIPT_DIR}/../thirdparty/lib:${LD_LIBRARY_PATH}"
[ $# -eq 0 ] && set -- "../data/move/"
exec "${SCRIPT_DIR}/vio_yuv_runner" "$@"
