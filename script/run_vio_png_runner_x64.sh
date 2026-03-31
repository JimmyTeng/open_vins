#!/usr/bin/env bash
# x64 运行 vio_png_runner：回放 VioDataRecorder 数据集（dataset_meta + imu.csv + camera_index + mosaics）
# 默认数据: /home/jimmy/data/20260321145202
# 默认配置: ov_yuv_parser/config/openvins ver2/estimator_config.yaml
#
# 用法:
#   ./script/run_vio_png_runner_x64.sh
#   ./script/run_vio_png_runner_x64.sh /path/to/dataset
#   ./script/run_vio_png_runner_x64.sh /path/to/dataset --no-display
# 环境变量覆盖:
#   VIO_DATASET_DIR  数据集根目录
#   VIO_ESTIMATOR_YAML  estimator_config.yaml 绝对路径（可选，默认 ver2）

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# x64 Release 构建目录（与 run_vio_yuv_runner_x64.sh 一致）
BUILD_DIR="${ROOT}/build/x86_64/Release/x64-release"
BIN="${BUILD_DIR}/ov_yuv_parser/vio_png_runner"
if [ -x "${ROOT}/install/x86_64/Release/bin/vio_png_runner" ]; then
  BIN="${ROOT}/install/x86_64/Release/bin/vio_png_runner"
  export LD_LIBRARY_PATH="${ROOT}/install/x86_64/Release/lib:${ROOT}/install/x86_64/Release/thirdparty/lib:${LD_LIBRARY_PATH:-}"
else
  [ ! -x "$BIN" ] && {
    echo "错误: 未找到 vio_png_runner，请先构建: cmake --build --preset x64-release" >&2
    exit 1
  }
  export LD_LIBRARY_PATH="${BUILD_DIR}/ov_core:${BUILD_DIR}/ov_yuv_parser:${LD_LIBRARY_PATH:-}"
fi

: "${VIO_DATASET_DIR:=/home/jimmy/data/20260321145202}"
CONFIG_DIR="${ROOT}/ov_yuv_parser/config/openvins ver2"
DEFAULT_YAML="${CONFIG_DIR}/estimator_config.yaml"
CONFIG_ABS="${VIO_ESTIMATOR_YAML:-$DEFAULT_YAML}"

if [ $# -ge 1 ] && [ "$1" != -* ]; then
  VIO_DATASET_DIR="$1"
  shift
fi

if [ ! -d "$VIO_DATASET_DIR" ]; then
  echo "错误: 数据集目录不存在: $VIO_DATASET_DIR" >&2
  exit 1
fi
if [ ! -f "$CONFIG_ABS" ]; then
  echo "错误: 配置文件不存在: $CONFIG_ABS" >&2
  exit 1
fi

DATA_ABS="$(cd "$VIO_DATASET_DIR" && pwd)"
CONFIG_DIR_ABS="$(cd "$(dirname "$CONFIG_ABS")" && pwd)"
CONFIG_NAME="$(basename "$CONFIG_ABS")"

echo "vio_png_runner (x64)"
echo "  数据集: $DATA_ABS"
echo "  配置目录: $CONFIG_DIR_ABS"
echo "  YAML:     $CONFIG_NAME"
echo

# YAML 内 relative_config_imu 等为相对路径：在配置目录下启动，保证 kalibr 等能打开
cd "$CONFIG_DIR_ABS"
exec "$BIN" "$@" "$DATA_ABS" "$(pwd)/$CONFIG_NAME"
