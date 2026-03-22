#!/usr/bin/env bash
# 从脚本文件所在目录解析路径，再拼出数据/配置的绝对路径，避免依赖 cwd。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 本包根目录：.../open_vins/ov_yuv_parser
OVS_PKG="$(cd "$SCRIPT_DIR/.." && pwd)"
# 工作区根：优先 git 根目录；否则假定脚本在 open_vins/ov_yuv_parser/scripts/ 下，向上三级
REPO_ROOT="$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null || true)"
if [[ -z "$REPO_ROOT" ]]; then
  REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
fi

# 数据场景目录（不含 dump_yuv；程序会自动追加 dump_yuv，IMU 为 ../imu.txt）
: "${VIO_YUV_SCENE:=data/yuv_data/init/outdoor/grass/static}"
INPUT_DIR="$REPO_ROOT/$VIO_YUV_SCENE"

CONFIG_FILE="${VIO_YUV_CONFIG:-$OVS_PKG/config/openvins/estimator_config.yaml}"

resolve_bin() {
  if [[ -n "${VIO_YUV_RUNNER:-}" ]]; then
    echo "$VIO_YUV_RUNNER"
    return
  fi
  local cand
  for cand in \
    "$OVS_PKG/build/vio_yuv_runner" \
    "$REPO_ROOT/build/open_vins/ov_yuv_parser/vio_yuv_runner" \
    "$REPO_ROOT/build/vio_yuv_runner"; do
    if [[ -x "$cand" ]]; then
      echo "$cand"
      return
    fi
  done
  echo ""
}

BIN="$(resolve_bin)"
if [[ -z "$BIN" ]]; then
  echo "未找到可执行文件 vio_yuv_runner。请先编译，或设置环境变量 VIO_YUV_RUNNER 为二进制绝对路径。" >&2
  exit 1
fi

if [[ ! -d "$INPUT_DIR" ]]; then
  echo "数据目录不存在: $INPUT_DIR（可设置 VIO_YUV_SCENE 为相对 REPO_ROOT 的子路径）" >&2
  exit 1
fi
if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "配置文件不存在: $CONFIG_FILE（可设置 VIO_YUV_CONFIG）" >&2
  exit 1
fi

echo "SCRIPT_DIR=$SCRIPT_DIR"
echo "REPO_ROOT=$REPO_ROOT"
echo "INPUT_DIR=$INPUT_DIR"
echo "CONFIG_FILE=$CONFIG_FILE"
echo "BIN=$BIN"
exec "$BIN" "$@" "$INPUT_DIR" "$CONFIG_FILE"
