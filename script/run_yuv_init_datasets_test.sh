#!/usr/bin/env bash
# 批量测试 data/yuv_data/init 下所有 dump_yuv 数据集（需已构建 x64-release）
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BIN="${ROOT}/build/x86_64/Release/x64-release/ov_yuv_parser/yuv_init_datasets_test"
DEFAULT_INIT="${ROOT}/data/yuv_data/init"

if [[ ! -x "$BIN" ]]; then
  echo "未找到可执行文件: $BIN" >&2
  echo "请先执行: ${ROOT}/script/build_preset.sh x64-release" >&2
  exit 1
fi

if [[ $# -eq 0 ]]; then
  exec "$BIN" "$DEFAULT_INIT"
fi

# 若仅传 --skip-yuv，仍使用默认 init 路径
has_path=false
for a in "$@"; do
  if [[ "$a" != -* ]]; then
    has_path=true
    break
  fi
done
if ! $has_path; then
  exec "$BIN" "$@" "$DEFAULT_INIT"
fi

exec "$BIN" "$@"
