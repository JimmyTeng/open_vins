#!/usr/bin/env bash
# 在 x64 上运行 vio_yuv_runner
# 数据根: <工作区>/data/yuv_data/init（或与 open_vins 并列的 ws 根下同名路径）
# 用法:
#   ./script/run_vio_yuv_runner_x64.sh
#       无参数且存在 init 目录 → 列出 init 下所有「含 dump_yuv」的多层级子目录，按序号选择
#   ./script/run_vio_yuv_runner_x64.sh -i|--pick
#       同上，显式进入交互选择
#   ./script/run_vio_yuv_runner_x64.sh 3
#       非交互：选第 3 条（与列表顺序一致）
#   ./script/run_vio_yuv_runner_x64.sh [数据目录] [配置文件] [--no-display ...]
#       与原先一致；第一个参数为已存在目录时按路径运行

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
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

# x64 Release 构建目录（CMake preset: x64-release）
BUILD_DIR="${ROOT}/build/x86_64/Release/x64-release"
BIN="${BUILD_DIR}/ov_yuv_parser/vio_yuv_runner"
if [ -x "${ROOT}/install/x86_64/Release/bin/vio_yuv_runner" ]; then
  BIN="${ROOT}/install/x86_64/Release/bin/vio_yuv_runner"
  export LD_LIBRARY_PATH="${ROOT}/install/x86_64/Release/lib:${ROOT}/install/x86_64/Release/thirdparty/lib:${LD_LIBRARY_PATH}"
else
  [ ! -x "$BIN" ] && {
    echo "错误: 未找到 x64 可执行文件，请先构建: cmake --build --preset x64-release" >&2
    exit 1
  }
  export LD_LIBRARY_PATH="${BUILD_DIR}/ov_core:${BUILD_DIR}/ov_yuv_parser:${LD_LIBRARY_PATH}"
fi

# 打印将要执行的完整命令（可直接复制复现）
run_and_exec() {
  echo "实际运行命令:"
  printf '  '
  printf '%q ' "$@"
  echo
  exec "$@"
}

# 解析 init 根目录（优先工作区上一级，与脚本注释一致）
resolve_init_base() {
  INIT_BASE=""
  for candidate in "$WS_ROOT/data/yuv_data/init" "$ROOT/data/yuv_data/init"; do
    if [ -d "$candidate" ]; then
      INIT_BASE="$candidate"
      return 0
    fi
  done
  return 1
}

# 列出 init 下任意深度、且直接包含子目录 dump_yuv 的数据集根路径（一层一层多层级）
discover_dataset_roots() {
  local base="$1"
  [ -d "$base" ] || return 1
  find "$base" -mindepth 1 -type d 2>/dev/null | while IFS= read -r d; do
    [ -d "$d/dump_yuv" ] && printf '%s\n' "$d"
  done | LC_ALL=C sort -u
}

# 全局数组 DATASET_LIST（下标 0..n-1）
DATASET_LIST=()
refresh_dataset_list() {
  DATASET_LIST=()
  local line
  while IFS= read -r line; do
    [ -n "$line" ] && DATASET_LIST+=("$line")
  done < <(discover_dataset_roots "$INIT_BASE")
}

pick_dataset_interactive() {
  if ! resolve_init_base; then
    echo "未找到目录: data/yuv_data/init（已尝试工作区上一级与 open_vins 根目录）" >&2
    echo "使用默认数据目录: $DEFAULT_DATA" >&2
    printf '%s\n' "$DEFAULT_DATA"
    return 0
  fi
  refresh_dataset_list
  local n=${#DATASET_LIST[@]}
  if [ "$n" -eq 0 ]; then
    echo "在 $INIT_BASE 下未发现含 dump_yuv/ 的子目录" >&2
    echo "使用默认: $DEFAULT_DATA" >&2
    printf '%s\n' "$DEFAULT_DATA"
    return 0
  fi
  echo "" >&2
  echo "数据目录（相对于 .../init/）：" >&2
  local i rel
  for i in "${!DATASET_LIST[@]}"; do
    rel="${DATASET_LIST[$i]#$INIT_BASE/}"
    rel="${rel#/}"
    printf '  %2d) %s\n' "$((i + 1))" "$rel" >&2
  done
  echo "" >&2
  local sel=""
  while true; do
    read -r -p "请输入序号 [1-${n}]（回车默认 1）: " sel
    sel="${sel:-1}"
    if [[ "$sel" =~ ^[0-9]+$ ]] && [ "$sel" -ge 1 ] && [ "$sel" -le "$n" ]; then
      printf '%s\n' "${DATASET_LIST[$((sel - 1))]}"
      return 0
    fi
    echo "无效序号，请输入 1 到 $n 之间的数字" >&2
  done
}

pick_dataset_by_index() {
  local idx="$1"
  if ! resolve_init_base; then
    echo "错误: 未找到 data/yuv_data/init，无法按序号选择" >&2
    exit 1
  fi
  refresh_dataset_list
  local n=${#DATASET_LIST[@]}
  if [ "$n" -eq 0 ]; then
    echo "错误: $INIT_BASE 下无可用数据集（需含 dump_yuv/）" >&2
    exit 1
  fi
  if ! [[ "$idx" =~ ^[0-9]+$ ]] || [ "$idx" -lt 1 ] || [ "$idx" -gt "$n" ]; then
    echo "错误: 序号必须在 1..$n 之间，当前为 $idx" >&2
    exit 1
  fi
  printf '%s\n' "${DATASET_LIST[$((idx - 1))]}"
}

# ---------- 参数解析（去掉仅作用于脚本的 -i / --pick）----------
FILTERED=()
for a in "$@"; do
  case "$a" in
  -i | --pick | --interactive) ;;
  *) FILTERED+=("$a") ;;
  esac
done
set -- "${FILTERED[@]}"

if [ $# -eq 0 ]; then
  D="$(pick_dataset_interactive)"
  run_and_exec "$BIN" "$D" "$DEFAULT_CONFIG"
fi

# 首参数是选项（如仅 --no-display）→ 先选目录再带上选项
if [[ "$1" == -* ]] && [ ! -d "$1" ]; then
  D="$(pick_dataset_interactive)"
  run_and_exec "$BIN" "$D" "$DEFAULT_CONFIG" "$@"
fi

# 纯数字且当前目录下不存在同名目录 → 按序号选（若存在名为数字的文件夹则仍按路径处理）
if [[ "$1" =~ ^[0-9]+$ ]]; then
  if [ -d "$1" ]; then
    run_and_exec "$BIN" "$@"
  fi
  D="$(pick_dataset_by_index "$1")"
  echo "已选数据集: $D" >&2
  shift
  if [ $# -eq 0 ]; then
    run_and_exec "$BIN" "$D" "$DEFAULT_CONFIG"
  fi
  # 第一个剩余参数是 yaml 则认为是指定配置；否则插入默认配置（供 --no-display 等选项）
  if [[ "$1" == *.yaml ]] || [[ "$1" == *.yml ]]; then
    run_and_exec "$BIN" "$D" "$@"
  fi
  run_and_exec "$BIN" "$D" "$DEFAULT_CONFIG" "$@"
fi

# 显式数据目录：须存在
case "$1" in
--no-display | --no-imshow | -n | --cache-size | -h | --help) ;;
*)
  if [[ "$1" != -* ]] && [ ! -d "$1" ]; then
    echo "错误: 数据目录不存在: $1" >&2
    echo "  不传参数: 在 data/yuv_data/init 下按序号交互选择" >&2
    echo "  或: $0 N   选第 N 条；或: $0 /path/to/scene [config.yaml] [选项]" >&2
    exit 1
  fi
  ;;
esac

run_and_exec "$BIN" "$@"
