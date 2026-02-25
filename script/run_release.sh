#!/bin/sh
# Release 目录运行脚本：引用 run_release.conf 设置 LD 库路径，
# 默认数据目录 ./data/move，默认使用 config 中的配置文件。
# 用法：复制本脚本与 run_release.conf 到 Release 目录，可改名为 run.sh / run.conf。

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
CONFIG="${SCRIPT_DIR}/run_release.conf"
[ -f "${SCRIPT_DIR}/run.conf" ] && CONFIG="${SCRIPT_DIR}/run.conf"

# 引用配置文件
if [ -f "$CONFIG" ]; then
  . "$CONFIG"
else
  echo "警告: 未找到配置文件 $CONFIG，使用默认值" >&2
  LIB_DIR="./lib"
  RUNNER="./bin/vio_yuv_runner"
  CONFIG_FILE="./config/openvins/estimator_config.yaml"
fi

# 相对路径转为基于脚本目录的绝对路径
resolve() {
  case "$1" in
    /*) echo "$1" ;;
    *) echo "${SCRIPT_DIR}/$1" ;;
  esac
}

LIB_DIR=$(resolve "$LIB_DIR")
export LD_LIBRARY_PATH="${LIB_DIR}:${LD_LIBRARY_PATH}"

RUNNER=$(resolve "$RUNNER")
CONFIG_PATH=""
[ -n "$CONFIG_FILE" ] && [ -f "$(resolve "$CONFIG_FILE")" ] && CONFIG_PATH=$(resolve "$CONFIG_FILE")

# 默认数据目录 ./data/move；若提供了 CONFIG_PATH 则作为第二参数传入
cd "$SCRIPT_DIR"
if [ $# -eq 0 ]; then
  set -- "./data/move"
  [ -n "$CONFIG_PATH" ] && set -- "./data/move" "$CONFIG_FILE"
fi

exec "$RUNNER" "$@"
