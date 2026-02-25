#!/usr/bin/env bash
# 将 CMake 预编译包下载到 script/，供 Docker 构建使用（版本需与 Dockerfile 一致，满足 vcpkg 要求）
# 用法：./script/download_cmake_for_docker.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
V="3.31.10"
FILE="cmake-${V}-linux-x86_64.tar.gz"
DEST="${SCRIPT_DIR}/${FILE}"

if [ -f "$DEST" ]; then
  echo "已存在: $DEST"
  exit 0
fi

URLS=(
  "https://cmake.org/files/v3.31/${FILE}"
  "https://github.com/Kitware/CMake/releases/download/v${V}/${FILE}"
)

for url in "${URLS[@]}"; do
  echo "尝试: $url"
  if curl -fL --connect-timeout 15 --max-time 120 -o "$DEST" "$url"; then
    echo "已保存: $DEST"
    exit 0
  fi
  rm -f "$DEST"
done

echo "所有地址均失败。请用浏览器或下载工具下载 ${FILE} 后放到 script/ 目录。"
echo "官方: https://cmake.org/download/"
exit 1
