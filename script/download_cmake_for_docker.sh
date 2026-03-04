#!/usr/bin/env bash
# 预下载 CMake 用于 Docker 构建（优先国内镜像），避免镜像构建时网络拉取超时
# 用法：在项目根目录执行  ./script/download_cmake_for_docker.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CMAKE_VERSION="${CMAKE_VERSION:-3.31.10}"
FILENAME="cmake-${CMAKE_VERSION}-linux-x86_64.tar.gz"
OUTPUT="${SCRIPT_DIR}/${FILENAME}"
# v3.31.10 linux-x86_64 官方 sha256
SHA256_EXPECTED="3cb3dd247b6a1de2d0f4b20c6fd4326c9024e894cebc9dc8699758887e566ca7"

GITHUB_URL="https://github.com/Kitware/CMake/releases/download/v${CMAKE_VERSION}/${FILENAME}"
# 国内镜像优先，可设置 MIRROR_GH 指定单一镜像，如 MIRROR_GH=https://ghproxy.net
if [ -n "${MIRROR_GH}" ]; then
  URLS=("${MIRROR_GH}/${GITHUB_URL}")
else
  URLS=(
    "https://ghproxy.com/${GITHUB_URL}"
    "https://ghproxy.net/${GITHUB_URL}"
    "https://ghp.ci/${GITHUB_URL}"
    "https://mirror.ghproxy.com/${GITHUB_URL}"
    "${GITHUB_URL}"
  )
fi

if [ -f "${OUTPUT}" ]; then
  if command -v sha256sum &>/dev/null; then
    SHA256_ACTUAL="$(sha256sum < "${OUTPUT}" | awk '{print $1}')"
    if [ "${SHA256_ACTUAL}" = "${SHA256_EXPECTED}" ]; then
      echo "已存在且校验通过: ${OUTPUT}"
      exit 0
    fi
    echo "文件已存在但校验失败，重新下载..."
  else
    echo "已存在（跳过校验）: ${OUTPUT}"
    exit 0
  fi
fi

download_one() {
  local url="$1"
  if command -v curl &>/dev/null; then
    curl -fSL --connect-timeout 15 --max-time 300 "${url}" -o "${OUTPUT}"
  elif command -v wget &>/dev/null; then
    wget --timeout=15 -O "${OUTPUT}" "${url}"
  else
    echo "错误: 需要 curl 或 wget" >&2
    exit 1
  fi
}

for url in "${URLS[@]}"; do
  echo "尝试: ${url}"
  rm -f "${OUTPUT}"
  download_one "${url}" || true
  if [ -f "${OUTPUT}" ] && [ -s "${OUTPUT}" ]; then
    break
  fi
  echo "失败，尝试下一源..."
done

if [ ! -f "${OUTPUT}" ]; then
  echo "错误: 所有镜像源均失败。" >&2
  exit 1
fi

if command -v sha256sum &>/dev/null; then
  SHA256_ACTUAL="$(sha256sum < "${OUTPUT}" | awk '{print $1}')"
  if [ "${SHA256_ACTUAL}" != "${SHA256_EXPECTED}" ]; then
    echo "错误: sha256 校验失败" >&2
    rm -f "${OUTPUT}"
    exit 1
  fi
fi

echo "完成: ${OUTPUT}"
echo "可继续运行: ./script/build_arm64_docker.sh"
