#!/usr/bin/env bash
# 预下载 Ninja 用于 Docker 内 vcpkg（detect_compiler 等会用到），避免容器内拉取 GitHub 失败
# 用法：在项目根目录执行  ./script/download_ninja_for_docker.sh
# 输出：script/vcpkg-downloads/ninja-linux-1.13.2.zip（与 build_*_docker.sh 使用的缓存目录一致）

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${SCRIPT_DIR}/vcpkg-downloads"
ARCHIVE="ninja-linux-1.13.2.zip"
OUTPUT="${OUT_DIR}/${ARCHIVE}"

# vcpkg 使用的 URL，保存为 vcpkg 期望的 archive 名
URL="https://github.com/ninja-build/ninja/releases/download/v1.13.2/ninja-linux.zip"
if [ -n "${MIRROR_GH}" ]; then
  URLS=("${MIRROR_GH}/${URL}")
else
  URLS=(
    "https://ghproxy.com/${URL}"
    "https://ghproxy.net/${URL}"
    "https://ghp.ci/${URL}"
    "https://mirror.ghproxy.com/${URL}"
    "${URL}"
  )
fi

mkdir -p "${OUT_DIR}"

if [ -f "${OUTPUT}" ] && [ -s "${OUTPUT}" ]; then
  echo "已存在: ${OUTPUT}"
  echo "可继续运行: ./script/build_arm64_docker.sh 或 ./script/build_x86_docker.sh"
  exit 0
fi

download_one() {
  local url="$1"
  if command -v curl &>/dev/null; then
    curl -fSL --connect-timeout 15 --max-time 120 "${url}" -o "${OUTPUT}"
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

if [ ! -f "${OUTPUT}" ] || [ ! -s "${OUTPUT}" ]; then
  echo "错误: 所有镜像源均失败。" >&2
  exit 1
fi

echo "完成: ${OUTPUT}"
echo "可继续运行: ./script/build_arm64_docker.sh 或 ./script/build_x86_docker.sh"
