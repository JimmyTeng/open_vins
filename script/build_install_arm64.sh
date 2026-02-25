#!/usr/bin/env bash
# ARM64 编译并安装（含 vcpkg 依赖库）到 install/aarch64/Release
# 用法：在项目根目录执行  ./script/build_install_arm64.sh

set -e
cd "$(dirname "$0")/.."

PRESET="${1:-arm64-release-vcpkg}"
echo "使用预设: ${PRESET}"
echo "安装目录: $(pwd)/install/aarch64/Release"
echo ""

echo "[1/3] 配置 (首次会通过 vcpkg 安装依赖，可能较久)..."
cmake --preset "$PRESET"

echo ""
echo "[2/3] 编译..."
cmake --build --preset "$PRESET"

echo ""
echo "[3/3] 安装（含依赖 .so 到 install/aarch64/Release）..."
cmake --build --preset "$PRESET" --target install

echo ""
echo "完成。安装内容："
echo "  - 可执行文件与库: install/aarch64/Release/bin, install/aarch64/Release/lib"
echo "  - vcpkg 依赖 .so 已复制到 install/aarch64/Release/lib"
echo "若遇 vcpkg 锁占用，可先执行: ./script/setup_vcpkg_local.sh  再传入预设: $0 arm64-release-vcpkg-local"
