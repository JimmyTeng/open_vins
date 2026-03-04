#!/usr/bin/env bash
# ARM64 交叉编译（宿主机本机，不用 Docker），使用 vcpkg 管理依赖
# 用法：在项目根目录执行  ./script/build_arm64_vcpkg.sh
#
# 依赖：gcc-aarch64-linux-gnu, g++-aarch64-linux-gnu, gfortran-aarch64-linux-gnu
#   sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu gfortran-aarch64-linux-gnu

set -e
cd "$(dirname "$0")/.."

PRESET="${PRESET:-arm64-release-vcpkg}"
BUILD_DIR="build/aarch64/Release/arm64-release-vcpkg"
INSTALL_PREFIX="install/aarch64/Release"

echo "预设: ${PRESET}"
echo "配置/构建目录: $(pwd)/${BUILD_DIR}"
echo "安装目录: $(pwd)/${INSTALL_PREFIX}"
[ -n "${SKIP_CONFIGURE}" ] && echo "SKIP_CONFIGURE=1：不重新配置，只编译与安装"
[ -n "${CLEAN_CONFIGURE}" ] && echo "CLEAN_CONFIGURE=1：清空构建目录"
echo ""

# 检查交叉编译工具链
for t in aarch64-linux-gnu-gcc aarch64-linux-gnu-g++ aarch64-linux-gnu-gfortran; do
  if ! command -v "$t" &>/dev/null; then
    echo "错误：缺少 $t。请执行：" >&2
    echo "  sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu gfortran-aarch64-linux-gnu" >&2
    exit 1
  fi
done

# 确保 vcpkg 可用
export VCPKG_ROOT="${VCPKG_ROOT:-$(pwd)/build-tool/vcpkg}"
if [[ ! -f "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake" ]]; then
  echo "错误：未找到 vcpkg，请检查 build-tool/vcpkg 或设置 VCPKG_ROOT" >&2
  exit 1
fi

if [ -n "${CLEAN_CONFIGURE}" ] && [ -d "$BUILD_DIR" ]; then
  echo "[0/4] 清空构建目录..."
  rm -rf "$BUILD_DIR"
  echo ""
fi

if [ -z "${SKIP_CONFIGURE}" ]; then
  echo "[1/4] 配置..."
  cmake --preset "$PRESET"
  echo ""
fi

echo "[2/4] 编译..."
cmake --build --preset "$PRESET"
echo ""

echo "[3/4] 安装..."
cmake --build --preset "$PRESET" --target install
echo ""

echo "[4/4] 同步 thirdparty（vcpkg + 系统库如 libgfortran，FAT32 兼容）..."
if [ -d "${BUILD_DIR}/vcpkg_installed/arm64-linux-custom/lib" ]; then
  ./script/sync_thirdparty_from_vcpkg.sh arm64-release-vcpkg
else
  echo "提示：vcpkg_installed 不在预期路径，跳过。若需部署请手动执行："
  echo "  ./script/sync_thirdparty_from_vcpkg.sh arm64-release-vcpkg"
fi

echo ""
echo "完成。${INSTALL_PREFIX} 已生成，拷贝到设备即可运行。"
