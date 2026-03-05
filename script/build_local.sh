#!/usr/bin/env bash
#
# 利用本地系统库编译 OpenVINS（不使用 vcpkg）
# 依赖：Eigen3、OpenCV、Ceres 等通过 apt 安装
#
# 用法：
#   ./script/build_local.sh              # Debug 模式
#   ./script/build_local.sh --release    # Release 模式
#   ./script/build_local.sh --ros        # 启用 ROS 构建（需 catkin 环境 + ov_init）
#   ./script/build_local.sh --clean      # 清理后重新配置
#   ./script/build_local.sh -j 8         # 指定并行任务数
#
# 安装系统依赖（首次使用前执行）：
#   sudo apt install -y build-essential ninja-build cmake pkg-config \
#     libeigen3-dev libopencv-dev libceres-dev libgflags-dev libgoogle-glog-dev
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_TYPE="${BUILD_TYPE:-Debug}"
BUILD_DIR="${PROJECT_ROOT}/build/x86_64/${BUILD_TYPE}/local"
INSTALL_PREFIX="${PROJECT_ROOT}/install/x86_64/${BUILD_TYPE}"
ENABLE_ROS="${ENABLE_ROS:-OFF}"
CLEAN=false
JOBS="$(nproc 2>/dev/null || echo 4)"

usage() {
  cat <<'EOF'
用法: ./script/build_local.sh [选项]

选项:
  --release          Release 模式（默认 Debug）
  --ros              启用 ROS 构建（需在 catkin 环境中，且需 ov_init）
  --clean            清理构建目录后重新配置
  -j, --jobs N       并行编译任务数
  -h, --help         显示帮助

说明:
  - 使用系统 apt 安装的 Eigen3、OpenCV、Ceres，不依赖 vcpkg
  - 安装依赖: sudo apt install libeigen3-dev libopencv-dev libceres-dev
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --release)
      BUILD_TYPE="Release"
      shift
      ;;
    --ros)
      ENABLE_ROS="ON"
      shift
      ;;
    --clean)
      CLEAN=true
      shift
      ;;
    -j|--jobs)
      [[ $# -ge 2 ]] || { echo "错误: $1 需要参数" >&2; exit 1; }
      JOBS="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知选项: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

# 更新路径
BUILD_DIR="${PROJECT_ROOT}/build/x86_64/${BUILD_TYPE}/local"
INSTALL_PREFIX="${PROJECT_ROOT}/install/x86_64/${BUILD_TYPE}"

echo "========================================"
echo "OpenVINS 本地库编译（无 vcpkg）"
echo "========================================"
echo "构建类型: ${BUILD_TYPE}"
echo "构建目录: ${BUILD_DIR}"
echo "安装前缀: ${INSTALL_PREFIX}"
echo "启用 ROS: ${ENABLE_ROS}"
echo "并行任务: ${JOBS}"
echo ""

# 检查必要依赖（find_package 会实际查找，这里仅做基本检测）
if ! pkg-config --exists eigen3 2>/dev/null; then
  echo "错误: 未找到 Eigen3，请安装: sudo apt install libeigen3-dev" >&2
  exit 1
fi
if ! (pkg-config --exists opencv4 2>/dev/null || pkg-config --exists opencv 2>/dev/null); then
  echo "错误: 未找到 OpenCV，请安装: sudo apt install libopencv-dev" >&2
  exit 1
fi
# Ceres 可能没有 pkg-config，通过头文件粗略判断
if [ ! -d "/usr/include/ceres" ] && [ ! -d "/usr/local/include/ceres" ]; then
  echo "错误: 未找到 Ceres，请安装: sudo apt install libceres-dev" >&2
  exit 1
fi

if [ "${ENABLE_ROS}" = "ON" ]; then
  if [ -z "${ROS_DISTRO:-}" ]; then
    echo "提示: ENABLE_ROS=ON 需先 source ROS 环境，例如:" >&2
    echo "  source /opt/ros/noetic/setup.bash" >&2
    echo "  source /path/to/workspace/devel/setup.bash  # 若使用 catkin 工作空间" >&2
  fi
  if [ ! -d "${PROJECT_ROOT}/../ov_init" ] && [ ! -d "${PROJECT_ROOT}/ov_init" ]; then
    echo "警告: ov_ros 依赖 ov_init，未找到 ov_init 目录，ROS 构建可能失败。" >&2
    echo "  若 ov_init 已并入 ov_core，请根据实际情况修改 ov_ros/cmake/ROS1.cmake" >&2
  fi
fi

# 清理
if [ "${CLEAN}" = "true" ] && [ -d "${BUILD_DIR}" ]; then
  echo "[1/4] 清理构建目录..."
  rm -rf "${BUILD_DIR}"
  echo ""
fi

# 配置
echo "[2/4] 配置 CMake..."
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"
cmake "${PROJECT_ROOT}" \
  -G Ninja \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_TOOLCHAIN_FILE="" \
  -DENABLE_ROS="${ENABLE_ROS}" \
  -DUSE_BLAS_LAPACK=ON

# 编译
echo ""
echo "[3/4] 编译..."
cmake --build . -j "${JOBS}"

# 安装
echo ""
echo "[4/4] 安装..."
cmake --install .

echo ""
echo "完成。安装目录: ${INSTALL_PREFIX}"
echo "运行示例: ${INSTALL_PREFIX}/bin/run.sh 或 ${INSTALL_PREFIX}/bin/vio_yuv_runner"
