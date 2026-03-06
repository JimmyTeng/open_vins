#!/usr/bin/env bash
# 强制完整重建以应用体积优化（-ffunction-sections -fdata-sections + --gc-sections）
# 会清空 build 与 vcpkg 缓存，确保依赖用“按段编译”重新构建，链接时剔除未使用符号。
#
# 用法：
#   ./script/rebuild_for_size.sh x64-release-vcpkg
#   ./script/rebuild_for_size.sh arm64-release-vcpkg
#   ./script/rebuild_for_size.sh arm64-release-vcpkg-docker
#   ./script/rebuild_for_size.sh x64-release-vcpkg-docker

set -e
cd "$(dirname "$0")/.."

PRESET="${1:-}"
if [[ -z "$PRESET" ]]; then
  echo "用法: $0 <preset>" >&2
  echo "  x64-release-vcpkg | arm64-release-vcpkg | arm64-release-vcpkg-docker | x64-release-vcpkg-docker" >&2
  exit 1
fi

case "$PRESET" in
  x64-release-vcpkg)
    BUILD_DIR="build/x86_64/Release/x64-release-vcpkg"
    CACHE_CLEAR=""
    RBS_CACHE="${HOME}/.cache/vcpkg-rbs"
    USE_PRESET=1
    ;;
  arm64-release-vcpkg)
    BUILD_DIR="build/aarch64/Release/arm64-release-vcpkg"
    CACHE_CLEAR=""
    RBS_CACHE="${HOME}/.cache/vcpkg-rbs"
    USE_PRESET=1
    ;;
  arm64-release-vcpkg-docker)
    BUILD_DIR="build/aarch64/Release-docker/arm64-release-vcpkg-docker"
    CACHE_CLEAR=".vcpkg-docker-cache-arm64"
    RBS_CACHE=""
    USE_PRESET=0
    ;;
  x64-release-vcpkg-docker)
    BUILD_DIR="build/x86_64/Release-docker/x64-release-vcpkg-docker"
    CACHE_CLEAR=".vcpkg-docker-cache-x64"
    RBS_CACHE=""
    USE_PRESET=0
    ;;
  *)
    echo "错误：未知 preset: $PRESET" >&2
    exit 1
    ;;
esac

echo "Preset: $PRESET"
echo " 将清空: $BUILD_DIR"
[[ -n "$CACHE_CLEAR" ]] && echo " 将清空 vcpkg 缓存: $CACHE_CLEAR"
[[ -n "${RBS_CACHE:-}" ]] && echo " 将清空宿主机 vcpkg 二进制缓存: ${RBS_CACHE}（强制依赖按段重编）"
echo ""

if [[ -d "$BUILD_DIR" ]]; then
  echo "[1] 删除 $BUILD_DIR ..."
  rm -rf "$BUILD_DIR"
fi
if [[ -n "$CACHE_CLEAR" ]] && [[ -d "$CACHE_CLEAR" ]]; then
  echo "[2] 删除 vcpkg 缓存 $CACHE_CLEAR（强制依赖重新编译）..."
  rm -rf "$CACHE_CLEAR" 2>/dev/null || true
  if [[ -d "$CACHE_CLEAR" ]]; then
    chmod -R u+wx "$CACHE_CLEAR" 2>/dev/null || true
    rm -rf "$CACHE_CLEAR" 2>/dev/null || true
  fi
  if [[ -d "$CACHE_CLEAR" ]]; then
    echo " 警告：无法完全删除 $CACHE_CLEAR（可能被占用或为挂载点），继续构建。若需清空请手动删除。" >&2
  fi
fi
if [[ -n "${RBS_CACHE:-}" ]] && [[ -d "${RBS_CACHE}" ]]; then
  echo "[3] 删除宿主机 vcpkg 二进制缓存 ${RBS_CACHE} ..."
  rm -rf "${RBS_CACHE}"
fi
if [[ -n "$CACHE_CLEAR" ]]; then
  echo "[*] 使用 Docker 构建..."
  if [[ "$PRESET" == "arm64-release-vcpkg-docker" ]]; then
    ./script/build_arm64_docker.sh
  else
    [[ -f ./script/build_x86_docker.sh ]] && ./script/build_x86_docker.sh || { echo "未找到 script/build_x86_docker.sh，请手动执行 Docker 构建" >&2; exit 1; }
  fi
else
  echo "[*] 重新配置并构建（已清空缓存，vcpkg 将从源码重编依赖）..."
  cmake --preset "$PRESET"
  cmake --build --preset "$PRESET"
  cmake --build --preset "$PRESET" --target install
  echo ""
  echo "完成。若需同步 thirdparty：./script/sync_thirdparty_from_vcpkg.sh $PRESET"
fi
