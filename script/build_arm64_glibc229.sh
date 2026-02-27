#!/usr/bin/env bash
# 在 Ubuntu 18.04 Docker 中交叉编译 ARM64，生成兼容 glibc 2.29 设备的安装包
# 用法：在项目根目录执行  ./script/build_arm64_glibc229.sh
# 从国内源拉取基础镜像；换源示例：MIRROR=docker.1ms.run ./script/build_arm64_glibc229.sh

set -e
cd "$(dirname "$0")/.."

# 优先使用 docker，其次 podman
if command -v docker &>/dev/null; then
  DOCKER_CMD=docker
elif command -v podman &>/dev/null; then
  DOCKER_CMD=podman
else
  echo "未找到 docker 或 podman。请先安装其一："
  echo "  Docker:  sudo apt install docker.io  然后  sudo usermod -aG docker \$USER  并重新登录"
  echo "  Podman:  sudo apt install podman"
  exit 1
fi

IMAGE_NAME="${1:-open-vins-arm64-glibc229}"
# Docker 专用 preset：VCPKG_ROOT=/tmp/vcpkg-docker，缓存用 /root/.cache/vcpkg-rbs（挂载 .vcpkg-docker-cache），
# 与本地 build-tool/vcpkg 及 ~/.cache/vcpkg-rbs 完全分离，避免容器内 root 写回项目目录导致权限问题。
# 禁止在容器内使用非 -docker 的 preset（如 arm64-release-vcpkg），否则会污染宿主机 build-tool/vcpkg/buildtrees。
PRESET="${PRESET:-arm64-release-vcpkg-docker}"
if [[ "$PRESET" != *-docker ]]; then
  echo "错误：在 Docker 构建中必须使用 -docker 的 preset，否则会污染宿主机 build-tool/vcpkg（root 权限）。"
  echo "当前 PRESET=$PRESET，请勿覆盖为非 -docker 预设，或直接使用默认（不设置 PRESET）。"
  exit 1
fi

echo "使用: ${DOCKER_CMD}"
echo "镜像名: ${IMAGE_NAME}"
echo "预设: ${PRESET}"
echo "安装目录: $(pwd)/install/aarch64/Release"
if [ ! -f script/cmake-3.31.10-linux-x86_64.tar.gz ]; then
  echo "提示: 无 script/cmake-3.31.10-linux-x86_64.tar.gz，可先运行 ./script/download_cmake_for_docker.sh"
fi
if [ -z "${HTTPS_PROXY}${HTTP_PROXY}" ] && { [ ! -d script/vcpkg-downloads ] || [ -z "$(ls -A script/vcpkg-downloads 2>/dev/null)" ]; }; then
  echo "提示: 容器内连 GitHub 易超时，可先运行 ./script/download_vcpkg_deps_for_docker.sh 预下载依赖，或设置 HTTPS_PROXY 后重试"
fi
[ -n "${PASS_PROXY_TO_DOCKER}" ] && [ -n "${HTTP_PROXY}${HTTPS_PROXY}" ] && echo "已传入代理到容器（USE_HOST_NETWORK=1 时 127.0.0.1 可用）"
[ -n "${SKIP_CONFIGURE}" ] && echo "SKIP_CONFIGURE=1：不清空构建目录，不重新配置，只执行编译与安装"
[ -z "${CLEAN_CONFIGURE}" ] && [ -d "build/aarch64/Release/arm64-release-vcpkg-docker" ] && echo "复用已有构建目录与 .vcpkg-docker-cache（不重新下载/编译依赖）"
echo ""

# 构建镜像（若不存在）
if ! $DOCKER_CMD image inspect "$IMAGE_NAME" &>/dev/null; then
  # 确保有 ubuntu:18.04：优先本地 tar，否则从国内镜像站拉取
  if ! $DOCKER_CMD image inspect ubuntu:18.04 &>/dev/null; then
    if [ -f script/ubuntu-18.04-docker.tar ]; then
      echo "从 script/ubuntu-18.04-docker.tar 加载基础镜像..."
      $DOCKER_CMD load -i script/ubuntu-18.04-docker.tar
    else
      echo "从国内镜像站拉取 ubuntu:18.04（可设置 MIRROR 换源）..."
      MIRROR="${MIRROR:-docker.xuanyuan.me}"
      if ! $DOCKER_CMD pull "${MIRROR}/library/ubuntu:18.04"; then
        echo "当前镜像站 ${MIRROR} 拉取失败，可尝试: MIRROR=registry.cn-hangzhou.aliyuncs.com $0"
        exit 1
      fi
      $DOCKER_CMD tag "${MIRROR}/library/ubuntu:18.04" ubuntu:18.04
    fi
    echo ""
  fi
  echo "构建 Docker 镜像（基于 Ubuntu 18.04，glibc 2.27）..."
  if ! $DOCKER_CMD build -f script/Dockerfile.arm64-glibc229 -t "$IMAGE_NAME" .; then
    echo ""
    echo "构建失败。若卡在 CMake 下载：运行 ./script/download_cmake_for_docker.sh 或将 cmake-3.31.10-linux-x86_64.tar.gz 放到 script/ 后重试。"
    exit 1
  fi
  echo ""
fi

# 强制 vcpkg 在容器内从源码构建（不用宿主机/远程二进制缓存），否则依赖会是新 glibc，设备跑不了
# 缓存挂到宿主机，这样 SKIP_CONFIGURE=1 时新容器仍能用到配置阶段写入的 ninja/cmake 等路径
export VCPKG_BINARY_SOURCES=clear
VCPKG_CACHE_HOST="$(pwd)/.vcpkg-docker-cache"
mkdir -p "$VCPKG_CACHE_HOST"
# 把 script/vcpkg-downloads/ 预下载文件同步到缓存，容器内 vcpkg 会直接使用，无需再下
# 使用覆盖复制：若之前缓存里有过 SHA512 不符的文件，会被预下载的正确文件替换
if [ -d script/vcpkg-downloads ] && [ -n "$(ls -A script/vcpkg-downloads 2>/dev/null)" ]; then
  mkdir -p "${VCPKG_CACHE_HOST}/downloads"
  N="$(( $(ls -1 script/vcpkg-downloads 2>/dev/null | wc -l) ))"
  cp -f script/vcpkg-downloads/* "${VCPKG_CACHE_HOST}/downloads/" 2>/dev/null || true
  echo "已同步 script/vcpkg-downloads/ -> .vcpkg-docker-cache/downloads/（$N 个文件，已覆盖旧缓存）"
fi
# 若有 script/cmake tarball 也放进缓存
if [ -f script/cmake-3.31.10-linux-x86_64.tar.gz ]; then
  mkdir -p "${VCPKG_CACHE_HOST}/downloads"
  cp -n script/cmake-3.31.10-linux-x86_64.tar.gz "${VCPKG_CACHE_HOST}/downloads/" 2>/dev/null || true
fi
RUN_ARGS=(
  --rm
  -v "$(pwd):/work" -w /work
  -v "${VCPKG_CACHE_HOST}:/root/.cache/vcpkg-rbs"
  -e PRESET="$PRESET"
  -e VCPKG_BINARY_SOURCES=clear
)
# 仅当显式启用时才把宿主机代理传入容器，避免环境里残留的 HTTP_PROXY 导致容器连 host.docker.internal 失败
# 用法：PASS_PROXY_TO_DOCKER=1 HTTPS_PROXY=http://127.0.0.1:10808 ./script/build_arm64_glibc229.sh
# 若代理只监听 127.0.0.1，再加 USE_HOST_NETWORK=1
if [ -n "${PASS_PROXY_TO_DOCKER}" ] && [ -n "${HTTP_PROXY}${HTTPS_PROXY}" ]; then
  if [ -n "${USE_HOST_NETWORK}" ]; then
    RUN_ARGS+=( --network host )
    for _e in HTTP_PROXY HTTPS_PROXY http_proxy https_proxy; do
      [ -n "${!_e}" ] && RUN_ARGS+=( -e "$_e=${!_e}" )
    done
  else
    _proxy_for_container() {
      local v="$1"
      if [[ "$v" =~ ^(https?://)(127\.0\.0\.1|localhost)(:[0-9]+) ]]; then
        echo "${BASH_REMATCH[1]}host.docker.internal${BASH_REMATCH[3]}"
      else
        echo "$v"
      fi
    }
    RUN_ARGS+=( --add-host=host.docker.internal:host-gateway )
    for _e in HTTP_PROXY HTTPS_PROXY http_proxy https_proxy; do
      [ -n "${!_e}" ] && RUN_ARGS+=( -e "$_e=$(_proxy_for_container "${!_e}")" )
    done
  fi
fi

# 默认复用已有构建目录和 .vcpkg-docker-cache，不重新下载/编译依赖。
# CLEAN_CONFIGURE=1 时才清空并重新配置；SKIP_CONFIGURE=1 时只编译+安装。
BUILD_DIR="build/aarch64/Release/arm64-release-vcpkg-docker"
if [ -n "${CLEAN_CONFIGURE}" ] && [ -d "$BUILD_DIR" ]; then
  echo "CLEAN_CONFIGURE=1：清空 $BUILD_DIR..."
  if ! rm -rf "$BUILD_DIR" 2>/dev/null; then
    echo "目录内有 root 创建的文件，用容器清理..."
    $DOCKER_CMD run --rm -v "$(pwd):/work" -w /work "$IMAGE_NAME" rm -rf "$BUILD_DIR"
  fi
fi

# 容器内使用 vcpkg 副本；复制 git 模块后需修正 worktree 为 /tmp/vcpkg-docker，否则 git read-tree 报 cannot chdir
VCPKG_COPY='mkdir -p /tmp/vcpkg-docker && (cd /work/build-tool/vcpkg && tar cf - --exclude=buildtrees --exclude=packages --exclude=downloads .) | (cd /tmp/vcpkg-docker && tar xf -) && (rm -f /tmp/vcpkg-docker/.git && cp -a /work/.git/modules/build-tool/modules/vcpkg /tmp/vcpkg-docker/.git && git --git-dir=/tmp/vcpkg-docker/.git --work-tree=/tmp/vcpkg-docker config core.worktree /tmp/vcpkg-docker) 2>/dev/null || true'
# 若 script/ 下有 cmake 或 script/vcpkg-downloads/ 有预下载包，预填 vcpkg 的 downloads，避免从 GitHub 拉取超时
VCPKG_PRELOAD_CMAKE='mkdir -p /root/.cache/vcpkg-rbs/downloads && cp /work/script/cmake-3.31.10-linux-x86_64.tar.gz /root/.cache/vcpkg-rbs/downloads/ 2>/dev/null; [ -d /work/script/vcpkg-downloads ] && cp -f /work/script/vcpkg-downloads/* /root/.cache/vcpkg-rbs/downloads/ 2>/dev/null; true'
RUN_CMAKE="${VCPKG_COPY} && ${VCPKG_PRELOAD_CMAKE} && export VCPKG_ROOT=/tmp/vcpkg-docker && cmake --preset \"\$PRESET\""
RUN_BUILD="${VCPKG_COPY} && export VCPKG_ROOT=/tmp/vcpkg-docker && cmake --build --preset \"\$PRESET\""
RUN_INSTALL="${VCPKG_COPY} && export VCPKG_ROOT=/tmp/vcpkg-docker && cmake --build --preset \"\$PRESET\" --target install"

if [ -z "${SKIP_CONFIGURE}" ]; then
  echo "[1/3] 配置（vcpkg 将在容器内从源码构建依赖，首次较久）..."
  $DOCKER_CMD run "${RUN_ARGS[@]}" "$IMAGE_NAME" bash -c "$RUN_CMAKE"
  echo ""
fi

echo "[2/3] 编译..."
$DOCKER_CMD run "${RUN_ARGS[@]}" "$IMAGE_NAME" bash -c "$RUN_BUILD"

echo ""
echo "[3/3] 安装..."
$DOCKER_CMD run "${RUN_ARGS[@]}" "$IMAGE_NAME" bash -c "$RUN_INSTALL"

echo ""
echo "完成。install/aarch64/Release 已生成，可部署到 glibc 2.29 设备。"
