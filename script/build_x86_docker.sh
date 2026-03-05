#!/usr/bin/env bash
# 在 Ubuntu 18.04 + ROS Melodic 感知版 Docker 中编译 x86_64 版本（含 ROS）
# 用法：在项目根目录执行  ./script/build_x86_docker.sh
# 与 build_arm64_docker.sh 共用同一镜像 open-vins-ubuntu18.04
# 代理：宿主机设置 http_proxy/https_proxy 会自动传入容器，例如：
#   export https_proxy=http://127.0.0.1:7890 http_proxy=http://127.0.0.1:7890
#   ./script/build_x86_docker.sh

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

IMAGE_NAME="${1:-open-vins-ubuntu18.04}"
PRESET="${PRESET:-x64-release-vcpkg-docker}"
if [[ "$PRESET" != *-docker ]]; then
  echo "错误：在 Docker 构建中必须使用 -docker 的 preset，否则会污染宿主机 build-tool/vcpkg（root 权限）。"
  echo "当前 PRESET=$PRESET，请勿覆盖为非 -docker 预设，或直接使用默认（不设置 PRESET）。"
  exit 1
fi

echo "使用: ${DOCKER_CMD}"
echo "镜像名: ${IMAGE_NAME}"
echo "预设: ${PRESET}（x86_64 + ROS）"
echo "配置/构建目录: $(pwd)/build/x86_64/Release-docker/"
echo "安装目录: $(pwd)/install/x86_64/Release-docker"
if [ ! -f script/cmake-3.31.10-linux-x86_64.tar.gz ]; then
  echo "提示: 无 script/cmake-3.31.10-linux-x86_64.tar.gz，可先运行 ./script/download_cmake_for_docker.sh"
fi
if [ ! -d script/vcpkg-downloads ] || [ -z "$(ls -A script/vcpkg-downloads 2>/dev/null)" ]; then
  echo "提示: 容器内需从 GitHub 拉取 vcpkg 依赖（含 ninja），可先运行 ./script/download_ninja_for_docker.sh 与 ./script/download_cmake_for_docker.sh 预下载（使用国内源）"
fi
[ -n "${SKIP_CONFIGURE}" ] && echo "SKIP_CONFIGURE=1：不清空构建目录，不重新配置，只执行编译与安装"
[ -z "${CLEAN_CONFIGURE}" ] && [ -d "build/x86_64/Release-docker/x64-release-vcpkg-docker" ] && echo "复用已有构建目录与 .vcpkg-docker-cache-x64（不重新下载/编译依赖）"
echo ""

# 确保镜像存在（与 ARM 脚本共用，先构建或拉取）
if ! $DOCKER_CMD image inspect "$IMAGE_NAME" &>/dev/null; then
  echo "镜像 $IMAGE_NAME 不存在，请先运行 ./script/build_arm64_docker.sh 构建，或手动 build Dockerfile.ubuntu-18.04"
  exit 1
fi

export VCPKG_BINARY_SOURCES=clear
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
# 与 arm64 脚本对称：共用 .vcpkg-docker-tool，缓存按架构分离
VCPKG_CACHE_HOST="$(pwd)/.vcpkg-docker-cache-x64"
VCPKG_BUILDTREES_HOST="$(pwd)/.vcpkg-docker-buildtrees-x64"
mkdir -p "$VCPKG_CACHE_HOST" "$VCPKG_BUILDTREES_HOST" "$(pwd)/.vcpkg-docker-tool"
if [ -d script/vcpkg-downloads ] && [ -n "$(ls -A script/vcpkg-downloads 2>/dev/null)" ]; then
  mkdir -p "${VCPKG_CACHE_HOST}/downloads"
  N="$(( $(ls -1 script/vcpkg-downloads 2>/dev/null | wc -l) ))"
  cp -f script/vcpkg-downloads/* "${VCPKG_CACHE_HOST}/downloads/" 2>/dev/null || true
  echo "已同步 script/vcpkg-downloads/ -> .vcpkg-docker-cache-x64/downloads/（$N 个文件）"
fi
if [ -f script/cmake-3.31.10-linux-x86_64.tar.gz ]; then
  mkdir -p "${VCPKG_CACHE_HOST}/downloads"
  cp -n script/cmake-3.31.10-linux-x86_64.tar.gz "${VCPKG_CACHE_HOST}/downloads/" 2>/dev/null || true
fi

RUN_ARGS=(
  --rm
  -u "${HOST_UID}:${HOST_GID}"
  -v "$(pwd):/work" -w /work
  -v "${VCPKG_BUILDTREES_HOST}:/work/.vcpkg-docker-tool/buildtrees"
  -e PRESET="$PRESET"
  -e VCPKG_BINARY_SOURCES=clear
  -e PATH="/usr/local/bin:/usr/bin:/bin"
  -e CC="/usr/bin/gcc-8"
  -e CXX="/usr/bin/g++-8"
  -e VCPKG_MAX_CONCURRENCY="${VCPKG_MAX_CONCURRENCY:-8}"
)
# 传入宿主机代理到容器（用于 vcpkg/CMake 下载 GitHub 等）；容器内 127.0.0.1 指向容器自身，需改为宿主机地址
NEED_HOST_GATEWAY=
for _v in http_proxy HTTP_PROXY https_proxy HTTPS_PROXY no_proxy NO_PROXY all_proxy ALL_PROXY; do
  _val="${!_v:-}"
  [[ -z "$_val" ]] && continue
  if [[ "$_val" == *"127.0.0.1"* ]]; then
    [[ -z "${NEED_HOST_GATEWAY}" ]] && RUN_ARGS+=(--add-host=host.docker.internal:host-gateway) && NEED_HOST_GATEWAY=1
    _val="${_val//127.0.0.1/host.docker.internal}"
  fi
  RUN_ARGS+=(-e "${_v}=${_val}")
done
[[ -n "${https_proxy:-}${HTTPS_PROXY:-}${http_proxy:-}${HTTP_PROXY:-}" ]] && echo "已传入代理到容器: ${https_proxy:-${HTTPS_PROXY:-}}${http_proxy:-${HTTP_PROXY:-}}"

# 若曾以 root 跑过导致目录不可写，需在宿主机做一次性 chown（本脚本已用 -u，之后产出均为当前用户）。
NEED_CHOWN=
for _d in .vcpkg-docker-tool .vcpkg-docker-cache-x64 .vcpkg-docker-buildtrees-x64 build install; do
  if [ -d "$_d" ] && ! [ -w "$_d" ]; then NEED_CHOWN=1; break; fi
done
if [ -n "${NEED_CHOWN}" ]; then
  echo "错误：检测到上述目录为 root 所属或不可写，请先在宿主机执行一次："
  echo "  sudo chown -R $(id -u):$(id -g) .vcpkg-docker-tool .vcpkg-docker-cache-x64 .vcpkg-docker-buildtrees-x64 build install"
  echo "（若 .vcpkg-docker-tool 内已有 root 创建的子目录/文件，也会导致容器内 tar 解压报 Permission denied，需 chown -R 整棵目录。）"
  exit 1
fi

BUILD_DIR="build/x86_64/Release-docker/x64-release-vcpkg-docker"
# 若缓存里仍是旧 vcpkg 路径，清空构建目录以强制用 preset 重新配置
if [ -f "$BUILD_DIR/CMakeCache.txt" ] && grep -q '/tmp/vcpkg-docker' "$BUILD_DIR/CMakeCache.txt" 2>/dev/null; then
  echo "检测到构建目录使用旧 vcpkg 路径 /tmp/vcpkg-docker，清空以重新配置..."
  rm -rf "$BUILD_DIR" 2>/dev/null || $DOCKER_CMD run --rm -v "$(pwd):/work" -w /work "$IMAGE_NAME" rm -rf "/work/$BUILD_DIR"
fi
if [ -n "${CLEAN_CONFIGURE}" ] && [ -d "$BUILD_DIR" ]; then
  echo "CLEAN_CONFIGURE=1：清空 $BUILD_DIR..."
  if ! rm -rf "$BUILD_DIR" 2>/dev/null; then
    $DOCKER_CMD run --rm -v "$(pwd):/work" -w /work "$IMAGE_NAME" rm -rf "/work/$BUILD_DIR"
  fi
fi

# 需 source ROS 环境以启用 catkin（ENABLE_ROS 在 x86 下默认 ON）
ROS_SETUP='source /opt/ros/melodic/setup.bash'
VCPKG_COPY='mkdir -p /work/.vcpkg-docker-tool && (cd /work/build-tool/vcpkg && tar cf - --exclude=buildtrees --exclude=packages --exclude=downloads --exclude=.git .) | (cd /work/.vcpkg-docker-tool && tar xf -) && (rm -f /work/.vcpkg-docker-tool/.git && cp -a /work/.git/modules/build-tool/modules/vcpkg /work/.vcpkg-docker-tool/.git && git --git-dir=/work/.vcpkg-docker-tool/.git --work-tree=/work/.vcpkg-docker-tool config core.worktree /work/.vcpkg-docker-tool) 2>/dev/null || true'
VCPKG_PRELOAD_CMAKE='mkdir -p /work/.vcpkg-docker-cache-x64/downloads && cp /work/script/cmake-3.31.10-linux-x86_64.tar.gz /work/.vcpkg-docker-cache-x64/downloads/ 2>/dev/null; [ -d /work/script/vcpkg-downloads ] && cp -f /work/script/vcpkg-downloads/* /work/.vcpkg-docker-cache-x64/downloads/ 2>/dev/null; true'
RUN_CMAKE="${ROS_SETUP} && ${VCPKG_COPY} && ${VCPKG_PRELOAD_CMAKE} && export VCPKG_ROOT=/work/.vcpkg-docker-tool && cmake --preset \"\$PRESET\""
RUN_BUILD="${ROS_SETUP} && ${VCPKG_COPY} && export VCPKG_ROOT=/work/.vcpkg-docker-tool && cmake --build --preset \"\$PRESET\""
RUN_INSTALL="${ROS_SETUP} && ${VCPKG_COPY} && export VCPKG_ROOT=/work/.vcpkg-docker-tool && cmake --build --preset \"\$PRESET\" --target install"

if [ -z "${SKIP_CONFIGURE}" ]; then
  echo "[1/3] 配置（vcpkg + ROS，首次较久）..."
  $DOCKER_CMD run "${RUN_ARGS[@]}" "$IMAGE_NAME" bash -c "$RUN_CMAKE"
  echo ""
fi

echo "[2/3] 编译..."
$DOCKER_CMD run "${RUN_ARGS[@]}" "$IMAGE_NAME" bash -c "$RUN_BUILD"

echo ""
echo "[3/3] 安装..."
$DOCKER_CMD run "${RUN_ARGS[@]}" "$IMAGE_NAME" bash -c "$RUN_INSTALL"

echo ""
echo "[4/4] 同步 thirdparty..."
if [ -d "${BUILD_DIR}/vcpkg_installed/x64-linux-custom-release/lib" ]; then
  $DOCKER_CMD run "${RUN_ARGS[@]}" "$IMAGE_NAME" ./script/sync_thirdparty_from_vcpkg.sh x64-release-vcpkg-docker
else
  echo "提示：vcpkg_installed 不在预期路径，跳过 thirdparty 同步。若需部署，请手动执行："
  echo "  ./script/sync_thirdparty_from_vcpkg.sh x64-release-vcpkg-docker"
fi

echo ""
echo "完成。install/x86_64/Release-docker 已生成，含 ROS 节点，可直接运行（容器以当前用户运行，无需 chown/sudo）。"
