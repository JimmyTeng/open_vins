# 脚本说明

`script/` 目录包含构建、运行及交叉编译相关脚本。下文重点介绍 **ARM64 交叉编译** 及兼容 glibc 2.29 设备的构建流程。

---

## ARM64 交叉编译（兼容 glibc 2.29）

在 x86_64 主机上为 ARM64 设备生成可执行文件，且保证在 **glibc 2.29** 及以上系统上可运行（例如部分嵌入式 Linux 或旧版 Ubuntu ARM64）。

### 原理简述

- 使用 **Docker/Podman** 在 Ubuntu 18.04 容器内构建（系统 glibc 2.27，生成的二进制可在 glibc 2.29 上运行）。
- 使用 **vcpkg** 在容器内从源码编译依赖，避免使用宿主机或远程二进制缓存（否则可能链接到更高版本 glibc，目标设备无法运行）。
- 通过 CMake Preset `arm64-release-vcpkg-docker` 指定交叉编译工具链 `aarch64-linux-gnu-gcc/g++` 及 vcpkg 在容器内的路径。

### 环境要求

- **Docker** 或 **Podman**（任选其一）
  - Docker: `sudo apt install docker.io`，然后 `sudo usermod -aG docker $USER` 并重新登录
  - Podman: `sudo apt install podman`
- 项目根目录下已存在 `build-tool/vcpkg`（git submodule 等已就绪）

### 基本用法

在项目根目录执行：

```bash
./script/build_arm64_glibc229.sh
```

- 若本地没有镜像 `open-vins-arm64-glibc229`，脚本会先构建 Docker 镜像（基于 `script/Dockerfile.arm64-glibc229`），再在容器内完成配置、编译、安装。
- 产物输出到 **`install/aarch64/Release`**，可直接拷贝到 glibc 2.29 的 ARM64 设备上使用。

### 可选：使用国内镜像拉取基础镜像

若拉取 `ubuntu:18.04` 较慢，可指定镜像站：

```bash
# 默认使用 docker.xuanyuan.me
MIRROR=docker.xuanyuan.me ./script/build_arm64_glibc229.sh

# 若失败可尝试阿里云
MIRROR=registry.cn-hangzhou.aliyuncs.com ./script/build_arm64_glibc229.sh
```

也可事先导入本地 tar 包，避免拉取：

- 将 `ubuntu:18.04` 保存为 `script/ubuntu-18.04-docker.tar`，脚本会优先执行 `docker load -i script/ubuntu-18.04-docker.tar`。

### 预下载依赖（推荐，避免容器内网络超时）

容器内访问 GitHub 可能较慢或超时，建议在宿主机先下载好再给容器用：

1. **CMake**（vcpkg 需要 ≥3.31.10）  
   在项目根目录执行：
   ```bash
   ./script/download_cmake_for_docker.sh
   ```
   会得到 `script/cmake-3.31.10-linux-x86_64.tar.gz`，构建镜像时会自动使用，无需在容器内下载。

2. **vcpkg 依赖**（如 Boost、OpenCV、Ceres 等）  
   在项目根目录执行：
   ```bash
   ./script/download_vcpkg_deps_for_docker.sh
   ```
   会下载到 `script/vcpkg-downloads/`。  
   脚本运行时会把该目录同步到 `.vcpkg-docker-cache/downloads/`，容器内 vcpkg 直接使用，避免在容器里拉 GitHub。

   若 GitHub 访问较慢，可使用镜像站（例如）：
   ```bash
   VCPKG_GH_MIRROR=https://ghproxy.com/https://github.com ./script/download_vcpkg_deps_for_docker.sh
   ```

### 可选：代理传入容器

若宿主机有 HTTP/HTTPS 代理，且希望**容器内**通过该代理访问外网（如 GitHub），可设置环境变量后执行脚本。代理地址与端口请按实际填写（将 `<代理IP>`、`<端口>` 替换为你的代理 IP 和端口）：

```bash
# 传入代理到容器（代理地址、端口自定义）
export HTTP_PROXY=http://<代理IP>:<端口>
export HTTPS_PROXY=http://<代理IP>:<端口>
PASS_PROXY_TO_DOCKER=1 ./script/build_arm64_glibc229.sh
```

- 仅当设置 `PASS_PROXY_TO_DOCKER=1` 时才会把 `HTTP_PROXY`/`HTTPS_PROXY` 传入容器；不设置则不会使用代理。
- 若代理只监听本机（如 `127.0.0.1:<端口>`），需同时使用宿主机网络，否则容器无法访问：
  ```bash
  export HTTPS_PROXY=http://127.0.0.1:<端口>
  USE_HOST_NETWORK=1 PASS_PROXY_TO_DOCKER=1 ./script/build_arm64_glibc229.sh
  ```

预下载依赖时也可走代理（同样自定义 IP 与端口）：
```bash
HTTPS_PROXY=http://<代理IP>:<端口> ./script/download_vcpkg_deps_for_docker.sh
```

### 构建行为控制

| 变量 | 说明 |
|------|------|
| `CLEAN_CONFIGURE=1` | 清空 `build/aarch64/Release/arm64-release-vcpkg-docker`，下次运行会重新配置并完整构建。 |
| `SKIP_CONFIGURE=1` | 不重新配置，只执行编译和安装（复用已有 CMake 配置）。 |

示例：

```bash
# 完全重新配置并构建
CLEAN_CONFIGURE=1 ./script/build_arm64_glibc229.sh

# 只编译 + 安装，不重新 cmake 配置
SKIP_CONFIGURE=1 ./script/build_arm64_glibc229.sh
```

默认会复用已有构建目录和 `.vcpkg-docker-cache`，以加快后续构建。

### 自定义镜像名

第一个参数为镜像名，默认为 `open-vins-arm64-glibc229`：

```bash
./script/build_arm64_glibc229.sh my-arm64-builder
```

### 相关文件

- **`script/build_arm64_glibc229.sh`** — 主入口脚本（Docker/Podman 拉镜像、挂载、配置/编译/安装）。
- **`script/Dockerfile.arm64-glibc229`** — 构建环境镜像（Ubuntu 18.04 + 交叉工具链 + CMake/Ninja 等）。
- **`build-tool/cmake/CMakePresets.json`** — 定义 `arm64-release-vcpkg` 与 `arm64-release-vcpkg-docker` preset。
- **`build-tool/toolchains/aarch64-linux-gnu.cmake`** — ARM64 交叉编译工具链文件。
- **`script/download_cmake_for_docker.sh`** — 预下载 CMake，供 Docker 构建使用。
- **`script/download_vcpkg_deps_for_docker.sh`** — 预下载 vcpkg 依赖，减少容器内网络依赖。

---

## 其他脚本（简要）

- **`build_preset.sh`** — 使用 CMake Preset 在宿主机本地构建（x64/arm64 等），例如：  
  `./script/build_preset.sh arm64-release-vcpkg --install`
- **`build_install_arm64.sh`** — 宿主机本地 ARM64 构建（需本机已安装 `aarch64-linux-gnu-*` 和 vcpkg）。
- **`run_release.sh`** / **`run_vio_yuv_runner.sh`** — 运行已构建的 Release 或 VIO YUV 示例。

以上与「在 Docker 中为 glibc 2.29 设备交叉编译」不同：若目标设备 glibc 较老，请以 **`build_arm64_glibc229.sh`** 为准。
