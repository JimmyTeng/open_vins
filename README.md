# Open VINS

## ARM64 一键构建

### 方式一：Docker（推荐，兼容 glibc 2.29 设备）

```bash
./script/build_arm64_docker.sh
```

产物：`install/aarch64/Release-docker/`

### 方式二：宿主机 vcpkg（不用 Docker）

```bash
# 需先安装交叉工具链
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu gfortran-aarch64-linux-gnu
./script/build_arm64_vcpkg.sh
```

产物：`install/aarch64/Release/`

---

## 部署到设备

构建完成后，**直接把安装目录拷到设备的对应路径**即可：

```
install/aarch64/Release-docker/  或  install/aarch64/Release/
├── bin/          # 可执行文件（run.sh, vio_yuv_runner 等）
├── lib/          # 项目库
├── config/       # OpenVINS 配置
└── thirdparty/
    └── lib/      # 依赖库（已去符号链接，FAT32 兼容）
```

```bash
# 拷贝到设备（示例）
scp -r install/aarch64/Release-docker/* user@device:/mnt/VIO/
# 或通过 SD 卡/U 盘拷贝
```

设备上运行：

```bash
cd /mnt/VIO
./bin/run.sh [数据目录] [配置文件]
```

---

## Cursor / VS Code 与 CMake Presets

项目使用 CMake Presets，若需代码分析/跳转：

1. 安装扩展：**CMake Tools**、**clangd**
2. 配置并生成 `compile_commands.json`：
   ```bash
   cmake --preset x64-debug-vcpkg
   cmake --build --preset x64-debug-vcpkg
   ```

---

## 其他脚本

| 脚本 | 说明 |
|------|------|
| `build_arm64_docker.sh` | ARM64 Docker 构建（glibc 2.29 兼容） |
| `build_arm64_vcpkg.sh` | ARM64 宿主机 vcpkg 构建 |
| `build_preset.sh` | 通用构建（x64/arm64、vcpkg/系统依赖） |
| `sync_thirdparty_from_vcpkg.sh` | 同步依赖到 thirdparty（FAT32 兼容） |
| `download_cmake_for_docker.sh` | 预下载 CMake（Docker 用） |
| `download_vcpkg_deps_for_docker.sh` | 预下载 vcpkg 依赖（Docker 用） |
