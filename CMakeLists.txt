cmake_minimum_required(VERSION 3.16)
project(open_vins)

# C++17 for std::filesystem (replacing Boost.Filesystem)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ARM 优化：-O3 -mcpu=native（本机编译）或 -march=armv8-a+simd（交叉编译）
option(OPENVINS_ARM_OPTIMIZE "Add ARM-specific flags (-mcpu=native or -march=armv8-a+simd)" OFF)
if(OPENVINS_ARM_OPTIMIZE AND CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64" OR CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    # 本机 ARM 编译：-mcpu=native 发挥当前 CPU 最佳性能
    if(NOT CMAKE_CROSSCOMPILING)
      add_compile_options(-mcpu=native)
      message(STATUS "ARM optimize: -mcpu=native (native build)")
    else()
      # 交叉编译：使用通用 ARMv8-A + NEON
      add_compile_options(-march=armv8-a+simd)
      message(STATUS "ARM optimize: -march=armv8-a+simd (cross-compile)")
    endif()
  endif()
endif()

# 抑制 GCC 与 Eigen 头文件相关的已知误报
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Wno-psabi)
  # Eigen CoreEvaluators.h 等处的 -Wmaybe-uninitialized 误报
  add_compile_options(-Wno-maybe-uninitialized)
endif()

# 通过编译器强制包含 <cassert>，解决部分 .cpp 使用 assert 但未包含头文件的问题（不修改源码）
add_compile_options(-include cassert)
# ARM 目标默认关闭 ROS（嵌入式部署无 ROS）；x86_64 默认开启
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|armv[0-9]")
  set(ENABLE_ROS_DEFAULT OFF)
else()
  set(ENABLE_ROS_DEFAULT ON)
endif()
option(ENABLE_ROS "Enable or disable building with ROS (if it is found)" ${ENABLE_ROS_DEFAULT})

# 将 RPATH 写入二进制，使 bin/ 与 lib/ 能从 ../lib 与 ../thirdparty/lib 加载依赖，无需 run.sh
set(CMAKE_INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}:$ORIGIN:$ORIGIN/../thirdparty/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)
add_subdirectory(ov_core)
add_subdirectory(ov_yuv_parser)
if(ENABLE_ROS)
    add_subdirectory(ov_ros)
endif()

# 不安装第三方库与头文件：部署时请使用项目内 thirdparty/lib 或配置 LD_LIBRARY_PATH
# 安装运行脚本到 bin，部署到设备后可直接 ./run.sh 或 ./run.sh <数据目录>
install(PROGRAMS script/run_vio_yuv_runner.sh DESTINATION "${CMAKE_INSTALL_BINDIR}" RENAME "run.sh")

# Release 安装后对 lib/*.so* 与 bin/* 执行 strip，减小部署体积（可选关闭：-DOPENVINS_INSTALL_STRIP=OFF）
set(OPENVINS_INSTALL_STRIP ON CACHE BOOL "Strip installed binaries and .so in Release build")
if(OPENVINS_INSTALL_STRIP AND CMAKE_BUILD_TYPE STREQUAL "Release" AND CMAKE_STRIP)
  install(CODE "
    set(_prefix \"\$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}\")
    set(_libdir \"\${_prefix}/${CMAKE_INSTALL_LIBDIR}\")
    set(_bindir \"\${_prefix}/${CMAKE_INSTALL_BINDIR}\")
    if(EXISTS \"\${_libdir}\")
      file(GLOB _libs \"\${_libdir}/*.so*\")
      foreach(_f \${_libs})
        if(NOT IS_DIRECTORY \"\${_f}\")
          execute_process(COMMAND \"${CMAKE_STRIP}\" --strip-unneeded \"\${_f}\" RESULT_VARIABLE _r)
        endif()
      endforeach()
    endif()
    if(EXISTS \"\${_bindir}\")
      file(GLOB _bins \"\${_bindir}/*\")
      foreach(_f \${_bins})
        if(NOT IS_DIRECTORY \"\${_f}\")
          execute_process(COMMAND \"${CMAKE_STRIP}\" \"\${_f}\" RESULT_VARIABLE _r)
        endif()
      endforeach()
    endif()
  ")
endif()
