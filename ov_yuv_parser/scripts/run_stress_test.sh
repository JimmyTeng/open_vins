#!/bin/bash
# 流式加载器压力测试运行脚本
# 用法: ./run_stress_test.sh [--frames N] [--iterations M] [--cache-size C]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build/x86_64/Release/x64-release-vcpkg"
EXE="${BUILD_DIR}/ov_yuv_parser/stress_test_stream_loader"

if [ ! -f "$EXE" ]; then
    # 尝试其他常见构建路径
    for d in "${PROJECT_ROOT}/build" "${PROJECT_ROOT}/build/ov_yuv_parser"; do
        if [ -f "${d}/stress_test_stream_loader" ]; then
            EXE="${d}/stress_test_stream_loader"
            break
        fi
    done
fi

if [ ! -f "$EXE" ]; then
    echo "错误: 未找到 stress_test_stream_loader，请先编译:"
    echo "  cmake --preset x64-release-vcpkg && cmake --build build --target stress_test_stream_loader"
    exit 1
fi

echo "运行: $EXE $*"
exec "$EXE" "$@"
