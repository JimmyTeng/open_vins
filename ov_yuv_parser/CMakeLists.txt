cmake_minimum_required(VERSION 3.5)
project(ov_yuv_parser)

# =============================================================================
# Dependencies
# =============================================================================
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)

# =============================================================================
# Library: ov_yuv_parser_lib
# =============================================================================
add_library(ov_yuv_parser_lib SHARED
    src/parser/yuv_parser.cpp
    src/parser/imu_parser.cpp
    src/system/vio_interface.cpp
)
target_include_directories(ov_yuv_parser_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/app
    ${CMAKE_CURRENT_SOURCE_DIR}/src/example
    ${CMAKE_CURRENT_SOURCE_DIR}/src/parser
    ${CMAKE_CURRENT_SOURCE_DIR}/src/system
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(ov_yuv_parser_lib PUBLIC
    ${OpenCV_LIBS}
    ov_core_lib
)
install(TARGETS ov_yuv_parser_lib
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# 安装后从 bin/ 运行时在 ../lib 查找 libov_yuv_parser_lib.so，无需 LD_LIBRARY_PATH
set(OV_YUV_PARSER_INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")

# =============================================================================
# Executable: yuv_to_png
# =============================================================================
add_executable(yuv_to_png src/app/yuv_to_png.cpp)
target_link_libraries(yuv_to_png PRIVATE ov_yuv_parser_lib)
target_compile_options(yuv_to_png PRIVATE -Wall -Wextra)
set_target_properties(yuv_to_png PROPERTIES INSTALL_RPATH "${OV_YUV_PARSER_INSTALL_RPATH}")
install(TARGETS yuv_to_png RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# =============================================================================
# Executable: data_bridge_example (data_bridge.h 使用示例)
# =============================================================================
add_executable(data_bridge_example src/example/data_bridge_example.cpp)
target_link_libraries(data_bridge_example PRIVATE ov_yuv_parser_lib)
target_compile_options(data_bridge_example PRIVATE -Wall -Wextra)
set_target_properties(data_bridge_example PROPERTIES INSTALL_RPATH "${OV_YUV_PARSER_INSTALL_RPATH}")
install(TARGETS data_bridge_example RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# =============================================================================
# Executable: vio_yuv_runner (vio_interface API 测试：YUV+IMU 跑 VIO)
# =============================================================================
add_executable(vio_yuv_runner src/app/vio_yuv_runner.cpp)
target_link_libraries(vio_yuv_runner PRIVATE ov_yuv_parser_lib)
target_compile_options(vio_yuv_runner PRIVATE -Wall -Wextra)
set_target_properties(vio_yuv_runner PROPERTIES INSTALL_RPATH "${OV_YUV_PARSER_INSTALL_RPATH}")
install(TARGETS vio_yuv_runner RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
