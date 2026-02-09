cmake_minimum_required(VERSION 3.3)
project(ov_yuv_parser)

# NOTE: ov_core, ov_init, and ov_msckf are no longer catkin packages
# They are built as regular CMake libraries and linked directly

# Find catkin and ROS packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    cv_bridge
)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find Eigen3, Boost, Ceres (for OpenVINS)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)

# Get paths to OpenVINS packages
get_filename_component(OV_CORE_DIR "../ov_core" ABSOLUTE)
get_filename_component(OV_INIT_DIR "../ov_init" ABSOLUTE)
get_filename_component(OV_MSCKF_DIR "../ov_msckf" ABSOLUTE)

# Include directories
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    # OpenVINS 包含目录
    ${OV_MSCKF_DIR}/src
    ${OV_CORE_DIR}/src
    ${OV_INIT_DIR}/src
)

# Catkin package configuration (must be before add_executable)
catkin_package(
    CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
    INCLUDE_DIRS src/
    LIBRARIES ov_yuv_parser_lib
)

# Create ov_yuv_parser_lib for reuse by other packages (e.g. ov_ros)
add_library(ov_yuv_parser_lib STATIC
    src/yuv_parser.cpp
    src/imu_parser.cpp
)
target_include_directories(ov_yuv_parser_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(ov_yuv_parser_lib PUBLIC ${OpenCV_LIBS})
install(TARGETS ov_yuv_parser_lib ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Source files for yuv_parser executable
set(YUV_PARSER_SOURCES
        src/main.cpp
)

# Source files for YUV to PNG converter
set(YUV_TO_PNG_SOURCES
        src/yuv_to_png.cpp
)

# Source files for VIO YUV runner (暂时注释，因为 vio_interface API 不存在)
# set(VIO_YUV_RUNNER_SOURCES
#     src/vio_yuv_runner.cpp
#     src/yuv_parser.cpp
#     src/imu_parser.cpp
#     ../open_vins/ov_msckf/src/api/vio_interface.cpp
# )

# Create yuv_parser executable
add_executable(ov_yuv_parser ${YUV_PARSER_SOURCES})

# Create YUV to PNG converter
add_executable(yuv_to_png ${YUV_TO_PNG_SOURCES})

# Create VIO YUV runner (暂时注释，因为 vio_interface API 不存在)
# add_executable(vio_yuv_runner ${VIO_YUV_RUNNER_SOURCES})

# Link libraries for ov_yuv_parser
target_link_libraries(ov_yuv_parser
    ov_yuv_parser_lib
)

# Link libraries for YUV to PNG converter
target_link_libraries(yuv_to_png
    ov_yuv_parser_lib
)

# Link libraries for VIO YUV runner (暂时注释)
# target_link_libraries(vio_yuv_runner
#     ov_msckf_lib
#     ov_core_lib
#     ov_init_lib
#     ${OpenCV_LIBS}
#     ${Boost_LIBRARIES}
#     ${CERES_LIBRARIES}
# )

# Set include directories for targets (暂时注释)
# target_include_directories(vio_yuv_runner PRIVATE
#     ${OV_MSCKF_DIR}/src
#     ${OV_CORE_DIR}/src
#     ${OV_INIT_DIR}/src
# )

# Compile flags
target_compile_options(ov_yuv_parser PRIVATE
    -Wall
    -Wextra
)

target_compile_options(yuv_to_png PRIVATE
    -Wall
    -Wextra
)

# target_compile_options(vio_yuv_runner PRIVATE
#     -Wall
#     -Wextra
#     -O3
# )

# Install executables
install(TARGETS ov_yuv_parser yuv_to_png
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
message(STATUS "OV_CORE_DIR: ${OV_CORE_DIR}")
message(STATUS "OV_INIT_DIR: ${OV_INIT_DIR}")
message(STATUS "OV_MSCKF_DIR: ${OV_MSCKF_DIR}")
