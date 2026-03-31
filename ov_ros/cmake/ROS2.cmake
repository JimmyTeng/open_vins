cmake_minimum_required(VERSION 3.3)

# Find ROS build system
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
# 与根目录 add_subdirectory(ov_core) 一起构建时仅有目标 ov_core_lib，无 ov_coreConfig.cmake；ov_init 已并入 ov_core
if(NOT TARGET ov_core_lib)
  find_package(ov_core REQUIRED)
endif()

# Describe ROS project
option(ENABLE_ROS "Enable or disable building with ROS (if it is found)" ON)
if (NOT ENABLE_ROS)
    message(FATAL_ERROR "Build with ROS1.cmake if you don't have ROS.")
endif ()
add_definitions(-DROS_AVAILABLE=2)

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${CERES_LIBRARIES}
        ${OpenCV_LIBRARIES}
)
# ament 包名列表（不含 ov_core：顶层工程用 target_link_libraries 链上 ov_core_lib）
set(ament_libraries_ros
        rclcpp
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        cv_bridge
        image_transport
)

##################################################
# Make the shared library
##################################################

# 滤波与仿真实现均在 ov_core；此处仅编译 ROS 可视化与桩
list(APPEND LIBRARY_SOURCES
        src/dummy.cpp
        src/ros/ROS2Visualizer.cpp
        src/ros/ROSVisualizerHelper.cpp
)
file(GLOB_RECURSE LIBRARY_HEADERS "src/*.h")
add_library(ov_msckf_lib SHARED ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
ament_target_dependencies(ov_msckf_lib ${ament_libraries_ros})
if(TARGET ov_core_lib)
  # 与 ament_target_dependencies 一致使用「无关键字」签名，避免与 ament 混用 PUBLIC/PLAIN
  target_link_libraries(ov_msckf_lib ov_core_lib)
else()
  ament_target_dependencies(ov_msckf_lib ov_core)
endif()
target_link_libraries(ov_msckf_lib ${thirdparty_libraries})
target_include_directories(ov_msckf_lib PUBLIC src/)
install(TARGETS ov_msckf_lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        PUBLIC_HEADER DESTINATION include
)
install(DIRECTORY src/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
ament_export_include_directories(include)
ament_export_libraries(ov_msckf_lib)

##################################################
# Make binary files!
##################################################

add_executable(run_subscribe_msckf src/run_subscribe_msckf.cpp)
ament_target_dependencies(run_subscribe_msckf ${ament_libraries_ros})
if(NOT TARGET ov_core_lib)
  ament_target_dependencies(run_subscribe_msckf ov_core)
endif()
target_link_libraries(run_subscribe_msckf ov_msckf_lib ${thirdparty_libraries})
install(TARGETS run_subscribe_msckf DESTINATION lib/${PROJECT_NAME})

add_executable(run_simulation src/run_simulation.cpp)
ament_target_dependencies(run_simulation ${ament_libraries_ros})
if(NOT TARGET ov_core_lib)
  ament_target_dependencies(run_simulation ov_core)
endif()
target_link_libraries(run_simulation ov_msckf_lib ${thirdparty_libraries})
install(TARGETS run_simulation DESTINATION lib/${PROJECT_NAME})

add_executable(test_sim_meas src/test_sim_meas.cpp)
ament_target_dependencies(test_sim_meas ${ament_libraries_ros})
if(NOT TARGET ov_core_lib)
  ament_target_dependencies(test_sim_meas ov_core)
endif()
target_link_libraries(test_sim_meas ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_sim_meas DESTINATION lib/${PROJECT_NAME})

add_executable(test_sim_repeat src/test_sim_repeat.cpp)
ament_target_dependencies(test_sim_repeat ${ament_libraries_ros})
if(NOT TARGET ov_core_lib)
  ament_target_dependencies(test_sim_repeat ov_core)
endif()
target_link_libraries(test_sim_repeat ov_msckf_lib ${thirdparty_libraries})
install(TARGETS test_sim_repeat DESTINATION lib/${PROJECT_NAME})

# Install launch and config directories
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch/)
install(DIRECTORY ../config/ DESTINATION share/${PROJECT_NAME}/config/)

# finally define this as the package
ament_package()
