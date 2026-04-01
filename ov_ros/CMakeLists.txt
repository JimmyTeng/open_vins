cmake_minimum_required(VERSION 3.3)
project(ov_ros)

# The OpenCV version needs to match the one used by cv_bridge otherwise you will get a segmentation fault!
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 QUIET)
find_package(Ceres REQUIRED)
message(STATUS "OPENCV: " ${OpenCV_VERSION} " | CERES: " ${Ceres_VERSION})

# By default we build with ROS, but you can disable this and just build as a library
option(ENABLE_ROS "Enable or disable building with ROS (if it is found)" ON)

# ArUco 依赖 opencv_contrib；默认关闭。需要时：-DENABLE_ARUCO_TAGS=ON
option(ENABLE_ARUCO_TAGS "Build with OpenCV ArUco tag tracking (requires opencv contrib aruco)" OFF)
if (ENABLE_ARUCO_TAGS)
    add_definitions(-DENABLE_ARUCO_TAGS=1)
    message(STATUS "ENABLE_ARUCO_TAGS: ON (opencv2/aruco.hpp required)")
else ()
    add_definitions(-DENABLE_ARUCO_TAGS=0)
    message(STATUS "ENABLE_ARUCO_TAGS: OFF")
endif ()

# C++17 for std::filesystem
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -fno-omit-frame-pointer")

# Find our ROS version!
# NOTE: Default to using the ROS1 package if both are in our enviroment
# NOTE: https://github.com/romainreignier/share_ros1_ros2_lib_demo
find_package(catkin QUIET COMPONENTS roscpp)
find_package(ament_cmake QUIET)
if (catkin_FOUND AND ENABLE_ROS)
    message(STATUS "ROS *1* version found, building ROS1.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
elseif (ament_cmake_FOUND AND ENABLE_ROS)
    message(STATUS "ROS *2* version found, building ROS2.cmake")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2.cmake)
else ()
    if (ENABLE_ROS)
        message(FATAL_ERROR
            "ENABLE_ROS=ON but neither catkin (ROS1) nor ament_cmake (ROS2) was found.\n"
            "Please source your ROS setup before configuring, e.g.:\n"
            "  source /opt/ros/noetic/setup.bash")
    else ()
        message(STATUS "ROS disabled, skipping ov_ros build")
    endif ()
endif ()

