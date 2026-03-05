cmake_minimum_required(VERSION 3.3)

# Find ROS build system
find_package(catkin QUIET COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs image_transport cv_bridge ov_yuv_parser)

# ROS include dirs: catkin_INCLUDE_DIRS may be empty with vcpkg toolchain; always add standard ROS path
set(ROS_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
if(EXISTS "/opt/ros/melodic/include")
  list(APPEND ROS_INCLUDE_DIRS "/opt/ros/melodic/include")
endif()
if(NOT catkin_INCLUDE_DIRS)
  message(STATUS "ov_ros: catkin_INCLUDE_DIRS empty, using /opt/ros/melodic/include")
endif()

# When using vcpkg/standalone CMake, catkin_LIBRARIES may be empty; add explicit ROS libs so linking succeeds
set(ROS_FALLBACK_LIBRARIES "")
if(NOT catkin_LIBRARIES AND EXISTS "/opt/ros/melodic/lib")
  message(STATUS "ov_ros: catkin_LIBRARIES empty, linking explicitly to /opt/ros/melodic libs")
  link_directories(/opt/ros/melodic/lib)
  set(ROS_FALLBACK_LIBRARIES
    roscpp rosconsole rostime roscpp_serialization xmlrpcpp
    cv_bridge image_transport
    tf tf2 tf2_ros
    cpp_common
    boost_system
    rosbag
    rosbag_storage
  )
endif()

# Describe ROS project
add_definitions(-DROS_AVAILABLE=1)
catkin_package(
        CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs image_transport cv_bridge ov_yuv_parser
        INCLUDE_DIRS src/
        LIBRARIES ov_msckf_lib
)


# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
        ${ROS_INCLUDE_DIRS}
        ${ov_yuv_parser_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${OpenCV_LIBRARIES}
        ${CERES_LIBRARIES}
        ${catkin_LIBRARIES}
        ${ROS_FALLBACK_LIBRARIES}
)

# If we are not building with ROS then we need to manually link to its headers
# This isn't that elegant of a way, but this at least allows for building without ROS
# If we had a root cmake we could do this: https://stackoverflow.com/a/11217008/7718197
# But since we don't we need to basically build all the cpp / h files explicitly :(

message(STATUS "MANUALLY LINKING TO OV_CORE LIBRARY....")
file(GLOB_RECURSE OVCORE_LIBRARY_SOURCES "${CMAKE_SOURCE_DIR}/../ov_core/src/*.cpp")
list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_profile\\.cpp$")
list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_webcam\\.cpp$")
list(FILTER OVCORE_LIBRARY_SOURCES EXCLUDE REGEX ".*test_tracking\\.cpp$")
list(APPEND LIBRARY_SOURCES ${OVCORE_LIBRARY_SOURCES})
include_directories(${CMAKE_SOURCE_DIR}/../ov_core/src/)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/../ov_core/src/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

message(STATUS "MANUALLY LINKING TO OV_INIT LIBRARY....")
file(GLOB_RECURSE OVINIT_LIBRARY_SOURCES "${CMAKE_SOURCE_DIR}/../ov_init/src/*.cpp")
list(FILTER OVINIT_LIBRARY_SOURCES EXCLUDE REGEX ".*test_dynamic_init\\.cpp$")
list(FILTER OVINIT_LIBRARY_SOURCES EXCLUDE REGEX ".*test_dynamic_mle\\.cpp$")
list(FILTER OVINIT_LIBRARY_SOURCES EXCLUDE REGEX ".*test_simulation\\.cpp$")
list(FILTER OVINIT_LIBRARY_SOURCES EXCLUDE REGEX ".*Simulator\\.cpp$")
list(APPEND LIBRARY_SOURCES ${OVINIT_LIBRARY_SOURCES})
include_directories(${CMAKE_SOURCE_DIR}/../ov_init/src/)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/../ov_init/src/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
    )



##################################################
# Make the shared library
##################################################



list(APPEND LIBRARY_SOURCES src/ros/ROS1Visualizer.cpp src/ros/ROSVisualizerHelper.cpp)

file(GLOB_RECURSE LIBRARY_HEADERS "src/*.h")
add_library(ov_msckf_lib SHARED ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
target_link_libraries(ov_msckf_lib
        PUBLIC ov_core_lib
        PUBLIC ${thirdparty_libraries})
target_include_directories(ov_msckf_lib PUBLIC
        src/
        ${ROS_INCLUDE_DIRS})
install(TARGETS ov_msckf_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY src/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)


##################################################
# Make binary files!
##################################################



add_executable(ros1_serial_msckf src/ros1_serial_msckf.cpp)
target_include_directories(ros1_serial_msckf PRIVATE ${ROS_INCLUDE_DIRS})
target_link_libraries(ros1_serial_msckf
        PUBLIC ov_core_lib
        PUBLIC ov_msckf_lib
        PUBLIC ${thirdparty_libraries})
install(TARGETS ros1_serial_msckf
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(run_subscribe_msckf src/run_subscribe_msckf.cpp)
target_include_directories(run_subscribe_msckf PRIVATE ${ROS_INCLUDE_DIRS})
target_link_libraries(run_subscribe_msckf
        PUBLIC ov_core_lib
        PUBLIC ov_msckf_lib
        PUBLIC ${thirdparty_libraries})
install(TARGETS run_subscribe_msckf
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


add_executable(run_simulation src/run_simulation.cpp)
target_include_directories(run_simulation PRIVATE ${ROS_INCLUDE_DIRS})
target_link_libraries(run_simulation
        PUBLIC ov_core_lib
        PUBLIC ov_msckf_lib
        PUBLIC ${thirdparty_libraries})
install(TARGETS run_simulation
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test_sim_meas src/test_sim_meas.cpp)
target_include_directories(test_sim_meas PRIVATE ${ROS_INCLUDE_DIRS})
target_link_libraries(test_sim_meas
        PUBLIC ov_core_lib
        PUBLIC ov_msckf_lib
        PUBLIC ${thirdparty_libraries})
install(TARGETS test_sim_meas
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test_sim_repeat src/test_sim_repeat.cpp)
target_include_directories(test_sim_repeat PRIVATE ${ROS_INCLUDE_DIRS})
target_link_libraries(test_sim_repeat
        PUBLIC ov_core_lib
        PUBLIC ov_msckf_lib
        PUBLIC ${thirdparty_libraries})
install(TARGETS test_sim_repeat
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# YUV/IMU publisher node (moved from ov_yuv_parser)
add_executable(ros_publisher_node src/ros_publisher_node.cpp)
target_include_directories(ros_publisher_node PRIVATE
        ${ROS_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/../ov_yuv_parser/src
        ${CMAKE_CURRENT_SOURCE_DIR}/../ov_yuv_parser/src/parser
)
target_link_libraries(ros_publisher_node
        PUBLIC ov_yuv_parser_lib
        PUBLIC ${thirdparty_libraries})
install(TARGETS ros_publisher_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

