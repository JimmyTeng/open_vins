cmake_minimum_required(VERSION 3.9)
project(ov_core)

# Include libraries (if we don't have opencv 4, then fallback to opencv 3)
# The OpenCV version needs to match the one used by cv_bridge otherwise you will get a segmentation fault!
find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 QUIET)
if (NOT OpenCV_FOUND)
    find_package(OpenCV 4 REQUIRED)
endif ()
find_package(Ceres REQUIRED)
message(STATUS "OPENCV: " ${OpenCV_VERSION})

# By default we build with ROS, but you can disable this and just build as a library
option(ENABLE_ROS "Enable or disable building with ROS (if it is found)" OFF)

# If we will compile with aruco support
option(ENABLE_ARUCO_TAGS "Enable or disable aruco tag (disable if no contrib modules)" ON)
if (NOT ENABLE_ARUCO_TAGS)
    add_definitions(-DENABLE_ARUCO_TAGS=0)
    message(WARNING "DISABLING ARUCOTAG TRACKING!")
else ()
    add_definitions(-DENABLE_ARUCO_TAGS=1)
endif ()

# check if we have our python libs files (will search for python3 then python2 installs)
# sudo apt-get install python-matplotlib python-numpy python-dev
# https://cmake.org/cmake/help/v3.10/module/FindPythonLibs.html
find_package(PythonLibs QUIET)
option(DISABLE_MATPLOTLIB "Disable or enable matplotlib plot scripts in ov_eval" OFF)
if (PYTHONLIBS_FOUND AND NOT DISABLE_MATPLOTLIB)
    add_definitions(-DHAVE_PYTHONLIBS=1)
    message(STATUS "PYTHON VERSION: " ${PYTHONLIBS_VERSION_STRING})
    message(STATUS "PYTHON INCLUDE: " ${PYTHON_INCLUDE_DIRS})
    message(STATUS "PYTHON LIBRARIES: " ${PYTHON_LIBRARIES})
    include_directories(${PYTHON_INCLUDE_DIRS})
    list(APPEND thirdparty_libraries ${PYTHON_LIBRARIES})
endif ()

# C++17 for std::filesystem (replacing Boost.Filesystem)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 静态库被链接到共享库时需要 -fPIC (Position Independent Code)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Release：优化 + 无调试符号，减小 .so 体积；Debug：保留 -g3 便于 gdb
message(STATUS "CMAKE_BUILD_TYPE: " ${CMAKE_BUILD_TYPE})
if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wuninitialized")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -fno-omit-frame-pointer")
endif()


#add_definitions(-DROS_AVAILABLE=0)
#message(WARNING "BUILDING WITHOUT ROS!")
include(GNUInstallDirs)
set(CATKIN_PACKAGE_LIB_DESTINATION "${CMAKE_INSTALL_LIBDIR}")
set(CATKIN_PACKAGE_BIN_DESTINATION "${CMAKE_INSTALL_BINDIR}")
set(CATKIN_GLOBAL_INCLUDE_DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/open_vins/")

# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${CERES_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${OpenCV_LIBRARIES}
        ${CERES_LIBRARIES}
)

# BLAS/LAPACK 加速 Eigen 稠密运算（矩阵乘、Cholesky 等）
option(USE_BLAS_LAPACK "Use BLAS/LAPACK to accelerate Eigen dense operations" ON)
set(OV_USE_BLAS_LAPACK OFF)
if(USE_BLAS_LAPACK)
  find_package(BLAS QUIET)
  find_package(LAPACK QUIET)
  if(BLAS_FOUND AND LAPACK_FOUND)
    set(OV_USE_BLAS_LAPACK ON)
    message(STATUS "Eigen: Using BLAS/LAPACK for acceleration")
  else()
    message(STATUS "Eigen: BLAS/LAPACK not found, using built-in")
  endif()
endif()

##################################################
# Make the core library
##################################################

# Collect all source files
file(GLOB_RECURSE SRC_LIST_CPP
        "${PROJECT_SOURCE_DIR}/src/cam/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/ceres/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/core/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/cpi/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/data_preprocessing/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/dynamic/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/feat/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/init/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/plot/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/sim/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/state/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/static/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/types/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/track/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/update/*.cpp"
        "${PROJECT_SOURCE_DIR}/src/utils/*.cpp"
)

file(GLOB_RECURSE SRC_LIST_H
        "${PROJECT_SOURCE_DIR}/src/cam/*.h"
        "${PROJECT_SOURCE_DIR}/src/ceres/*.h"
        "${PROJECT_SOURCE_DIR}/src/core/*.h"
        "${PROJECT_SOURCE_DIR}/src/cpi/*.h"
        "${PROJECT_SOURCE_DIR}/src/data_preprocessing/*.h"
        "${PROJECT_SOURCE_DIR}/src/dynamic/*.h"
        "${PROJECT_SOURCE_DIR}/src/feat/*.h"
        "${PROJECT_SOURCE_DIR}/src/init/*.h"
        "${PROJECT_SOURCE_DIR}/src/plot/*.h"
        "${PROJECT_SOURCE_DIR}/src/sim/*.h"
        "${PROJECT_SOURCE_DIR}/src/state/*.h"
        "${PROJECT_SOURCE_DIR}/src/static/*.h"
        "${PROJECT_SOURCE_DIR}/src/types/*.h"
        "${PROJECT_SOURCE_DIR}/src/track/*.h"
        "${PROJECT_SOURCE_DIR}/src/update/*.h"
        "${PROJECT_SOURCE_DIR}/src/utils/*.h"
)

# Add the library
add_library(${PROJECT_NAME}_lib SHARED ${SRC_LIST_CPP} ${SRC_LIST_H})

# Add include directories
target_include_directories(${PROJECT_NAME}_lib
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src)


target_link_libraries(${PROJECT_NAME}_lib PUBLIC ${thirdparty_libraries})

# GCC 7 用 experimental/filesystem 需链接 libstdc++fs
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "8")
  target_link_libraries(${PROJECT_NAME}_lib PRIVATE stdc++fs)
endif()

if(OV_USE_BLAS_LAPACK)
  target_compile_definitions(${PROJECT_NAME}_lib PRIVATE EIGEN_USE_BLAS EIGEN_USE_LAPACK)
  target_link_libraries(${PROJECT_NAME}_lib PRIVATE ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES})
endif()

target_include_directories(${PROJECT_NAME}_lib PUBLIC src/)
install(TARGETS ${PROJECT_NAME}_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##################################################
# Make binary files!
##################################################


add_executable(test_webcam src/test_webcam.cpp)
target_link_libraries(test_webcam ${PROJECT_NAME}_lib ${thirdparty_libraries})
install(TARGETS test_webcam
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test_profile src/test_profile.cpp)
target_link_libraries(test_profile ${PROJECT_NAME}_lib ${thirdparty_libraries})
install(TARGETS test_profile
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(test_cpi_consistency src/test_cpi_consistency.cpp)
target_link_libraries(test_cpi_consistency ${PROJECT_NAME}_lib ${thirdparty_libraries})
install(TARGETS test_cpi_consistency
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)





