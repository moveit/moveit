cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

set(ROS_BUILD_TYPE Release)

find_package(PkgConfig REQUIRED)
find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
  pkg_check_modules(ASSIMP assimp)
endif()
if (ASSIMP_FOUND)
  if( ${ASSIMP_VERSION} STRGREATER "2.0.0" )
    set(IS_ASSIMP3 1)
    add_definitions(-DIS_ASSIMP3)
    message(STATUS "Found assimp v3")
  else()
    message(STATUS "Found assimp v2")
  endif()
  include_directories(${ASSIMP_INCLUDE_DIRS})
  link_directories(${ASSIMP_LIBRARY_DIRS})
else()
  message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assimping assimp v2")
  set(ASSIMP_LIBRARIES assimp)
  set(ASSIMP_LIBRARY_DIRS)
  set(ASSIMP_CXX_FLAGS)
  set(ASSIMP_CFLAGS_OTHER)
  set(ASSIMP_LINK_FLAGS)
  set(ASSIMP_INCLUDE_DIRS)
  set(IS_ASSIMP3 0) # most likely not
endif()

rosbuild_init()

rosbuild_add_boost_directories()

rosbuild_add_library(robot_self_filter src/self_mask.cpp)
rosbuild_add_openmp_flags(robot_self_filter)
target_link_libraries(robot_self_filter assimp)

rosbuild_add_executable (test_filter src/test_filter.cpp)
rosbuild_add_openmp_flags (test_filter)
target_link_libraries (test_filter robot_self_filter assimp)

rosbuild_add_executable(self_filter src/self_filter.cpp)
rosbuild_add_openmp_flags(self_filter)
target_link_libraries(self_filter robot_self_filter assimp)
rosbuild_link_boost(self_filter signals)
