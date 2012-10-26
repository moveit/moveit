cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)

rosbuild_init()

rosbuild_add_boost_directories()

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(FCL REQUIRED fcl)
#include_directories(${FCL_INCLUDE_DIRS})
#link_directories(${FCL_LIBRARY_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME}
  src/collision_octomap_filter.cpp
  src/collision_world.cpp 
  src/collision_robot.cpp
  src/collision_matrix.cpp
  src/collision_tools.cpp
  src/allvalid/collision_robot.cpp
  src/allvalid/collision_world.cpp
)

rosbuild_add_compile_flags(${PROJECT_NAME})
#target_link_libraries(${PROJECT_NAME} ${FCL_LIBRARIES})
rosbuild_link_boost(${PROJECT_NAME} thread)

