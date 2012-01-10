cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

set(ROS_BUILD_TYPE Debug)

rosbuild_init()

rosbuild_add_boost_directories()

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME}
  src/shapes.cpp
  src/shape_operations.cpp
  src/bodies.cpp
  src/body_operations.cpp)
target_link_libraries(${PROJECT_NAME} assimp qhull)


# Unit tests
rosbuild_add_gtest(test_point_inclusion test/test_point_inclusion.cpp)
target_link_libraries(test_point_inclusion ${PROJECT_NAME})
rosbuild_link_boost(test_point_inclusion system filesystem)
