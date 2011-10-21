cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Debug)

rosbuild_init()
rosbuild_add_boost_directories()

rosbuild_add_library(planning_models
  src/kinematic_model.cpp
  src/kinematic_state.cpp
  src/transforms.cpp
  src/conversions.cpp)

# Unit tests
rosbuild_add_gtest(test_kinematic test/test_kinematic.cpp)
target_link_libraries(test_kinematic planning_models)
target_link_libraries(test_kinematic assimp)

rosbuild_add_gtest(test_kinematic_complex test/test_kinematic_complex.cpp)
target_link_libraries(test_kinematic_complex planning_models)
target_link_libraries(test_kinematic_complex assimp)

