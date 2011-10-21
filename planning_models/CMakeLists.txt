cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
rosbuild_add_boost_directories()

set(ROS_BUILD_TYPE Debug)

rosbuild_add_library(planning_models
  src/kinematic_model.cpp
  src/kinematic_state.cpp
  src/transforms.cpp
  src/conversions.cpp)
rosbuild_link_boost(planning_models thread)

# Unit tests
#rosbuild_add_gtest(test_kinematic test/test_kinematic.cpp)
#target_link_libraries(test_kinematic planning_models)
#target_link_libraries(test_kinematic assimp)
#rosbuild_link_boost(test_kinematic thread)

#rosbuild_add_gtest(test_kinematic_complicated test/test_kinematic_complicated.cpp)
#target_link_libraries(test_kinematic_complicated planning_models)
#target_link_libraries(test_kinematic_complicated assimp)
#rosbuild_link_boost(test_kinematic_complicated thread)
