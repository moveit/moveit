cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Debug)

rosbuild_init()
rosbuild_add_boost_directories()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME}
  src/kinematic_model.cpp
  src/kinematic_model/joint_model.cpp
  src/kinematic_model/fixed_joint_model.cpp
  src/kinematic_model/planar_joint_model.cpp
  src/kinematic_model/floating_joint_model.cpp
  src/kinematic_model/prismatic_joint_model.cpp
  src/kinematic_model/revolute_joint_model.cpp
  src/kinematic_model/link_model.cpp
  src/kinematic_model/joint_model_group.cpp
  src/kinematic_state.cpp
  src/kinematic_state/joint_state.cpp
  src/kinematic_state/link_state.cpp
  src/kinematic_state/attached_body.cpp
  src/kinematic_state/joint_state_group.cpp
  src/transforms.cpp
  src/conversions.cpp)

# Unit tests
rosbuild_add_gtest(test_kinematic test/test_kinematic.cpp)
target_link_libraries(test_kinematic ${PROJECT_NAME})

rosbuild_add_gtest(test_kinematic_complex test/test_kinematic_complex.cpp)
target_link_libraries(test_kinematic_complex ${PROJECT_NAME})

rosbuild_add_gtest(test_transforms test/test_transforms.cpp)
target_link_libraries(test_transforms ${PROJECT_NAME})
