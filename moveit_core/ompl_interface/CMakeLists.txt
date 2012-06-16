cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

rosbuild_add_boost_directories()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(OMPL REQUIRED ompl)
#include_directories(${OMPL_INCLUDE_DIRS})
#link_directories(${OMPL_LIBRARY_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME}
  src/ompl_interface.cpp
  src/planning_context_manager.cpp
  src/constraints_library.cpp
  src/model_based_planning_context.cpp
  src/parameterization/model_based_joint_state_space.cpp
  src/parameterization/model_based_state_space.cpp
  src/parameterization/model_based_state_space_factory.cpp
  src/parameterization/joint_space/joint_model_state_space.cpp
  src/parameterization/joint_space/joint_model_state_space_factory.cpp  
  src/parameterization/work_space/pose_model_state_space.cpp  
  src/parameterization/work_space/pose_model_state_space_factory.cpp
  src/detail/threadsafe_state_storage.cpp
  src/detail/state_validity_checker.cpp
  src/detail/projection_evaluators.cpp
  src/detail/goal_union.cpp
  src/detail/constrained_sampler.cpp
  src/detail/constrained_goal_sampler.cpp
  src/detail/ompl_console.cpp
)

rosbuild_add_openmp_flags(${PROJECT_NAME})
#target_link_libraries(${PROJECT_NAME} ${OMPL_LIBRARIES})
rosbuild_link_boost(${PROJECT_NAME} thread date_time serialization system filesystem)

rosbuild_add_gtest(test_state_space test/test_state_space.cpp) 
target_link_libraries(test_state_space ${PROJECT_NAME})
