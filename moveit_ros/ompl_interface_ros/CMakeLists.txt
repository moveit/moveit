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


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/ompl_interface_ros.cpp)


rosbuild_add_executable(ompl_planner src/ompl_planner.cpp)
target_link_libraries(ompl_planner ${PROJECT_NAME})

rosbuild_add_executable(call_benchmark src/call_benchmark.cpp)
target_link_libraries(call_benchmark ${PROJECT_NAME})

rosbuild_add_executable(ompl_db src/ompl_db.cpp)
target_link_libraries(ompl_db ${PROJECT_NAME})

# Unit tests
rosbuild_add_gtest(test_planner test/test_planner.cpp)
target_link_libraries(test_planner ${PROJECT_NAME})

rosbuild_add_gtest(test_simple_plan test/test_simple_plan.cpp)
target_link_libraries(test_simple_plan ${PROJECT_NAME})

rosbuild_add_gtest(test_path_constrained_plan test/test_path_constrained_plan.cpp)
target_link_libraries(test_path_constrained_plan ${PROJECT_NAME})

rosbuild_add_gtest(test_dual_arm test/test_dual_arm_plan.cpp)
target_link_libraries(test_dual_arm ${PROJECT_NAME})
