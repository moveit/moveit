cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME}
  src/clamped_cubic_spline_smoother.cpp
  src/fritsch_butland_spline_smoother.cpp
  src/numerical_differentiation_spline_smoother.cpp
  src/linear_spline_velocity_scaler.cpp
  src/cubic_spline_velocity_scaler.cpp
  src/cubic_trajectory.cpp
  src/linear_trajectory.cpp
  src/lspb_trajectory.cpp
  src/trajectory_processing_utils.cpp
  src/iterative_smoother.cpp
  src/cubic_parameterized_trajectory.cpp
  src/cubic_parameterized_spline_velocity_scaler.cpp
  src/spline_shortcutter.cpp
)
rosbuild_add_gtest(test_utils test/test_utils.cpp)
target_link_libraries(test_utils ${PROJECT_NAME})
rosbuild_add_gtest(regress_linear test/regress_linear.cpp)
target_link_libraries(regress_linear ${PROJECT_NAME})
rosbuild_add_gtest(regress_cubic test/regress_cubic.cpp)
target_link_libraries(regress_cubic ${PROJECT_NAME})
#THIS ONE BUILDS, BUT IS BROKEN
#rosbuild_add_gtest(regress_cubic_parameterized test/regress_cubic_parameterized.cpp)
#target_link_libraries(regress_cubic_parameterized ${PROJECT_NAME})
rosbuild_add_gtest(regress_lspb test/regress_lspb.cpp)
target_link_libraries(regress_lspb ${PROJECT_NAME})

rosbuild_add_gtest(test_clamped_cubic_spline_smoother test/test_clamped_cubic_spline_smoother.cpp)
target_link_libraries(test_clamped_cubic_spline_smoother ${PROJECT_NAME})
rosbuild_add_gtest(test_linear_spline_velocity_scaler test/test_linear_spline_velocity_scaler.cpp)
target_link_libraries(test_linear_spline_velocity_scaler ${PROJECT_NAME})

#rosbuild_declare_test(test/test_linear_spline_velocity_scaler)
#rosbuild_add_gtest_build_flags(test/test_linear_spline_velocity_scaler)
#rosbuild_add_rostest(test/test_linear_spline_velocity_scaler.launch)
#target_link_libraries(test/test_linear_spline_velocity_scaler spline_smoother)



#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
