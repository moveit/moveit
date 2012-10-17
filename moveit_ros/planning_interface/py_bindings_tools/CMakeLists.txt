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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#common commands for building c++ executables and libraries
rosbuild_add_library(moveit_${PROJECT_NAME} src/roscpp_initializer.cpp)
rosbuild_add_boost_directories()
rosbuild_link_boost(moveit_${PROJECT_NAME} thread)


find_package(PythonLibs)
include_directories (${PYTHON_INCLUDE_DIRS})

rosbuild_add_library(_moveit_roscpp_initializer src/roscpp_initializer_pymodule.cpp)
target_link_libraries(_moveit_roscpp_initializer ${PYTHON_LIBRARIES})
set_target_properties(_moveit_roscpp_initializer PROPERTIES OUTPUT_NAME _moveit_roscpp_initializer PREFIX "")
rosbuild_link_boost(_moveit_roscpp_initializer thread python)
