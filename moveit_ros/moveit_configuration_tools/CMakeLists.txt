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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME}
  src/tools/compute_default_collisions.cpp
  src/widgets/compute_default_collisions_widget.cpp
  src/benchmark_timer.cpp
)

# Source files for the exectuable (?)
set( sources
  src/examples/compute_default_collisions_main.cpp
)

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set( headers
  include/moveit_configuration_tools/widgets/compute_default_collisions_widget.h
)

# Compute Default Collision Matrix Tool
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
qt4_wrap_cpp(moc_sources ${headers})
include_directories(${CMAKE_BINARY_DIR})
include_directories(${QT_INCLUDE_DIR})

rosbuild_add_executable(compute_default_collisions_main ${sources} ${headers} ${moc_sources})
target_link_libraries(compute_default_collisions_main ${QT_LIBRARIES})
target_link_libraries(compute_default_collisions_main ${PROJECT_NAME})
