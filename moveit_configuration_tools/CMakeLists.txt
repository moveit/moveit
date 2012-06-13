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

# Qt Stuff
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)
include_directories(${CMAKE_BINARY_DIR})
include_directories(${QT_INCLUDE_DIR})

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set( headers
  src/widgets/setup_assistant_widget.h
  src/widgets/start_screen_widget.h
  src/widgets/compute_default_collisions_widget.h

  # navigation - TODO remove?
  src/widgets/navigation/navs_view.h
  src/widgets/navigation/nav_item.h
  src/widgets/navigation/nav_delegate.h
)

# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(moc_sources ${headers})

# Tools Library
rosbuild_add_library(${PROJECT_NAME}
  src/tools/compute_default_collisions.cpp
  #src/tools/compute_longest_valid_segment_fraction.cpp
  #src/tools/write_srdf.cpp
  src/tools/benchmark_timer.cpp
)

# Widgets Library
rosbuild_add_library(${PROJECT_NAME}_widgets
  src/widgets/setup_assistant_widget.cpp
  src/widgets/start_screen_widget.cpp
  src/widgets/compute_default_collisions_widget.cpp

  # navigation
  src/widgets/navigation/navs_view.cpp
  src/widgets/navigation/nav_item.cpp
  src/widgets/navigation/nav_delegate.cpp
)
target_link_libraries(${PROJECT_NAME}_widgets ${PROJECT_NAME})

# Source files for the exectuable for testing
set( sources
  #src/examples/compute_default_collisions_main.cpp
  src/moveit_setup_assistant.cpp
)

# Create the executable
#rosbuild_add_executable(compute_default_collisions_main ${sources} ${headers} ${moc_sources})
#target_link_libraries(compute_default_collisions_main ${QT_LIBRARIES} ${PROJECT_NAME} ${PROJECT_NAME}_widgets)

rosbuild_add_executable(moveit_setup_assistant ${sources} ${headers} ${moc_sources})
target_link_libraries(moveit_setup_assistant ${QT_LIBRARIES} ${PROJECT_NAME} ${PROJECT_NAME}_widgets)

