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

find_package(srdfdom REQUIRED)
include_directories(${srdfdom_INCLUDE_DIRS})

find_library(YAML yaml-cpp REQUIRED)

# Qt Stuff
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)
include_directories(${CMAKE_BINARY_DIR})
include_directories(${QT_INCLUDE_DIR})

# Header files that need Qt Moc pre-processing for use with Qt signals, etc:
set( headers
  src/widgets/navigation_widget.h
  src/widgets/header_widget.h
  src/widgets/setup_assistant_widget.h
  src/widgets/start_screen_widget.h
  src/widgets/planning_groups_widget.h
  src/widgets/double_list_widget.h
  src/widgets/kinematic_chain_widget.h
  src/widgets/group_edit_widget.h
  src/widgets/default_collisions_widget.h
  src/widgets/robot_poses_widget.h
  src/widgets/end_effectors_widget.h
  src/widgets/virtual_joints_widget.h
  src/widgets/configuration_files_widget.h
  src/widgets/setup_screen_widget.h
)

# Convert the Qt Signals and Slots for QWidget events
qt4_wrap_cpp(moc_sources ${headers})

# Tools Library
rosbuild_add_library(${PROJECT_NAME}
  src/tools/compute_default_collisions.cpp
  src/tools/moveit_config_data.cpp
  src/tools/srdf_writer.cpp
)
target_link_libraries(${PROJECT_NAME} ${YAML})

# Main Widgets Library - all screens (navigation options)
rosbuild_add_library(${PROJECT_NAME}_widgets
  src/widgets/start_screen_widget.cpp
  src/widgets/planning_groups_widget.cpp
  src/widgets/double_list_widget.cpp
  src/widgets/kinematic_chain_widget.cpp
  src/widgets/group_edit_widget.cpp
  src/widgets/default_collisions_widget.cpp
  src/widgets/robot_poses_widget.cpp
  src/widgets/end_effectors_widget.cpp
  src/widgets/virtual_joints_widget.cpp
  src/widgets/configuration_files_widget.cpp
)
target_link_libraries(${PROJECT_NAME}_widgets ${PROJECT_NAME})

# Secondary Widgets Library - trying to speed up compile
rosbuild_add_library(${PROJECT_NAME}_secondary_widgets
  src/widgets/navigation_widget.cpp
  src/widgets/header_widget.cpp
  src/widgets/setup_assistant_widget.cpp
  src/widgets/setup_screen_widget.cpp
)
target_link_libraries(${PROJECT_NAME}_secondary_widgets ${PROJECT_NAME})

# Source files for the exectuable
set( sources
  src/setup_assistant_main.cpp
)

# Hide warnings for Boost depreciated stuff - this is necessary for Boost graph, not our fault
add_definitions(-Wno-deprecated)

# Link to boost libraries
rosbuild_link_boost(${PROJECT_NAME} thread program_options filesystem)

# Link libraries
rosbuild_add_executable(setup_assistant ${sources} ${headers} ${moc_sources})
target_link_libraries(setup_assistant ${QT_LIBRARIES} ${PROJECT_NAME} ${PROJECT_NAME}_widgets ${PROJECT_NAME}_secondary_widgets)


