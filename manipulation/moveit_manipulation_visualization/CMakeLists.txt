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

# Uses PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
include_directories(${CMAKE_BINARY_DIR})
include_directories(${QT_INCLUDE_DIR})

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  include/moveit_manipulation_visualization/grasp_evaluation_visualization_dialog.h
  include/moveit_manipulation_visualization/household_object_addition_dialog.h
  include/moveit_manipulation_visualization/object_recognition_qt_service_wrapper.h
  include/moveit_manipulation_visualization/object_recognition_dialog.h
)

rosbuild_add_library(${PROJECT_NAME} 
  src/grasp_evaluation_visualization.cpp
  src/moveit_manipulation_visualizer.cpp
  src/grasp_generator_dummy.cpp
  src/grasp_generator_visualization.cpp
  src/place_generator_dummy.cpp
  src/place_generator_visualization.cpp
  src/place_evaluation_visualization.cpp
  src/grasp_evaluation_visualization_dialog.cpp
  src/household_object_addition_dialog.cpp
  src/object_recognition_qt_service_wrapper.cpp
  src/object_recognition_dialog.cpp
  ${MOC_FILES}
)

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
rosbuild_add_executable(moveit_manipulation_visualizer src/moveit_manipulation_visualizer_main.cpp)
target_link_libraries(moveit_manipulation_visualizer ${PROJECT_NAME})