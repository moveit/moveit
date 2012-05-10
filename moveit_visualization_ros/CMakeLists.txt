cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
include_directories(${CMAKE_BINARY_DIR})
include_directories(${QT_INCLUDE_DIR})

add_definitions(-DQT_NO_KEYWORDS)

qt4_wrap_cpp(MOC_FILES
  include/moveit_visualization_ros/interactive_object_visualization_widget.h
  include/moveit_visualization_ros/interactive_object_visualization_qt_wrapper.h
  include/moveit_visualization_ros/primitive_object_addition_dialog.h
  include/moveit_visualization_ros/mesh_object_addition_dialog.h
  include/moveit_visualization_ros/planning_group_selection_menu.h
  include/moveit_visualization_ros/planning_scene_file_menu.h
  include/moveit_visualization_ros/planning_scene_database_dialog.h
  include/moveit_visualization_ros/planning_visualization_qt_wrapper.h
)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} 
  src/interactive_marker_helper_functions.cpp
  src/kinematics_group_visualization.cpp
  src/kinematics_start_goal_visualization.cpp 
  src/joint_trajectory_visualization.cpp 
  src/collision_visualization.cpp 
  src/interactive_object_visualization.cpp 
  src/planning_visualization.cpp
  src/interactive_object_visualization_widget.cpp
  src/interactive_object_visualization_qt_wrapper.cpp
  src/primitive_object_addition_dialog.cpp
  src/mesh_object_addition_dialog.cpp
  src/planning_group_selection_menu.cpp
  src/planning_visualization_qt_wrapper.cpp
  src/planning_scene_file_menu.cpp
  src/planning_scene_database_dialog.cpp
  src/moveit_visualizer.cpp
  ${MOC_FILES}
  )
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES})
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(kinematics_visualization_test src/kinematics_visualization_test.cpp)
target_link_libraries(kinematics_visualization_test ${PROJECT_NAME})

rosbuild_add_executable(kinematics_visualization_start_goal_test src/kinematics_visualization_start_goal_test.cpp)
target_link_libraries(kinematics_visualization_start_goal_test ${PROJECT_NAME})

rosbuild_add_executable(joint_trajectory_visualization_test src/joint_trajectory_visualization_test.cpp)
target_link_libraries(joint_trajectory_visualization_test ${PROJECT_NAME})

rosbuild_add_executable(interactive_object_visualization_test src/interactive_object_visualization_test.cpp)
target_link_libraries(interactive_object_visualization_test ${PROJECT_NAME})

#rosbuild_add_executable(planning_components_visualizer src/planning_components_visualizer.cpp)
#target_link_libraries(planning_components_visualizer ${PROJECT_NAME})

#rosbuild_add_executable(planning_components_visualizer_full src/planning_components_visualizer_full.cpp)
#target_link_libraries(planning_components_visualizer_full ${QT_LIBRARIES} ${PROJECT_NAME})

rosbuild_add_executable(moveit_visualizer src/moveit_visualizer_main.cpp)
target_link_libraries(moveit_visualizer ${QT_LIBRARIES} ${PROJECT_NAME})

rosbuild_add_executable(interactive_distance_field src/interactive_distance_field.cpp)
target_link_libraries(interactive_distance_field ${QT_LIBRARIES} ${PROJECT_NAME})
