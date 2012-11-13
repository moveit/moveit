cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_planning_interface)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

find_package(Boost REQUIRED thread date_time system filesystem program_options python)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(PythonLibs)
include_directories (${PYTHON_INCLUDE_DIRS})

find_package(catkin COMPONENTS
  moveit_ros_planning
  moveit_ros_warehouse
  eigen_conversions
  interactive_markers
  tf
  roscpp
  actionlib
  rospy
  rosconsole
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES  
    moveit_robot_interaction
    moveit_move_group_interface
  DEPENDS
    moveit_ros_planning
    moveit_ros_warehouse
    )

include_directories(py_bindings_tools/include)
include_directories(robot_interaction/include)
include_directories(move_group_interface/include)

add_subdirectory(py_bindings_tools)
add_subdirectory(robot_interaction)
add_subdirectory(move_group_interface)

