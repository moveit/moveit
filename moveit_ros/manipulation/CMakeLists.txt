cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_manipulation)

find_package(Boost REQUIRED thread system filesystem date_time program_options)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS
  moveit_core
  moveit_msgs
  moveit_manipulation_msgs
  moveit_ros_planning
  moveit_ros_warehouse
  roscpp
  rosconsole
  tf
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES  
  DEPENDS
    moveit_core
    moveit_msgs
    moveit_manipulation_msgs
    moveit_ros_planning
    moveit_ros_warehouse)

include_directories(pick_place_planner/include)

add_subdirectory(pick_place_planner)
add_subdirectory(moveit_manipulation_testing)
