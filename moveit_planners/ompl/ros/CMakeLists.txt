cmake_minimum_required(VERSION 2.8.3)
project(moveit_ompl_planners_ros_plugin)

find_package(Boost REQUIRED system filesystem date_time thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS
  moveit_ros_planning
  moveit_ompl_planners_core
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
    moveit_ompl_interface_ros
  DEPENDS
    moveit_core)

include_directories(ompl_interface_ros/include)
add_subdirectory(ompl_interface_ros)
