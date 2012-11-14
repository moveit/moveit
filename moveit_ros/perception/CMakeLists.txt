cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_perception)

find_package(Eigen REQUIRED)
include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})

find_package(Boost REQUIRED thread signals)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS
  moveit_core
  roscpp
  rosconsole
  urdf
  message_filters
  tf
  tf_conversions
  pcl_ros
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES  
    moveit_robot_self_filter
    moveit_occupancy_map_monitor
  INCLUDE_DIRS
    robot_self_filter/include
    occupancy_map_monitor/include
  DEPENDS
    moveit_core)

include_directories(robot_self_filter/include)
include_directories(occupancy_map_monitor/include)

add_subdirectory(robot_self_filter)
add_subdirectory(occupancy_map_monitor)
