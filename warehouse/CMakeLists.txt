cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_warehouse)

include_directories(warehouse/include)

find_package(Boost REQUIRED thread system filesystem regex date_time program_options)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS
  moveit_ros_planning
  roscpp
  rosconsole
  warehouse_ros
  tf
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES  
    moveit_warehouse
  INCLUDE_DIRS
    warehouse/include
  DEPENDS
    moveit_ros_planning
    warehouse_ros)

add_subdirectory(warehouse)
