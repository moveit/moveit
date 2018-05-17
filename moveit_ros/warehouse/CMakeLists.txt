cmake_minimum_required(VERSION 2.8.12)
project(moveit_ros_warehouse)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread system filesystem regex date_time program_options)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning
  roscpp
  rosconsole
  warehouse_ros
  tf2_eigen
  tf2_ros
)

catkin_package(
  LIBRARIES
    moveit_warehouse
  INCLUDE_DIRS
    warehouse/include
  CATKIN_DEPENDS
    moveit_ros_planning
    warehouse_ros)

include_directories(warehouse/include)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(warehouse)
