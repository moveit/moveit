cmake_minimum_required(VERSION 3.1.3)
project(moveit_ros_warehouse)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
    warehouse_ros
    roscpp
)

include_directories(warehouse/include)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_subdirectory(warehouse)
