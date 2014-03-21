cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_benchmarks)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED COMPONENTS date_time filesystem program_options system thread)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning
  moveit_ros_warehouse
  eigen_conversions
  roscpp
  rosconsole
)

catkin_package(
  INCLUDE_DIRS
    benchmarks/include  
  LIBRARIES
    moveit_benchmark_execution
  CATKIN_DEPENDS
    moveit_ros_planning
    moveit_ros_warehouse
)

include_directories(benchmarks/include)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(benchmarks)
