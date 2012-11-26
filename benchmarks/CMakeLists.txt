cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_benchmarks)

include_directories(benchmarks/include)

find_package(Boost REQUIRED thread system filesystem regex date_time program_options)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS
  moveit_ros_planning
  moveit_ros_warehouse
  roscpp
  rosconsole
  REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  INCLUDE_DIRS
    benchmarks/include  
  LIBRARIES
    moveit_benchmarks_config
  DEPENDS
    moveit_ros_planning
    moveit_ros_warehouse
)

add_subdirectory(benchmarks)
