cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_benchmarks)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread program_options date_time system filesystem)

pkg_check_modules(OGRE OGRE)
link_directories(${OGRE_LIBRARY_DIRS})

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_warehouse
  moveit_ros_visualization
  eigen_conversions
  geometric_shapes
  interactive_markers
  rviz
  tf
  roscpp
  rosconsole
)

catkin_package(
  INCLUDE_DIRS
    benchmarks/include  
  LIBRARIES
    moveit_benchmarks_config
  CATKIN_DEPENDS
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_warehouse
    moveit_ros_visualization
    eigen_conversions
    geometric_shapes
    interactive_markers
    rviz
    tf
)

include_directories(benchmarks/include)
include_directories(benchmark_tool/include)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(benchmarks)
add_subdirectory(benchmark_tool)
