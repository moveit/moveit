cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_manipulation)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread system filesystem date_time program_options)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_warehouse
  roscpp
  rosconsole
  tf
)

catkin_package(
  LIBRARIES
    moveit_pick_place_planner
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    moveit_ros_warehouse)

include_directories(pick_place/include)

#include_directories(pick_place_planner/include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(SYSTEM
                    ${EIGEN_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(pick_place)

#add_subdirectory(pick_place_planner)
#add_subdirectory(moveit_manipulation_testing)
