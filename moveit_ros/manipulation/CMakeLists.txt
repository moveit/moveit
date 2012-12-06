cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_manipulation)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread system filesystem date_time program_options)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  moveit_manipulation_msgs
  moveit_ros_planning
  moveit_ros_warehouse
  roscpp
  rosconsole
  tf
)

catkin_package(
  LIBRARIES  
  DEPENDS
    moveit_core
    moveit_msgs
    moveit_manipulation_msgs
    moveit_ros_planning
    moveit_ros_warehouse)

include_directories(pick_place_planner/include)
include_directories(${catkin_INCLUDE_DIR}
                    ${EIGEN_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR})

add_subdirectory(pick_place_planner)
add_subdirectory(moveit_manipulation_testing)
