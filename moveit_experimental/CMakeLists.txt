cmake_minimum_required(VERSION 2.8.3)
project(moveit_experimental)

add_definitions(-std=c++14)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED
COMPONENTS
  control_msgs
  geometry_msgs
  joy_teleop
  moveit_ros_planning_interface
  rosparam_shortcuts
  spacenav_node
  tf2_ros
)
find_package(Eigen3 REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS
  jog_arm/include
)

catkin_package(
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
  CATKIN_DEPENDS
    control_msgs
    geometry_msgs
    joy_teleop
    moveit_ros_planning_interface
    rosparam_shortcuts
    spacenav_node
    tf2_ros
  DEPENDS
    EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${THIS_PACKAGE_INCLUDE_DIRS}
)

add_subdirectory(jog_arm)
