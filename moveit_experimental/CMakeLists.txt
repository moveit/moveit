cmake_minimum_required(VERSION 2.8.3)
project(moveit_experimental)

add_definitions(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#find_package(Boost REQUIRED system filesystem date_time thread iostreams)
find_package(catkin REQUIRED
COMPONENTS
  control_msgs
  geometry_msgs
  moveit_ros_planning_interface
  rosparam_shortcuts
  tf2_ros
#  eigen_conversions
#  eigen_stl_containers
#  geometric_shapes
#  kdl_parser
#  moveit_core
#  moveit_msgs
#  pluginlib
#  rosconsole
#  roslib
#  rostime
#  std_msgs
#  trajectory_msgs
#  visualization_msgs
)
find_package(Eigen3 REQUIRED)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

#find_package(octomap REQUIRED)
#find_package(urdfdom REQUIRED)
#find_package(urdfdom_headers REQUIRED)

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
    tf2_ros
#    eigen_conversions
#    eigen_stl_containers
#    geometric_shapes
#    kdl_parser
#    moveit_core
#    moveit_msgs
#    pluginlib
#    std_msgs
#    trajectory_msgs
#    visualization_msgs
  DEPENDS
    EIGEN3
#    Boost
#    urdfdom
#    urdfdom_headers
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${THIS_PACKAGE_INCLUDE_DIRS}
#  ${Boost_INCLUDE_DIR}
#  ${urdfdom_INCLUDE_DIRS}
#  ${urdfdom_headers_INCLUDE_DIRS}
#  ${OCTOMAP_INCLUDE_DIRS}
)

add_subdirectory(jog_arm)
