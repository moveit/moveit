cmake_minimum_required(VERSION 2.8.3)
project(moveit_experimental)

add_definitions(-std=c++14)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# find_package(Boost REQUIRED system filesystem date_time thread iostreams)
find_package(catkin REQUIRED)
# find_package(catkin REQUIRED
# COMPONENTS
#   moveit_core
#   moveit_msgs
#   resource_retriever
#   kdl_parser
#   geometric_shapes
#   eigen_stl_containers
#   tf2_eigen
#   random_numbers
#   visualization_msgs
#   roslib
#   rostime
#   rosconsole
#   pluginlib
# )
# find_package(Eigen3 REQUIRED)
# find_package(octomap REQUIRED)
# find_package(urdfdom REQUIRED)
# find_package(urdfdom_headers REQUIRED)

set(THIS_PACKAGE_INCLUDE_DIRS)

catkin_package(
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS
  #   moveit_core
  #   geometric_shapes
  #   eigen_stl_containers
  #   moveit_msgs
  #   kdl_parser
  #   pluginlib
  DEPENDS
  #   EIGEN3
  #   Boost
  #   OCTOMAP
  #   urdfdom
  #   urdfdom_headers
)

include_directories(
  ${THIS_PACKAGE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
  ${urdfdom_headers_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  )
