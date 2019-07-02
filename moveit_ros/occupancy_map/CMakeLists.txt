cmake_minimum_required(VERSION 2.8.12)
project(moveit_ros_occupancy_map)

add_compile_options(-std=c++14)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)

if(APPLE)
  find_package(X11 REQUIRED)
endif(APPLE)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_msgs
  pluginlib
)

find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS
    occupancy_map_monitor/include
  LIBRARIES
    moveit_occupancy_map_monitor
  CATKIN_DEPENDS
    moveit_core
    moveit_msgs
  DEPENDS
    EIGEN3
    OCTOMAP
)

include_directories(occupancy_map_monitor/include
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    )
include_directories(SYSTEM
                    ${EIGEN3_INCLUDE_DIRS}
                    ${X11_INCLUDE_DIR}
                    )

add_subdirectory(occupancy_map_monitor)
