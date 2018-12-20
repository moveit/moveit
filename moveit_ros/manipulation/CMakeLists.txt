cmake_minimum_required(VERSION 2.8.12)
project(moveit_ros_manipulation)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_move_group
  moveit_msgs
  dynamic_reconfigure
  roscpp
  rosconsole
  tf
  pluginlib
  actionlib
)
find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost REQUIRED thread system filesystem date_time program_options)

generate_dynamic_reconfigure_options("pick_place/cfg/PickPlaceDynamicReconfigure.cfg")

catkin_package(
  INCLUDE_DIRS
    pick_place/include
    move_group_pick_place_capability/include
  LIBRARIES
    moveit_pick_place_planner
  CATKIN_DEPENDS
    moveit_core
    moveit_msgs
    moveit_ros_planning
  DEPENDS
    EIGEN3
)

include_directories(pick_place/include)
include_directories(move_group_pick_place_capability/include)

include_directories(${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM
                    ${EIGEN3_INCLUDE_DIRS})

add_subdirectory(pick_place)
add_subdirectory(move_group_pick_place_capability)

install(FILES
  pick_place_capability_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
