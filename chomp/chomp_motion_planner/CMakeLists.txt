cmake_minimum_required(VERSION 2.8.3)
project(chomp_motion_planner)

find_package(catkin REQUIRED COMPONENTS roscpp moveit_experimental moveit_core)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES  ${PROJECT_NAME}
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/chomp_cost.cpp
  src/chomp_parameters.cpp
  src/chomp_trajectory.cpp
  src/chomp_optimizer.cpp
  src/chomp_planner.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
