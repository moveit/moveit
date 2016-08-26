cmake_minimum_required(VERSION 2.8.3)
project(chomp_interface)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules 
  roscpp
  moveit_core
  pluginlib
  chomp_motion_planner
  moveit_experimental
)

find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES chomp_interface
  CATKIN_DEPENDS roscpp moveit_core pluginlib
) 

include_directories( 
  include
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
)

add_library(chomp_interface src/chomp_interface.cpp src/chomp_planning_context.cpp)
target_link_libraries(chomp_interface ${catkin_LIBRARIES})

add_library(chomp_planner_plugin src/chomp_plugin.cpp)
target_link_libraries(chomp_planner_plugin chomp_interface ${catkin_LIBRARIES})

