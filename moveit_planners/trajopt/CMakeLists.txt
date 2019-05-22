cmake_minimum_required(VERSION 2.8.12)
project(moveit_planners_trajopt)

# At least C++11 required for TrajOpt
add_compile_options(-std=c++14)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time thread serialization)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  roscpp
  pluginlib
)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
)

# the following include_directory should have include folder of the new planner. For example for ompl, we need to add "ompl_interface/include"
include_directories(${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${TrajOpt_INCLUDE_DIRS})

# Example planning plugin template
add_library(moveit_trajopt_planner_plugin src/trajopt_planner_manager.cpp)
set_target_properties(moveit_trajopt_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")

install(TARGETS
  moveit_trajopt_planner_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(FILES trajopt_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
