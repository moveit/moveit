cmake_minimum_required(VERSION 2.8.12)
project(moveit_planners_emptyplan)

# At least C++11 required for EmptyPlan
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

include_directories(${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${EmptyPlan_INCLUDE_DIRS})

# Example planning plugin template
add_library(moveit_emptyplan_planner_plugin src/emptyplan_planner_manager.cpp)
set_target_properties(moveit_emptyplan_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_emptyplan_planner_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
  moveit_emptyplan_planner_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

install(FILES emptyplan_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
