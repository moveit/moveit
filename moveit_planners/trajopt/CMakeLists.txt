cmake_minimum_required(VERSION 2.8.12)
project(moveit_planners_trajopt)

add_compile_options(-std=c++14)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED
    COMPONENTS
      interactive_markers
      moveit_core
      moveit_visual_tools
      moveit_ros_planning
      moveit_ros_planning_interface
      pluginlib
      roscpp

      tesseract_ros
      trajopt
      tesseract_planning
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
      interactivity_utils
      ${PROJECT_NAME}
    INCLUDE_DIRS
    CATKIN_DEPENDS
      roscpp
      pluginlib
      moveit_core
      moveit_visual_tools
      moveit_ros_planning_interface
      interactive_markers
)

# The following include_directory should have include folder of the new planner.
include_directories(
      include
      ${catkin_INCLUDE_DIRS}
   SYSTEM
      ${Boost_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
)


add_library(problem_description src/problem_description.cpp)
set_target_properties(problem_description PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(problem_description ${catkin_LIBRARIES} ${Boost_LIBRARIES})


#add_library(problem_description INTERFACE)
#target_include_directories(problem_description INTERFACE include/)

# add_library(trajopt_interface src/trajopt_interface.cpp)
# set_target_properties (trajopt_interface PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
# target_link_libraries(trajopt_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(${PROJECT_NAME}
  src/trajopt_planning_context.cpp
  src/trajopt_interface.cpp
  #  src/problem_description.cpp
  )
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} problem_description)

# TrajOpt planning plugin
add_library(moveit_trajopt_planner_plugin src/trajopt_planner_manager.cpp)
set_target_properties(moveit_trajopt_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_trajopt_planner_plugin ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_trajopt src/test_trajopt.cpp)
target_link_libraries(test_trajopt  ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#install(TARGETS
#  moveit_trajopt_planner_plugin
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  moveit_trajopt_planner_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES trajopt_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
