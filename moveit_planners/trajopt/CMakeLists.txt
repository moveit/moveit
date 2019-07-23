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
      rosparam_shortcuts

      trajopt
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
      rosparam_shortcuts
)

# The following include_directory should have include folder of the new planner.
include_directories(
      include
      ${catkin_INCLUDE_DIRS}
   SYSTEM
      ${Boost_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
)



add_library(kinematic_terms src/kinematic_terms.cpp)
set_target_properties(kinematic_terms PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(kinematic_terms ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(problem_description src/problem_description.cpp)
set_target_properties(problem_description PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(problem_description ${catkin_LIBRARIES} ${Boost_LIBRARIES} kinematic_terms)

#add_library(problem_description INTERFACE)
#target_include_directories(problem_description INTERFACE include/)

add_library(trajopt_interface src/trajopt_interface.cpp)
set_target_properties (trajopt_interface PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(trajopt_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES} problem_description)

add_library(${PROJECT_NAME}  src/trajopt_planning_context.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} problem_description trajopt_interface)

# TrajOpt planning plugin
add_library(moveit_trajopt_planner_plugin src/trajopt_planner_manager.cpp)
set_target_properties(moveit_trajopt_planner_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
target_link_libraries(moveit_trajopt_planner_plugin ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(test_trajopt src/test_trajopt.cpp)
target_link_libraries(test_trajopt  ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(test_moveit src/test_moveit.cpp)
target_link_libraries(test_moveit  ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

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


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(trajectory_test test/trajectory_test.test test/trajectory_test.cpp)
  target_link_libraries(trajectory_test
    ${LIBRARY_NAME}
    ${catkin_LIBRARIES}
  )

endif()
