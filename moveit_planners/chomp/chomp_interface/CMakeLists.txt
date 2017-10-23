cmake_minimum_required(VERSION 2.8.3)
project(moveit_planners_chomp)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  moveit_core
  pluginlib
  chomp_motion_planner
  moveit_experimental
)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp moveit_core pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/chomp_interface.cpp src/chomp_planning_context.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(chomp_planner_plugin src/chomp_plugin.cpp)
set_target_properties(chomp_planner_plugin PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(chomp_planner_plugin ${PROJECT_NAME} ${catkin_LIBRARIES})

install(FILES chomp_interface_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/chomp_interface/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} chomp_planner_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS rostest roscpp moveit_ros_planning_interface)

  add_rostest_gtest(chomp_moveit_test
    test/chomp_moveit.test
    test/chomp_moveit_test.cpp)
  target_link_libraries(chomp_moveit_test ${catkin_LIBRARIES})
endif()
