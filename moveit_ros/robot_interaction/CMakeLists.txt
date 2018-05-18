cmake_minimum_required(VERSION 2.8.12)
project(moveit_ros_robot_interaction)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning
  interactive_markers
  tf2
  tf2_eigen
  tf2_geometry_msgs
  tf2_ros
  roscpp
)

set(MOVEIT_LIB_NAME moveit_robot_interaction)

catkin_package(
  LIBRARIES
    ${MOVEIT_LIB_NAME}
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    moveit_ros_planning
    interactive_markers
    )

include_directories(include
                    ${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})

link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(${MOVEIT_LIB_NAME}
  src/interactive_marker_helpers.cpp
  src/kinematic_options.cpp
  src/kinematic_options_map.cpp
  src/locked_robot_state.cpp
  src/interaction_handler.cpp
  src/robot_interaction.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(locked_robot_state_test test/locked_robot_state_test.cpp)
  target_link_libraries(locked_robot_state_test
    ${MOVEIT_LIB_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})
endif()

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
