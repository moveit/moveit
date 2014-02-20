cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_robot_interaction)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)
find_package(catkin REQUIRED COMPONENTS
  moveit_ros_planning
  interactive_markers
  eigen_conversions
  tf_conversions
  tf
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
                    ${catkin_INCLUDE_DIRS})

include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

add_library(${MOVEIT_LIB_NAME}
  src/interactive_marker_helpers.cpp
  src/kinematic_options.cpp
  src/kinematic_options_map.cpp
  src/locked_robot_state.cpp
  src/interaction_handler.cpp
  src/robot_interaction.cpp)
target_link_libraries(${MOVEIT_LIB_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

catkin_add_gtest(locked_robot_state_test test/locked_robot_state_test.cpp)
target_link_libraries(locked_robot_state_test
  ${MOVEIT_LIB_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES})

install(TARGETS ${MOVEIT_LIB_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
