cmake_minimum_required(VERSION 2.8.3)
project(moveit_ros_move_group)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED system filesystem date_time program_options signals thread)
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning
  moveit_ros_manipulation
  actionlib
  roscpp
  tf
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning
    moveit_ros_manipulation)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(SYSTEM
                    ${Boost_INCLUDE_DIRS}
                    )

link_directories(${Boost_LIBRARY_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(move_group_action_server
  src/move_group_capability.cpp
  src/move_group_move_action_capability.cpp
  src/move_group_pick_place_action_capability.cpp
  src/move_group_plan_service_capability.cpp
  src/move_group_execute_service_capability.cpp
  src/move_group_query_planners_service_capability.cpp  
  src/move_group_kinematics_service_capability.cpp
  src/move_group_state_validation_service_capability.cpp
  src/move_group_cartesian_path_service_capability.cpp
  src/move_group.cpp
  )

target_link_libraries(move_group_action_server ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS move_group_action_server RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/load_map scripts/save_map DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
