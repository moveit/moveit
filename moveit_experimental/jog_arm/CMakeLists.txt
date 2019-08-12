cmake_minimum_required(VERSION 2.8.3)
project(jog_arm)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  geometry_msgs
  moveit_ros_planning_interface
  rosparam_shortcuts
  tf2_ros
)
find_package(Eigen3 REQUIRED)


catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
  CATKIN_DEPENDS
    control_msgs
    geometry_msgs
    moveit_ros_planning_interface
    rosparam_shortcuts
    tf2_ros
  DEPENDS
    EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_server
  src/jog_arm/collision_check_thread.cpp
  src/jog_arm/jog_arm_server.cpp
  src/jog_arm/jog_calcs.cpp
  src/jog_arm/jog_ros_interface.cpp
  src/jog_arm/low_pass_filter.cpp
)

add_dependencies(${PROJECT_NAME}_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_server ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(${PROJECT_NAME}_spacenav_to_twist
  src/jog_arm/teleop_examples/spacenav_to_twist.cpp
)
add_dependencies(${PROJECT_NAME}_spacenav_to_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_spacenav_to_twist ${catkin_LIBRARIES})

install(
  TARGETS
    ${PROJECT_NAME}_server
    ${PROJECT_NAME}_spacenav_to_twist
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(ros_pytest REQUIRED)
  add_rostest(test/launch/jog_arm_integration_test.test)
endif()
