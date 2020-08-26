cmake_minimum_required(VERSION 3.1.3)
project(moveit_servo)

if(NOT "${CMAKE_CXX_STANDARD}")
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(LIBRARY_NAME moveit_servo_cpp_api)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rosparam_shortcuts
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${LIBRARY_NAME}
  CATKIN_DEPENDS
    control_msgs
    geometry_msgs
    moveit_msgs
    moveit_ros_planning_interface
    rosparam_shortcuts
    sensor_msgs
    std_msgs
    std_srvs
    trajectory_msgs
  DEPENDS
    EIGEN3
)

include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

#########################################
## A library providing a C++ interface ##
#########################################

add_library(${LIBRARY_NAME} SHARED
  src/collision_check.cpp
  src/servo_calcs.cpp
  src/servo.cpp
  src/joint_state_subscriber.cpp
  src/low_pass_filter.cpp
)
set_target_properties(${LIBRARY_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
add_dependencies(${LIBRARY_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${LIBRARY_NAME}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

# An example of using the C++ library
add_executable(cpp_interface_example
  src/cpp_interface_example/cpp_interface_example.cpp
)
add_dependencies(cpp_interface_example ${catkin_EXPORTED_TARGETS})
target_link_libraries(cpp_interface_example
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LIBRARY_NAME}
)

############################
## ROS message-based node ##
############################

add_executable(servo_server
  src/servo_server.cpp
)
add_dependencies(servo_server ${catkin_EXPORTED_TARGETS})
target_link_libraries(servo_server
  ${LIBRARY_NAME}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

################################################
## An example of converting joystick commands ##
################################################

add_executable(spacenav_to_twist
  src/teleop_examples/spacenav_to_twist.cpp
)
add_dependencies(spacenav_to_twist ${catkin_EXPORTED_TARGETS})
target_link_libraries(spacenav_to_twist ${catkin_LIBRARIES})

##################
## INSTALLATION ##
##################

install(
  TARGETS
    ${LIBRARY_NAME}
    servo_server
    spacenav_to_twist
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## TESTING ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  # servo_cpp_interface
  add_rostest_gtest(servo_cpp_interface_test
    test/servo_cpp_interface_test.test
    test/servo_cpp_interface_test.cpp
  )
  target_link_libraries(servo_cpp_interface_test
    ${LIBRARY_NAME}
    ${catkin_LIBRARIES}
    ${Eigen_LIBRARIES}
    ${Boost_LIBRARIES}
  )
endif()
